#include "mesh_context.h"
#include <igl/read_triangle_mesh.h>
#include <igl/adjacency_list.h>
#include <igl/massmatrix.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <filesystem>

bool MeshContext::load_mesh(const std::string& filepath) 
{
    if (!igl::read_triangle_mesh(filepath, V, F)) {
        std::cerr << "Cannot load mesh from: " << filepath << std::endl;
        return false;
    }

    name = std::filesystem::path(filepath).stem().string();
    igl::adjacency_list(F, adjacency);
    V_new = V;
    C.resize(V.rows(),3);
    C.col(0).setConstant(0.0);
    C.col(2).setConstant(1.0);
    anchors.clear();
    handles.clear();
    vertex_type.assign(V.rows(), 0);
    L.resize(V.rows(), V.rows());
    mode = 0;

    selected_vertex = -1;
    last_selected = -2;
    is_dragging = false;
    needs_draw = false;

    populate_cells();
    precompute_angles();
    precompute_voronoi();
    build_L();
    build_left_side();

    return true;
}

void MeshContext::precompute_angles()
{
    halfedges.resize(3 * F.rows());
    for (int i = 0; i < F.rows(); i++) 
    {
        const int a = F(i, 0);
        const int b = F(i, 1);
        const int c = F(i, 2);

        const double w_ab = cotangent(V, c, a, b);
        const double w_bc = cotangent(V, a, b, c);
        const double w_ca = cotangent(V, b, c, a);

        halfedges[3*i]   = {a, b, w_ab, w_ab * (V.row(b) - V.row(a)).transpose()};
        halfedges[3*i+1] = {b, c, w_bc, w_bc * (V.row(c) - V.row(b)).transpose()};
        halfedges[3*i+2] = {c, a, w_ca, w_ca * (V.row(a) - V.row(c)).transpose()};

        for (int v : {a, b, c}) {
            cells[v].he_indices.push_back(3*i);
            cells[v].he_indices.push_back(3*i+1);
            cells[v].he_indices.push_back(3*i+2);
        }
    }
}

void MeshContext::precompute_voronoi()
{
    //TODO: by hand if feasable
    Eigen::SparseMatrix<double> M;
    igl::massmatrix(V, F, igl::MASSMATRIX_TYPE_VORONOI, M);
    Eigen::VectorXd inverse_masses = M.diagonal().cwiseInverse();
    inverse_masses /= inverse_masses.mean();
    normalized_masses = inverse_masses.cwiseInverse();
    M_inv_norm.resize(V.rows(), V.rows());
    M_inv_norm = inverse_masses.asDiagonal();
    M_inv_norm.makeCompressed();
}

void MeshContext::populate_cells()
{
    //auto start = std::chrono::high_resolution_clock::now();
    cells.clear();
    cells.reserve(V.rows());
    for(int i=0;i<V.rows();i++) cells.emplace_back(i, adjacency);
    //auto end = std::chrono::high_resolution_clock::now();
    //std::chrono::duration<double> elapsed = end - start; 
    //std::cout << "Time: " << elapsed.count() << " seconds\n";
}

void MeshContext::build_L()
{
    std::vector<Eigen::Triplet<double>> triplets;
    for (const HalfEdge& he : halfedges) {
        int i = he.from;
        int j = he.to;
        double w = 0.5 * he.weight;

        triplets.emplace_back(i, i,  w);
        triplets.emplace_back(j, j,  w);
        triplets.emplace_back(i, j, -w);
        triplets.emplace_back(j, i, -w);
    }
    L.setFromTriplets(triplets.begin(), triplets.end());
    L.makeCompressed();

    Eigen::MatrixXd vectors = M_inv_norm * L * V;
    for(int i =0; i<V.rows(); i++) cells[i].laplacian_vector = vectors.row(i);
}

void MeshContext::build_left_side()
{
    double lambda_adjusted = lambda / 100.0;
    left_side_no_constraints = lambda_adjusted * L.transpose() * M_inv_norm * L + (1.0-lambda_adjusted) * L;
    left_side_no_constraints.prune(0.0);
    left_side_no_constraints.makeCompressed();
}

void MeshContext::factorize_left_side()
{
    const int n = V.rows();
    std::vector<char> fixed(n, false);

    for (int a : anchors) fixed[a] = true;
    if (selected_vertex >= 0) fixed[selected_vertex] = true;

    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(left_side_no_constraints.nonZeros());

    for (int k = 0; k < left_side_no_constraints.outerSize(); k++)
    {
        for (Eigen::SparseMatrix<double>::InnerIterator it(left_side_no_constraints, k); it; ++it)
        {
            const int i = it.row();
            const int j = it.col();
            if (!fixed[i] && !fixed[j]) triplets.emplace_back(i, j, it.value());
        }
    }

    for (int i = 0; i < n; ++i) {
        if (fixed[i]) triplets.emplace_back(i, i, 1.0);
    }

    left_side.resize(n, n);
    left_side.setFromTriplets(triplets.begin(), triplets.end());
    left_side.makeCompressed();

    solver.compute(left_side);
    if (solver.info() != Eigen::Success) std::cerr << "ARAP factorization failed\n";
}

void MeshContext::apply_constraints_to_rhs(Eigen::MatrixXd& rhs, const Eigen::RowVector3d& handle_target) const
{
    const int n = V.rows();
    std::vector<char> fixed(n, false);

    for (int a : anchors) fixed[a] = true;
    if (selected_vertex >= 0) fixed[selected_vertex] = true;

    auto eliminate_constraint = [&](int c, const Eigen::RowVector3d& target)
    {
        for (Eigen::SparseMatrix<double>::InnerIterator it(left_side_no_constraints, c); it; ++it)
        {
            const int i = it.row();
            if (!fixed[i]) rhs.row(i) -= it.value() * target;
        }
    };

    for (int a : anchors) eliminate_constraint(a, V.row(a));
    eliminate_constraint(selected_vertex, handle_target);
    for (int a : anchors) rhs.row(a) = V.row(a);
    rhs.row(selected_vertex) = handle_target;
}

void MeshContext::change_vertex_type(int i, int8_t new_type) {
    if (vertex_type[i] == new_type) return;
    switch (vertex_type[i]) {
        case 1: 
            handles.erase(std::remove(handles.begin(), handles.end(), i), handles.end());
            if(last_selected == i) last_selected = -2;
            break;
        case 2: anchors.erase(std::remove(anchors.begin(), anchors.end(), i), anchors.end()); break;
    }
    vertex_type[i] = new_type;
    switch (new_type) {
        case 1: handles.push_back(i); break;
        case 2: anchors.push_back(i); break;
    }
}

void MeshContext::reset_vertices() {
    handles.clear();
    anchors.clear();
    vertex_type.assign(V.rows(), 0);
}

void MeshContext::reset_mesh() {
    C.col(0).setConstant(0.0);
    C.col(2).setConstant(1.0);
    V_new = V;
    mode = 0;
    selected_vertex = -1;
    for (Cell& c : cells) c.rotation = Eigen::Matrix3d::Identity();
    is_dragging = false;
    needs_draw = false;
}

 std::array<double,3> MeshContext::calculate_energy() {
    std::array<double, 3> E = {0.0, 0.0, 0.0};
    double lambda_adjusted = lambda / 100.0;
    C.col(0).setConstant(0.0);
    C.col(2).setConstant(1.0);
    Eigen::MatrixXd deformed_laplacians = M_inv_norm * L * V_new;
    for (const Cell& c : cells) {
        double arap_e = 0.0;
        for (int he_index : c.he_indices) {
            HalfEdge& he = halfedges[he_index];
            Eigen::Vector3d e1 = (V_new.row(he.to) - V_new.row(he.from)).transpose();
            Eigen::Vector3d e2 = c.rotation * (V.row(he.to) - V.row(he.from)).transpose();
            Eigen::Vector3d diff = e1 - e2;
            double temp_energy = (0.5 * he.weight/3.0)*diff.squaredNorm();
            arap_e += temp_energy;
        }

        E[0] += arap_e;

        Eigen::Vector3d deformed_laplacian = deformed_laplacians.row(c.point_idx).transpose();
        Eigen::Vector3d rotated_original_laplacian = c.rotation * c.laplacian_vector.transpose();
        Eigen::Vector3d laplacian_difference = deformed_laplacian - rotated_original_laplacian;
        double smooth_arap_e = normalized_masses(c.point_idx) * laplacian_difference.squaredNorm();

        E[1] += smooth_arap_e; 

        double local_e = (1.0 - lambda_adjusted) * arap_e + lambda_adjusted * smooth_arap_e;
        C(c.point_idx,0) = std::clamp(local_e * energy_color_coeff, 0.0, 1.0);
        C(c.point_idx,2) -= C(c.point_idx,0);
    }
    E[2] = (1.0 - lambda_adjusted) * E[0] + lambda_adjusted * E[1];
    return E;
}

void MeshContext::load_config()
{
    std::ifstream file("config.txt");
    if (!file.is_open()) {
        std::cout << "Cannot open file\n";
    }
    std::string temp;
    while (file >> temp) {
        if (temp == name) break;
    }
    if(!file) return;

    vertex_type.assign(V.rows(), 0);
    anchors.clear();
    handles.clear();
    int num;

    file >> temp;
    file >> num;
    for(int i =0;i<num;i++)
    {
        int val;
        file >> val;
        change_vertex_type(val, 1);
    }
    file >> temp;
    file >> num;

    for(int i =0;i<num;i++)
    {
        int val;
        file >> val;
        change_vertex_type(val, 2);
    }
    file.close();
}

void MeshContext::save_config()
{
    std::ifstream in("config.txt");
    std::vector<std::string> lines;
    std::string line;
    bool skip = false;

    while (std::getline(in, line))
    {
        if (line == name)
        {
            skip = true;
            continue;
        }
        if (skip)
        {
            if (line.empty())
                skip = false;
            continue;
        }
        lines.push_back(line);
    }
    in.close();

    std::ofstream out("config.txt", std::ios::trunc);

    for (const auto& l : lines) out << l << '\n';

    out << name << '\n';

    out << "handles " << handles.size();
    for (int h : handles) out << ' ' << h;
    out << '\n';

    out << "anchors " << anchors.size();
    for (int a : anchors) out << ' ' << a;
    out << '\n';
    
    out << '\n';

    out.close();
}
