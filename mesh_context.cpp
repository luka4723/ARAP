#include "mesh_context.h"
#include <igl/read_triangle_mesh.h>
#include <igl/adjacency_list.h>
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
    C.row(0).setConstant(0.0);
    C.col(2).setConstant(1.0);
    anchors.clear();
    handles.clear();
    vertex_type.assign(V.rows(), 0);
    L.resize(V.rows(), V.rows());
    mode = 0;

    precompute_angles();
    populate_cells();
    return true;
}

void MeshContext::precompute_angles()
{
    angles.clear();
    for(int i=0; i<F.rows();i++)
    {
        int a = F(i,0);
        int b = F(i,1);
        int c = F(i,2);

        auto add_angle = [&](int x, int y, int opposite)
        {
            int a = std::min(x,y);
            int b = std::max(x,y);

            angles[{a,b}].push_back(cotangent(V, opposite,x,y));
        };

        add_angle(a,b,c);
        add_angle(b,c,a);
        add_angle(a,c,b);
    }
}

void MeshContext::populate_cells()
{
    //auto start = std::chrono::high_resolution_clock::now();
    cells.clear();
    cells.reserve(V.rows());
    for(int i=0;i<V.rows();i++) cells.emplace_back(i, V, adjacency, angles);
    //auto end = std::chrono::high_resolution_clock::now();
    //std::chrono::duration<double> elapsed = end - start; 
    //std::cout << "Time: " << elapsed.count() << " seconds\n";
}

void MeshContext::build_L()
{
    std::vector<Eigen::Triplet<double>> triplets;
    for (int i = 0; i < V.rows(); i++) {
        if (i == selected_vertex || vertex_type[i] == 2) { 
            triplets.emplace_back(i, i, 1.0);
            continue;  
        }

        double val = 0.0;
        int n = 0;
        for (int j : adjacency[i]) {
            double w = cells[i].weights[n];
            val += w; 
            if (j != selected_vertex && vertex_type[j] != 2) triplets.emplace_back(i, j, -w);   
            n++;
        }
        triplets.emplace_back(i, i, val);
    }
    
    L.setFromTriplets(triplets.begin(), triplets.end());
    L.makeCompressed();
    solver.compute(L);
}

void MeshContext::change_vertex_type(int i, int8_t new_type) {
    if (vertex_type[i] == new_type) return;
    switch (vertex_type[i]) {
        case 1: handles.erase(std::remove(handles.begin(), handles.end(), i), handles.end()); break;
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
    V_new = V;
    mode = 0;
    selected_vertex = -1;
    for (Cell& c : cells) c.rotation = Eigen::Matrix3d::Identity();
    is_dragging = false;
    needs_draw = false;
}

double MeshContext::calculate_energy() {
    double E = 0.0;
    C.col(0).setConstant(0.0);
    C.col(2).setConstant(1.0);
    for (const Cell& c : cells) {
        C(c.point_idx,0) = 0.0;
        int i = 0;
        for (int n : adjacency[c.point_idx]) {
            Eigen::Vector3d e1 = (V_new.row(c.point_idx) - V_new.row(n)).transpose();
            Eigen::Vector3d e2 = c.rotation * (V.row(c.point_idx) - V.row(n)).transpose();
            Eigen::Vector3d diff = e1 - e2;
            double local_energy = c.weights[i] * diff.squaredNorm();
            C(c.point_idx,0) += local_energy;
            E += local_energy;
            i++;
        }
        C(c.point_idx,0) = std::clamp(C(c.point_idx,0)*energy_color_coeff, 0.0, 1.0);
        C(c.point_idx,2) -= C(c.point_idx,0);
    }
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
