#include "mesh_context.h"
#include <igl/read_triangle_mesh.h>
#include <igl/adjacency_list.h>
#include <iostream>
#include <algorithm>

bool MeshContext::load_mesh(const std::string& filepath) 
{
    if (!igl::read_triangle_mesh(filepath, V, F)) {
        std::cerr << "Cannot load mesh from: " << filepath << std::endl;
        return false;
    }

    igl::adjacency_list(F, adjacency);
    V_new = V;
    anchors.clear();
    handles.clear();
    vertex_type.assign(V.rows(), 0);
    L.resize(V.rows(), V.rows());
    mode = 0;

    // for (int n : initial_anchors) {
    //     anchors.push_back(n);
    //     vertex_type[n] = 2;
    // }
    // for (int n : initial_handles) {
    //     handles.push_back(n);
    //     vertex_type[n] = 1;
    // }
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

double MeshContext::calculate_energy() const {
    double E = 0.0;
    for (const Cell& c : cells) {
        int i = 0;
        for (int n : adjacency[c.point_idx]) {
            Eigen::Vector3d e1 = (V_new.row(c.point_idx) - V_new.row(n)).transpose();
            Eigen::Vector3d e2 = c.rotation * (V.row(c.point_idx) - V.row(n)).transpose();
            Eigen::Vector3d diff = e1 - e2;

            E += c.weights[i] * diff.squaredNorm();
            i++;
        }
    }
    return E;
}
