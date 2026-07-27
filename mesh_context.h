#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <igl/arap.h>
#include <vector>
#include <map>
#include <string>
#include <cstdint>
#include "Cell.h"



struct MeshContext {
    Eigen::MatrixXd V;
    Eigen::MatrixXd V_new;
    Eigen::MatrixXi F;
    
    std::vector<std::vector<int>> adjacency;
    std::map<std::pair<int, int>, std::vector<double>> angles;
    std::vector<Cell> cells;

    std::vector<int> handles;
    std::vector<int> anchors;
    std::vector<int8_t> vertex_type; 

    Eigen::SparseMatrix<double> L;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    igl::ARAPData libigl_solver;
    Eigen::MatrixXd libigl_bc;

    int mode = 0; 
    int selected_vertex = -1;
    int number_of_iterations = 5;
    int algorithm = 0; // 0: Custom ARAP, 1: libigl ARAP
    bool is_dragging = false;
    bool needs_draw = false;
    double mouse_x = 0.0;
    double mouse_y = 0.0;

    Eigen::Vector3d drag_plane_point;
    Eigen::Vector3d drag_plane_normal;

    bool load_mesh(const std::string& filepath, 
                   const std::vector<int>& initial_anchors = {}, 
                   const std::vector<int>& initial_handles = {});
    void precompute_angles();
    void populate_cells();
    void build_L();
    void change_vertex_type(int i, int8_t new_type);
    void reset_all();
    double calculate_energy() const;
};
