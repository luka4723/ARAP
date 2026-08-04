#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <igl/arap.h>
#include <vector>
#include <map>
#include <string>
#include <cstdint>
#include "cell.h"



struct MeshContext {
    std::string name;

    Eigen::MatrixXd V;
    Eigen::MatrixXd V_new;
    Eigen::MatrixXi F;
    Eigen::MatrixXd C;
    Eigen::SparseMatrix<double> M_inv_norm;
    
    std::vector<std::vector<int>> adjacency;
    std::vector<HalfEdge> halfedges;
    std::vector<Cell> cells;

    std::vector<int> handles;
    std::vector<int> anchors;
    std::vector<int8_t> vertex_type; 

    Eigen::SparseMatrix<double> L;
    Eigen::SparseMatrix<double> left_side;
    Eigen::SparseMatrix<double> left_side_no_constraints;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    igl::ARAPData libigl_solver;
    Eigen::MatrixXd libigl_bc;

    int mode = 0; 
    int selected_vertex = -1;
    int number_of_iterations = 5;
    int energy_color_coeff = 50;
    int lambda = 0  ;
    int algorithm = 0; // 0: Custom ARAP, 1: libigl ARAP
    bool is_dragging = false;
    bool needs_draw = false;
    double mouse_x = 0.0;
    double mouse_y = 0.0;

    Eigen::Vector3d drag_plane_point;
    Eigen::Vector3d drag_plane_normal;

    bool load_mesh(const std::string& filepath);
    void load_config();
    void save_config();
    void precompute_angles();
    void precompute_voronoi();
    void populate_cells();
    void build_L();
    void build_left_side();
    void factorize_left_side();
    void apply_constraints_to_rhs(Eigen::MatrixXd& rhs, const Eigen::RowVector3d& handle_target) const;
    void change_vertex_type(int i, int8_t new_type);
    void reset_vertices();
    void reset_mesh();
    double calculate_energy();
};
