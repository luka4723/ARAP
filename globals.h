#pragma once

#include <unordered_set>
#include <Eigen/Core>
#include <igl/readOFF.h>
#include <igl/adjacency_list.h>
#include <igl/opengl/glfw/Viewer.h>
#include "cell.h"
#include <igl/arap.h>



extern int mode;
extern Eigen::MatrixXd V;
extern Eigen::MatrixXi F;
extern std::vector<int> handles;
extern std::vector<int> anchors;
extern std::vector<int8_t> vertex_type;
extern std::vector<std::vector<int>> adjacency;
extern std::map<std::pair<int,int>, std::vector<double>> angles;
extern igl::opengl::glfw::Viewer viewer;
extern std::vector<Cell> cells;
extern Eigen::SparseMatrix<double> L;
extern Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
extern int selected_vertex;
extern Eigen::MatrixXd V_new;


bool load_mesh();
void build_L();
