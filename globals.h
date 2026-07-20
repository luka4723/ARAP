#pragma once

#include <unordered_set>
#include <Eigen/Core>
#include <igl/readOFF.h>
#include <igl/adjacency_list.h>
#include <igl/opengl/glfw/Viewer.h>
#include "cell.h"



extern int mode;
extern Eigen::MatrixXd V;
extern Eigen::MatrixXi F;
extern std::unordered_set<int> handles;
extern std::unordered_set<int> anchors;
extern std::unordered_set<int> available;
extern std::vector<std::vector<int>> adjacency;
extern std::map<std::pair<int,int>, std::vector<double>> angles;
extern igl::opengl::glfw::Viewer viewer;
extern std::vector<Cell> cells;
extern Eigen::SparseMatrix<double> L;
extern Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
extern int selected_vertex;


void load_mesh();
void build_L();
