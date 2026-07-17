#pragma once

#include <unordered_set>
#include <Eigen/Core>
#include <igl/readOFF.h>
#include <igl/adjacency_list.h>


extern int mode;
extern Eigen::MatrixXd V;
extern Eigen::MatrixXi F;
extern std::unordered_set<int> handles;
extern std::unordered_set<int> anchors;
extern std::unordered_set<int> available;
extern std::vector<std::vector<int>> adjacency;
extern std::map<std::pair<int,int>, std::vector<double>> angles;

void load_mesh();