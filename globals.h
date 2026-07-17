#pragma once

#include <unordered_set>
#include <Eigen/Core>
#include <igl/readOFF.h>

extern int mode;
extern Eigen::MatrixXd V;
extern Eigen::MatrixXi F;
extern std::unordered_set<int> handles;
extern std::unordered_set<int> anchors;
extern std::unordered_set<int> available;
void load_mesh();