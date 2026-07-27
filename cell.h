#pragma once

#include <Eigen/Core>
#include <map>
#include <vector>
#include <utility>

struct Cell {
    int point_idx;
    std::vector<int> neighbors;
    Eigen::VectorXd weights;
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd weight_edges;


    Cell(int point_idx, const Eigen::MatrixXd& V, const std::vector<std::vector<int>>& adjacency, 
         const std::map<std::pair<int, int>, std::vector<double>>& angles);
    void find_rotation(const Eigen::MatrixXd& V_new);
};
double cotangent(const Eigen::MatrixXd& V, int a, int b, int c);
