#pragma once

#include <Eigen/Core>
#include <map>
#include <vector>
#include <utility>

struct Cell {
    int point_idx;
    std::vector<int> neighbors;
    Eigen::VectorXd weights;
    Eigen::Matrix3d rotation;
    Eigen::MatrixXd weight_edges;


    Cell(int point_idx);
    void find_rotation(const Eigen::MatrixXd& V_new);
};

void precompute_angles();
