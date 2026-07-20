#pragma once

#include <Eigen/Core>
#include <map>
#include <vector>
#include <utility>

struct Cell {
    int point_idx;
    std::vector<double> weights;
    Eigen::Matrix3d rotation;
    Eigen::MatrixXd P;
    Eigen::MatrixXd P_prim;
    Eigen::MatrixXd D;

    Cell(int point_idx);
    void find_rotation(Eigen::MatrixXd& V_new);
};

void precompute_angles();
