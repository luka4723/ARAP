#pragma once

#include <Eigen/Core>
#include <vector>

struct HalfEdge {
    int from;
    int to;
    double weight;
    Eigen::Vector3d weight_edge;
};

struct Cell {
    int point_idx;
    std::vector<int> he_indices;
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::RowVector3d laplacian_vector;


    Cell(int point_idx, const std::vector<std::vector<int>>& adjacency);
    void find_rotation(const Eigen::MatrixXd& V_new, const std::vector<HalfEdge>& halfedges);
};

double cotangent(const Eigen::MatrixXd& V, int a, int b, int c);
