#include "cell.h"
#include <igl/polar_svd3x3.h>

Cell::Cell(int point_idx, const Eigen::MatrixXd& V, const std::vector<std::vector<int>>& adjacency,
           const std::map<std::pair<int, int>, std::vector<double>>& angles) : point_idx(point_idx) 
{
    int neighbors_size = adjacency[point_idx].size();
    weight_edges.resize(3, neighbors_size);
    weights.resize(neighbors_size);
    neighbors.resize(neighbors_size);
    int k = 0;
    for(int n : adjacency[point_idx])
    {
        neighbors[k] = n;

        int a = std::min(n,point_idx);
        int b = std::max(n,point_idx);
        auto it = angles.find({a, b});
        double alpha = (it != angles.end() && !it->second.empty()) ? it->second[0] : 0.0;
        double beta = (it != angles.end() && it->second.size() == 2) ? it->second[1] : 0.0;
        weights[k] = ((alpha+beta)/2.0); 
        weight_edges.col(k) = weights[k] * (V.row(point_idx) - V.row(n)).transpose();
        k++;    
    }
}
void Cell::find_rotation(const Eigen::MatrixXd& V_new){
    Eigen::Matrix3d S = Eigen::Matrix3d::Zero();
    Eigen::Vector3d center_of_cell = V_new.row(point_idx);
    Eigen::Vector3d e_prim;
    for(int n = 0; n < weight_edges.cols(); n++)
    {
        e_prim[0] = center_of_cell(0) - V_new(neighbors[n],0);
        e_prim[1] = center_of_cell(1) - V_new(neighbors[n],1);
        e_prim[2] = center_of_cell(2) - V_new(neighbors[n],2);

        S(0,0) += weight_edges(0,n) * e_prim(0);
        S(0,1) += weight_edges(0,n) * e_prim(1);
        S(0,2) += weight_edges(0,n) * e_prim(2);

        S(1,0) += weight_edges(1,n) * e_prim(0);
        S(1,1) += weight_edges(1,n) * e_prim(1);
        S(1,2) += weight_edges(1,n) * e_prim(2);

        S(2,0) += weight_edges(2,n) * e_prim(0);
        S(2,1) += weight_edges(2,n) * e_prim(1);
        S(2,2) += weight_edges(2,n) * e_prim(2);
    }
    igl::polar_svd3x3(S, rotation);
    rotation.transposeInPlace();
}

double cotangent(const Eigen::MatrixXd& V, int a, int b, int c)
{
    Eigen::Vector3d A = V.row(a);
    Eigen::Vector3d B = V.row(b);
    Eigen::Vector3d C = V.row(c);

    Eigen::Vector3d u = B - A;
    Eigen::Vector3d v = C - A;

    return u.dot(v) / u.cross(v).norm();
}
