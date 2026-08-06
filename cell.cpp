#include "cell.h"
#include <igl/polar_svd3x3.h>
#include <cstddef>
#include <Eigen/Geometry>

Cell::Cell(int point_idx, const std::vector<std::vector<int>>& adjacency) : point_idx(point_idx)
{
    he_indices.reserve(3 * adjacency[point_idx].size());
}

void Cell::find_rotation(const Eigen::MatrixXd& V_new, const std::vector<HalfEdge>& halfedges){
    Eigen::Matrix3d S = Eigen::Matrix3d::Zero();

    for (std::size_t k = 0; k < he_indices.size(); k += 3) {
        const HalfEdge& he0 = halfedges[he_indices[k]];
        const HalfEdge& he1 = halfedges[he_indices[k + 1]];
        const HalfEdge& he2 = halfedges[he_indices[k + 2]];

        const Eigen::RowVector3d e0_prim = V_new.row(he0.to) - V_new.row(he0.from);
        const Eigen::RowVector3d e1_prim = V_new.row(he1.to) - V_new.row(he1.from);
        const Eigen::RowVector3d e2_prim = V_new.row(he2.to) - V_new.row(he2.from);

        S.noalias() += he0.weight_edge * e0_prim;
        S.noalias() += he1.weight_edge * e1_prim;
        S.noalias() += he2.weight_edge * e2_prim;
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
