#include "cell.h"
#include "globals.h"

void populate_cells()
{
    //auto start = std::chrono::high_resolution_clock::now();
    cells.reserve(V.rows());
    for(int i=0;i<V.rows();i++) cells.emplace_back(i);
    //auto end = std::chrono::high_resolution_clock::now();
    //std::chrono::duration<double> elapsed = end - start; 
    //std::cout << "Time: " << elapsed.count() << " seconds\n";
}

Cell::Cell(int point_idx) : point_idx(point_idx) {
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
        double alpha = angles[{a,b}][0];
        double beta = 0;
        if(angles[{a,b}].size() == 2 ) beta = angles[{a,b}][1];
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

double cotangent(int a, int b, int c)
{
    Eigen::Vector3d A = V.row(a);
    Eigen::Vector3d B = V.row(b);
    Eigen::Vector3d C = V.row(c);

    Eigen::Vector3d u = B - A;
    Eigen::Vector3d v = C - A;

    return u.dot(v) / u.cross(v).norm();
}

void precompute_angles()
{
    for(int i=0; i<F.rows();i++)
    {
        int a = F(i,0);
        int b = F(i,1);
        int c = F(i,2);

        auto add_angle = [&](int x, int y, int opposite)
        {
            int a = std::min(x,y);
            int b = std::max(x,y);

            angles[{a,b}].push_back(cotangent(opposite,x,y));
        };

        add_angle(a,b,c);
        add_angle(b,c,a);
        add_angle(a,c,b);
    }
    populate_cells();
}
