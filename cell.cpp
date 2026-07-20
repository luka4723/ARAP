#include "cell.h"
#include "globals.h"

Eigen::Matrix3d SVD_to_Rotation(Eigen::MatrixXd A)
{
 Eigen::JacobiSVD<Eigen::Matrix3d> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    Eigen::Matrix3d R = V * U.transpose();

    if (R.determinant() < 0) {
        Eigen::Matrix3d U_fixed = U;
        U_fixed.col(2) *= -1;
        R = V * U_fixed.transpose();
    }
    return R;
}

void populate_cells()
{
    auto start = std::chrono::high_resolution_clock::now();
    for(int i=0;i<V.rows();i++) cells.emplace_back(i);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start; 
    //std::cout << "Time: " << elapsed.count() << " seconds\n";
    build_L();
}

Cell::Cell(int point_idx) : point_idx(point_idx) {
    for(int n : adjacency[point_idx])
    {
        int a = std::min(n,point_idx);
        int b = std::max(n,point_idx);
        double alpha = angles[{a,b}][0];
        double beta;
        if(angles[{a,b}].size() == 2 ) beta = angles[{a,b}][1];
        else beta = 0;
        weights.push_back((alpha+beta)/2);
        
        int n = adjacency[point_idx].size();
        P.resize(3, n);
        P_prim.resize(3, n);
        D.resize(n, n);
        D.setZero();
    }
}
void Cell::find_rotation(Eigen::MatrixXd& V_new){
    
    int n = 0;
    for(int i: adjacency[point_idx])
    {
        P.col(n) = (V.row(point_idx) - V.row(i)).transpose();
        P_prim.col(n) = (V_new.row(point_idx) - V_new.row(i)).transpose();;
        D(n,n) = weights[n];
        n++;
    }
    Eigen::MatrixXd S = P * D * P_prim.transpose();
    rotation = SVD_to_Rotation(S);
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
