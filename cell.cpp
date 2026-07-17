#include "cell.h"
#include "globals.h"


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
    }
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
}
