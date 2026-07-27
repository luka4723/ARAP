#include "solver.h"
#include "globals.h"
#include "cell.h"
#include "point_manager.h"
#include <igl/unproject.h>
#include <Eigen/SparseCholesky>
#include <Eigen/Sparse>

bool is_dragging = false;
Eigen::Vector3d drag_plane_point;
Eigen::Vector3d drag_plane_normal;
Eigen::MatrixXd b;
bool needs_draw = false;
double x,y;
igl::ARAPData test;
Eigen::MatrixXd bc;

void build_b()
{
    b = Eigen::MatrixXd::Zero(V.rows(), 3);
    for (int i = 0; i < V.rows(); i++) {
        auto& VrowI = V.row(i);
        
        if (i == selected_vertex || vertex_type[i] == 2)
        {
            b.row(i) = (i == selected_vertex) ? V_new.row(i) : VrowI;
            continue;
        }

        Eigen::Vector3d row = Eigen::Vector3d::Zero();
        int n = 0;
        for (int j : adjacency[i]) {
            Cell& ci = cells[i];
            Cell& cj = cells[j];
            double w = ci.weights[n];
            auto& VrowJ = V.row(j);

            Eigen::Matrix3d R = 0.5 * (ci.rotation + cj.rotation);
            Eigen::Vector3d e = (VrowI - VrowJ).transpose();
            row += w * (R * e);            
            
            if (j == selected_vertex || vertex_type[j] == 2) { 
                Eigen::Vector3d target = j == selected_vertex ? V_new.row(j).transpose() : VrowJ.transpose();
                row += w * target;
            }
            n++;
        }
        b.row(i) = row;
    }
}

Eigen::RowVector3d mouse_to_plane(const igl::opengl::glfw::Viewer& viewer, double mouse_x, 
                                  double mouse_y, const Eigen::Vector3d& plane_point, 
                                  const Eigen::Vector3d& plane_normal)
{
    double x = mouse_x;
    double y = viewer.core().viewport(3) - mouse_y;

    Eigen::Vector3d pt_near, pt_far;
    
    igl::unproject(Eigen::Vector3d(x, y, 0.0), viewer.core().view, viewer.core().proj, viewer.core().viewport, pt_near);
    igl::unproject(Eigen::Vector3d(x, y, 1.0), viewer.core().view, viewer.core().proj, viewer.core().viewport, pt_far);

    Eigen::Vector3d ray_origin = pt_near;
    Eigen::Vector3d ray_direction = (pt_far - pt_near).normalized();

    double denom = ray_direction.dot(plane_normal);
    
    if (std::abs(denom) < 1e-6) return plane_point.transpose(); 

    double t = (plane_point - ray_origin).dot(plane_normal) / denom;
    
    Eigen::Vector3d intersection = ray_origin + t * ray_direction;
    
    return intersection.transpose(); 
    
}

double calculate_energy()
{
    double E = 0.0;
    for(Cell& c : cells)
    {
        int i = 0;
        for(int n: adjacency[c.point_idx])
        {
            Eigen::Vector3d e1 =
            (V_new.row(c.point_idx) - V_new.row(n)).transpose();

            Eigen::Vector3d e2 =
                c.rotation * (V.row(c.point_idx) - V.row(n)).transpose();

            Eigen::Vector3d diff = e1 - e2;

            E += c.weights[i] * diff.squaredNorm();
            i++;
        }
    }
    return E;
}

void solver_mouse_down(const igl::opengl::glfw::Viewer& viewer, int button)
{
    if(button != GLFW_MOUSE_BUTTON_LEFT) return;

    selected_vertex = point_picker(viewer);
    if(selected_vertex == -1 || vertex_type[selected_vertex] != 1)
    {
        selected_vertex = -1; 
        return;
    }
    build_L();
    drag_plane_point = V_new.row(selected_vertex);
    drag_plane_normal = viewer.core().view.block<3,3>(0,0).row(2).cast<double>();

    if(algorithm == 1){
        Eigen::VectorXi b(anchors.size()+1);
        bc.resize(anchors.size()+1, 3);
        int i =0;   
        for (int n : anchors) {
            b(i)=n;
            bc.row(i) = V.row(n);
            i++;
        }
        b(i) = selected_vertex;
        bc.row(i) = V.row(selected_vertex);

        test.energy= igl::ARAP_ENERGY_TYPE_SPOKES;
        igl::arap_precomputation(V,F,3,b,test);
    }
    is_dragging = true;
}


bool solver_mouse_move(igl::opengl::glfw::Viewer& /*viewer*/, double xx, double yy)
{
    if(!is_dragging || selected_vertex == -1) return false;
    needs_draw = true;
    x = xx;
    y = yy;

    return true;
}

void solver_pre_draw(igl::opengl::glfw::Viewer& viewer)
{
    if(needs_draw){
        energy_flag = true;
        V_new.row(selected_vertex) = mouse_to_plane(viewer, x, y, drag_plane_point, drag_plane_normal);
        
        if (algorithm == 0){
            for(int i=0; i<number_of_iterations;i++)
            {
                // auto start = std::chrono::high_resolution_clock::now();
                for(int i=0;i<V.rows();i++) cells[i].find_rotation(V_new);
                // auto end = std::chrono::high_resolution_clock::now();
                // std::chrono::duration<double> elapsed = end - start; 
                // std::cout << "Time for rotations: " << elapsed.count() << " seconds\n";

                // start = std::chrono::high_resolution_clock::now();
                build_b();
                // end = std::chrono::high_resolution_clock::now();
                // elapsed = end - start; 
                // std::cout << "Time for building b: " << elapsed.count() << " seconds\n";

                // start = std::chrono::high_resolution_clock::now();
                V_new = solver.solve(b);
                // end = std::chrono::high_resolution_clock::now();
                // elapsed = end - start; 
                // std::cout << "Time for solver: " << elapsed.count() << " seconds\n";
            }
        }
        else
        {
            bc.row(bc.rows()-1) = mouse_to_plane(viewer, x, y, drag_plane_point, drag_plane_normal);
            igl::arap_solve(bc,test,V_new);

            //for calculating energy
            for(int i=0;i<V.rows();i++) cells[i].find_rotation(V_new);
        }
        
        viewer.data().set_vertices(V_new);
        draw_vertices(viewer, false, true, true);
        needs_draw = false;
    }
}

void solver_mouse_up(igl::opengl::glfw::Viewer& /*viewer*/, int button)
{
    if(button == GLFW_MOUSE_BUTTON_LEFT)
    {
        is_dragging = false;
        needs_draw = false;
        selected_vertex = -1;
    }    
}