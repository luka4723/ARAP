#include "solver.h"
#include "globals.h"
#include "cell.h"
#include "point_manager.h"
#include <igl/unproject.h>
#include <Eigen/SparseCholesky>
#include <Eigen/Sparse>


const int NUMBER_OF_ITERATIONS = 5;
bool is_dragging = false;

Eigen::MatrixXd V_new;
Eigen::Vector3d drag_plane_point;
Eigen::Vector3d drag_plane_normal;
Eigen::MatrixXd b;
bool needs_draw = false;
double x,y;

void build_b()
{
    b = Eigen::MatrixXd::Zero(V.rows(), 3);
    for (int i = 0; i < V.rows(); i++) {
        bool is_handle = (handles.find(i) != handles.end());
        bool is_anchor = (anchors.find(i) != anchors.end());
        
        if (is_handle || is_anchor) { 
            b.row(i) = is_handle ? V_new.row(i) : V.row(i);
            continue;  
        }

        Eigen::Vector3d row = Eigen::Vector3d::Zero();
        int n = 0;
        for (int j : adjacency[i]) {
            double w = cells[i].weights[n];
            row += (0.5 * w * (cells[i].rotation + cells[j].rotation) * (V.row(i) - V.row(j)).transpose()).transpose();
            bool j_is_handle = (handles.find(j) != handles.end());
            bool j_is_anchor = (anchors.find(j) != anchors.end());
            
            if (j_is_handle || j_is_anchor) { 
                Eigen::Vector3d target = j_is_handle ? V_new.row(j).transpose() : V.row(j).transpose();
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

void solver_mouse_down(igl::opengl::glfw::Viewer& viewer, int button)
{
    if(button != GLFW_MOUSE_BUTTON_LEFT) return;

    selected_vertex = point_picker(viewer);
    if(selected_vertex == -1 || handles.find(selected_vertex) == handles.end())
    {
        selected_vertex = -1; 
        return;
    }
    V_new = V;
    drag_plane_point = V.row(selected_vertex);
    drag_plane_normal = viewer.core().view.block<3,3>(0,0).row(2).cast<double>();
    is_dragging = true;
}

bool solver_mouse_move(igl::opengl::glfw::Viewer& viewer, double xx, double yy)
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
        V_new.row(selected_vertex) = mouse_to_plane(viewer, x, y, drag_plane_point, drag_plane_normal);
        
        for(int i=0; i<NUMBER_OF_ITERATIONS;i++)
        {
            for(int i=0;i<V.rows();i++) cells[i].find_rotation(V_new);
            build_b();
            V_new = solver.solve(b);
        }
        
        viewer.data().set_vertices(V_new);
        draw_vertices(viewer, false, true, true);
        needs_draw = false;
    }
}

void solver_mouse_up(igl::opengl::glfw::Viewer& viewer, int button)
{
    if(button == GLFW_MOUSE_BUTTON_LEFT)
    {
        is_dragging = false;
        needs_draw = false;
        selected_vertex = -1;
        // if(V_new.size() != 0 || V != V_new)
        // {
        //     V = V_new;
        //     draw_vertices(viewer, false, true, true);
        // }
    }    
}