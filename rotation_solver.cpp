#include "rotation_solver.h"
#include "globals.h"
#include "point_manager.h"
#include <igl/unproject.h>


bool is_dragging = false;
int selected_vertex = -1;
Eigen::MatrixXd V_new;
Eigen::Vector3d drag_plane_point;
Eigen::Vector3d drag_plane_normal;

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

bool solver_mouse_move(igl::opengl::glfw::Viewer& viewer, double x, double y)
{
    if(!is_dragging || selected_vertex == -1) return false;

    V_new.row(selected_vertex) = mouse_to_plane(viewer, x, y, drag_plane_point, drag_plane_normal);
    V = V_new;
    viewer.data().set_vertices(V);
    draw_vertices(viewer, false, true, true);
    return true;
}
void solver_mouse_up(igl::opengl::glfw::Viewer& viewer, int button)
{
    if(button == GLFW_MOUSE_BUTTON_LEFT)
    {
        is_dragging = false;
        selected_vertex = -1;
    }    
}