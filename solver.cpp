#include "solver.h"
#include <igl/unproject.h>

void build_b(MeshContext& context, Eigen::MatrixXd& b)
{
    b = Eigen::MatrixXd::Zero(context.V.rows(), 3);
    for (int i = 0; i < context.V.rows(); i++) {
        auto& VrowI = context.V.row(i);
        
        if (i == context.selected_vertex || context.vertex_type[i] == 2)
        {
            b.row(i) = (i == context.selected_vertex) ? context.V_new.row(i) : VrowI;
            continue;
        }

        Eigen::Vector3d row = Eigen::Vector3d::Zero();
        int n = 0;
        for (int j : context.adjacency[i]) {
            Cell& ci = context.cells[i];
            Cell& cj = context.cells[j];
            double w = ci.weights[n];
            auto& VrowJ = context.V.row(j);

            Eigen::Matrix3d R = 0.5 * (ci.rotation + cj.rotation);
            Eigen::Vector3d e = (VrowI - VrowJ).transpose();
            row += w * (R * e);            
            
            if (j == context.selected_vertex || context.vertex_type[j] == 2) { 
                Eigen::Vector3d target = j == context.selected_vertex ? context.V_new.row(j).transpose() : VrowJ.transpose();
                row += w * target;
            }
            n++;
        }
        b.row(i) = row;
    }
}

Eigen::RowVector3d mouse_to_plane(double mouse_x, double mouse_y, const Eigen::Matrix4f& view,
                                  const Eigen::Matrix4f& proj, const Eigen::Vector4f& viewport,
                                  const Eigen::Vector3d& plane_point, const Eigen::Vector3d& plane_normal)
{
    double x = mouse_x;
    double y = viewport(3) - mouse_y;

    Eigen::Vector3d pt_near, pt_far;
    
    igl::unproject(Eigen::Vector3d(x, y, 0.0), view, proj, viewport, pt_near);
    igl::unproject(Eigen::Vector3d(x, y, 1.0), view, proj, viewport, pt_far);

    Eigen::Vector3d ray_origin = pt_near;
    Eigen::Vector3d ray_direction = (pt_far - pt_near).normalized();

    double denom = ray_direction.dot(plane_normal);
    
    if (std::abs(denom) < 1e-6) return plane_point.transpose(); 

    double t = (plane_point - ray_origin).dot(plane_normal) / denom;

    Eigen::Vector3d intersection = ray_origin + t * ray_direction;
    return intersection.transpose(); 
}

void prepare_drag_session(MeshContext& context, const Eigen::Matrix4f& view_matrix) {
    context.build_L();
    context.drag_plane_point = context.V_new.row(context.selected_vertex);
    context.drag_plane_normal = view_matrix.block<3, 3>(0, 0).row(2).cast<double>();

    if (context.algorithm == 1) {
        context.libigl_solver.max_iter = context.number_of_iterations;
        Eigen::VectorXi b_indices(context.anchors.size() + 1);
        context.libigl_bc.resize(context.anchors.size() + 1, 3);
        
        int i = 0;
        for (int n : context.anchors) {
            b_indices(i) = n;
            context.libigl_bc.row(i) = context.V.row(n);
            i++;
        }
        b_indices(i) = context.selected_vertex;
        context.libigl_bc.row(i) = context.V.row(context.selected_vertex);

        context.libigl_solver.energy = igl::ARAP_ENERGY_TYPE_SPOKES;
        igl::arap_precomputation(context.V, context.F, 3, b_indices, context.libigl_solver);
    }
    context.is_dragging = true;
}

void solve_arap_step(MeshContext& context, const Eigen::RowVector3d& new_handle_pos) {
    context.V_new.row(context.selected_vertex) = new_handle_pos;

    if (context.algorithm == 0) {
        Eigen::MatrixXd b;
        for (int iter = 0; iter < context.number_of_iterations; iter++) {
            #pragma omp parallel for
            for (int i = 0; i < context.V.rows(); i++) context.cells[i].find_rotation(context.V_new);
            build_b(context, b);
            context.V_new = context.solver.solve(b);
        }
    } else {
        context.libigl_bc.row(context.libigl_bc.rows() - 1) = new_handle_pos;
        igl::arap_solve(context.libigl_bc, context.libigl_solver, context.V_new);
        #pragma omp parallel for
        for (int i = 0; i < context.V.rows(); i++) context.cells[i].find_rotation(context.V_new);
    }
}