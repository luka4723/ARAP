#pragma once

#include <Eigen/Core>
#include "mesh_context.h"

struct BenchmarkDurations {
    std::chrono::duration<double> rotations{};
    std::chrono::duration<double> rhs{};
    std::chrono::duration<double> linear_solve{};

    void reset() {
        rotations = {};
        rhs = {};
        linear_solve = {};
    }
};

void build_b(MeshContext& context, Eigen::MatrixXd& b);
Eigen::RowVector3d mouse_to_plane(double mouse_x, double mouse_y, const Eigen::Matrix4f& view,
                                  const Eigen::Matrix4f& proj, const Eigen::Vector4f& viewport,
                                  const Eigen::Vector3d& plane_point, const Eigen::Vector3d& plane_normal);
void prepare_drag_session(MeshContext& ctx, const Eigen::Matrix4f& view_matrix);
void solve_arap_step(MeshContext& ctx, const Eigen::RowVector3d& new_handle_pos, BenchmarkDurations* durations);
