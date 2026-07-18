#pragma once

#include <Eigen/Core>
#include <igl/opengl/glfw/Viewer.h>

extern Eigen::MatrixXd V_new;

void solver_mouse_down(igl::opengl::glfw::Viewer& viewer, int button);
bool solver_mouse_move(igl::opengl::glfw::Viewer& viewer, double x, double y);
void solver_mouse_up(igl::opengl::glfw::Viewer& viewer, int button);
