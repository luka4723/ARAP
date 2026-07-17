#pragma once

#include <igl/opengl/glfw/Viewer.h>
#include <igl/unproject_onto_mesh.h>

void draw_vertices(igl::opengl::glfw::Viewer& viewer,bool draw_available, bool draw_handles, bool draw_anchors);
void point_manager(int mode, igl::opengl::glfw::Viewer& viewer, int button, int modifier);
