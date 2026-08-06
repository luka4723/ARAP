#pragma once

#include <igl/opengl/glfw/Viewer.h>
#include "mesh_context.h"

void draw_vertices(igl::opengl::glfw::Viewer& viewer, const MeshContext& context, bool draw_available, bool draw_handles, bool draw_anchors);
int point_picker(const igl::opengl::glfw::Viewer& viewer, const MeshContext& context);
void point_manager(MeshContext& context, igl::opengl::glfw::Viewer& viewer);
