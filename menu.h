#pragma once

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include "mesh_context.h"

void setup_menu(igl::opengl::glfw::Viewer& viewer, igl::opengl::glfw::imgui::ImGuiMenu& menu, MeshContext& context);