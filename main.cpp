
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <iostream>

#include "mesh_context.h"
#include "menu.h"

int main(int argc, char *argv[])
{
  MeshContext context;

  std::vector<int> default_anchors = {330, 18, 380, 300, 235, 677, 818, 189, 108};
  std::vector<int> default_handles = {853};

  if(!context.load_mesh("meshes/armadillo_1k.off", default_anchors, default_handles)) return EXIT_FAILURE;

  igl::opengl::glfw::Viewer viewer;
  viewer.data().point_size = 10;
  viewer.core().is_animating = true;
  viewer.core().animation_max_fps = 1000;

  igl::opengl::glfw::imgui::ImGuiPlugin plugin;
  igl::opengl::glfw::imgui::ImGuiMenu menu;
  viewer.plugins.push_back(&plugin);
  plugin.widgets.push_back(&menu);
  setup_menu(viewer, menu, context);
  
  viewer.data().set_mesh(context.V_new, context.F);
  viewer.launch();
  return EXIT_SUCCESS;
}