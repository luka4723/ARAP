
#include "menu.h"
#include "globals.h"

int main(int argc, char *argv[])
{
  load_mesh();

  igl::opengl::glfw::Viewer viewer;
  viewer.data().point_size = 10;

  igl::opengl::glfw::imgui::ImGuiPlugin plugin;
  viewer.plugins.push_back(&plugin);
  igl::opengl::glfw::imgui::ImGuiMenu menu;
  plugin.widgets.push_back(&menu);
  setup_menu(viewer, menu);

  viewer.data().set_mesh(V, F);
  viewer.launch();
}