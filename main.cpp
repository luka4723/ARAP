
#include "menu.h"
#include "globals.h"
#include "cell.h"
#include "solver.h"

int main(int argc, char *argv[])
{
  load_mesh();
  precompute_angles();

  igl::opengl::glfw::imgui::ImGuiPlugin plugin;
  igl::opengl::glfw::imgui::ImGuiMenu menu;
  viewer.plugins.push_back(&plugin);
  plugin.widgets.push_back(&menu);
  setup_menu(viewer, menu);
  
  viewer.data().set_mesh(V, F);
  viewer.launch();
}