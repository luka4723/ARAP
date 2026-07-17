
#include "menu.h"
#include "globals.h"
#include "cell.h"

int main(int argc, char *argv[])
{
  load_mesh();
  precompute_angles();

  igl::opengl::glfw::Viewer viewer;
  viewer.data().point_size = 10;

  igl::opengl::glfw::imgui::ImGuiPlugin plugin;
  viewer.plugins.push_back(&plugin);
  igl::opengl::glfw::imgui::ImGuiMenu menu;
  plugin.widgets.push_back(&menu);
  setup_menu(viewer, menu);

  std::vector<Cell> cells;
  auto start = std::chrono::high_resolution_clock::now();
  for(int i=0;i<V.rows();i++) cells.push_back(Cell(i));
  auto end = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> elapsed = end - start;

  std::cout << "Time: " << elapsed.count() << " seconds\n";

  viewer.data().set_mesh(V, F);
  viewer.launch();
}