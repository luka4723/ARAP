#include "point_manager.h"
#include "globals.h"


void draw_vertices(igl::opengl::glfw::Viewer& viewer,bool draw_available, bool draw_handles, bool draw_anchors)
{
  viewer.data().clear_points();
  if(draw_available){
    for(int i : available) viewer.data().add_points(V.row(i),Eigen::RowVector3d(0,1,0));
  }
  if(draw_handles){
    for(int i : handles) viewer.data().add_points(V.row(i),Eigen::RowVector3d(0,1,1));
  }
  if(draw_anchors){
    for(int i : anchors) viewer.data().add_points(V.row(i),Eigen::RowVector3d(1,0,0));
  }
}

int point_picker(igl::opengl::glfw::Viewer& viewer)
{
  Eigen::Vector2f mouse(viewer.current_mouse_x,
                            viewer.core().viewport(3) - viewer.current_mouse_y);

  int fid;
  Eigen::Vector3d bc;

  bool hit = igl::unproject_onto_mesh(mouse,viewer.core().view,viewer.core().proj,
                                      viewer.core().viewport,V,F,fid,bc);

  if(!hit) return -1;

  Eigen::RowVector3d hit_point =
      bc(0) * V.row(F(fid,0)) +
      bc(1) * V.row(F(fid,1)) +
      bc(2) * V.row(F(fid,2));

  int selected = -1;
  double min_dist_px = 10.0;

  for(int i = 0; i < 3; i++)
  {
      int vertex = F(fid,i);

      Eigen::Vector3f proj = igl::project(
          Eigen::Vector3f(V.row(vertex).cast<float>()),
          viewer.core().view,
          viewer.core().proj,
          viewer.core().viewport
      );

      double dist_px = (Eigen::Vector2f(proj(0), proj(1)) - mouse).norm();

      if(dist_px < min_dist_px)
      {
          min_dist_px = dist_px;
          selected = vertex;
      }  
  }
  return selected;
}

void point_manager(int mode, igl::opengl::glfw::Viewer& viewer, int button, int modifier)
{
  int selected = point_picker(viewer);

  if(selected != -1)
  {
    bool is_not_anchor = anchors.find(selected) == anchors.end();
    bool is_not_handle = handles.find(selected) == handles.end();
    if(is_not_anchor && is_not_handle)
    {      
      if(mode == 1 && is_not_anchor) {
        handles.insert(selected);
        available.erase(selected);
        build_L();
        draw_vertices(viewer, true, true, true);
      }
      else if(mode==2 && is_not_handle){
        anchors.insert(selected);
        available.erase(selected);
        build_L();
        draw_vertices(viewer, true, true, true);
      }
    }
    else if(!is_not_handle && mode == 3){
      handles.erase(selected);
      available.insert(selected);
      draw_vertices(viewer, false, true, false);
    }
    else if(!is_not_anchor && mode == 4){
      anchors.erase(selected);
      available.insert(selected);
      draw_vertices(viewer, false, false, true);
    }
  }
}
