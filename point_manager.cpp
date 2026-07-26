#include "point_manager.h"
#include "globals.h"

void change_vertex_type(int i, int8_t new_type) {
    if(vertex_type[i]==new_type) return;
    switch (vertex_type[i]) {
        case 1: handles.erase(std::remove(handles.begin(), handles.end(), i), handles.end());   break;
        case 2: anchors.erase(std::remove(anchors.begin(), anchors.end(), i), anchors.end());   break;
    }
    vertex_type[i] = new_type;
    switch (new_type) {
        case 1: handles.push_back(i);   break;
        case 2: anchors.push_back(i);   break;
    }
}

void draw_vertices(igl::opengl::glfw::Viewer& viewer,bool draw_available, bool draw_handles, bool draw_anchors)
{ 
  viewer.data().clear_points();
  if(draw_available){
    for(int i = 0; i<vertex_type.size();i++){
      if (vertex_type[i]==0) viewer.data().add_points(V_new.row(i),Eigen::RowVector3d(0,1,0));
    }
  }
  if(draw_handles){
    for(int i : handles) viewer.data().add_points(V_new.row(i),Eigen::RowVector3d(0,1,1));
  }
  if(draw_anchors){
    for(int i : anchors) viewer.data().add_points(V_new.row(i),Eigen::RowVector3d(1,0,0));
  }
}

int point_picker(const igl::opengl::glfw::Viewer& viewer)
{
  Eigen::Vector2f mouse(viewer.current_mouse_x,
                            viewer.core().viewport(3) - viewer.current_mouse_y);

  int fid;
  Eigen::Vector3d bc;

  bool hit = igl::unproject_onto_mesh(mouse,viewer.core().view,viewer.core().proj,
                                      viewer.core().viewport,V_new,F,fid,bc);

  if(!hit) return -1;

  Eigen::RowVector3d hit_point =
      bc(0) * V_new.row(F(fid,0)) +
      bc(1) * V_new.row(F(fid,1)) +
      bc(2) * V_new.row(F(fid,2));

  int selected = -1;
  double min_dist_px = 10.0;

  for(int i = 0; i < 3; i++)
  {
      int vertex = F(fid,i);

      Eigen::Vector3f proj = igl::project(
          Eigen::Vector3f(V_new.row(vertex).cast<float>()),
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
    int8_t vert_type = vertex_type[selected];
    if(vert_type == 0)
    {      
      if(mode == 1 && vert_type!=2) {
        change_vertex_type(selected, 1);
        build_L();
        draw_vertices(viewer, true, true, true);
      }
      else if(mode==2 && vert_type!=1){
        change_vertex_type(selected, 2);
        build_L();
        draw_vertices(viewer, true, true, true);
      }
    }
    else if(vert_type==1 && mode == 3){
      change_vertex_type(selected, 0);
      draw_vertices(viewer, false, true, false);
    }
    else if(vert_type==2 && mode == 4){
      change_vertex_type(selected, 0);
      draw_vertices(viewer, false, false, true);
    }
  }
}
