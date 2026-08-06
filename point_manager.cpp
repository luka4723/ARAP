#include "point_manager.h"
#include <igl/unproject_onto_mesh.h>
#include <igl/project.h>

void draw_vertices(igl::opengl::glfw::Viewer& viewer, const MeshContext& context, bool draw_available, bool draw_handles, bool draw_anchors)
{ 
  viewer.data().clear_points();
  if(draw_available && context.V.rows() < 2000){
    for(int i = 0; i<context.vertex_type.size();i++){
      if (context.vertex_type[i]==0) viewer.data().add_points(context.V_new.row(i),Eigen::RowVector3d(0,1,0));
    }
  }
  if(draw_handles){
    for(int i : context.handles) viewer.data().add_points(context.V_new.row(i),Eigen::RowVector3d(0,1,1));
  }
  if(draw_anchors){
    for(int i : context.anchors) viewer.data().add_points(context.V_new.row(i),Eigen::RowVector3d(1,0,0));
  }
}

int point_picker(const igl::opengl::glfw::Viewer& viewer, const MeshContext& context)
{
  Eigen::Vector2f mouse(viewer.current_mouse_x,
                            viewer.core().viewport(3) - viewer.current_mouse_y);

  int fid;
  Eigen::Vector3d bc;

  bool hit = igl::unproject_onto_mesh(mouse,viewer.core().view,viewer.core().proj,
                                      viewer.core().viewport,context.V_new,context.F,fid,bc);

  if(!hit) return -1;

  int selected = -1;
  double min_dist_px = 10.0;

  for(int i = 0; i < 3; i++)
  {
      int vertex = context.F(fid,i);

      Eigen::Vector3f proj = igl::project(
          Eigen::Vector3f(context.V_new.row(vertex).cast<float>()),
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

void point_manager(MeshContext& context, igl::opengl::glfw::Viewer& viewer)
{
  int selected = point_picker(viewer, context);

  if(selected != -1)
  {
    int8_t vert_type = context.vertex_type[selected];
    if(vert_type == 0)
    {      
      if(context.mode == 1) {
        context.change_vertex_type(selected, 1);
        draw_vertices(viewer, context, true, true, true);
      }
      else if(context.mode==2){
        context.change_vertex_type(selected, 2);
        draw_vertices(viewer, context, true, true, true);
      }
    }
    else if(vert_type==1 && context.mode == 3){
      context.change_vertex_type(selected, 0);
      draw_vertices(viewer, context, false, true, false);
    }
    else if(vert_type==2 && context.mode == 4){
      context.change_vertex_type(selected, 0);
      draw_vertices(viewer, context, false, false, true);
    }
  }
}
