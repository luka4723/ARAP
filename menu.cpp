#include "menu.h"
#include "point_manager.h"
#include "globals.h"
#include <GLFW/glfw3.h>
#include "rotation_solver.h"

void setup_menu(
    igl::opengl::glfw::Viewer& viewer,
    igl::opengl::glfw::imgui::ImGuiMenu& menu
)
{
    menu.callback_draw_viewer_menu = [&]()
    {
        viewer.callback_mouse_down =
        [&](igl::opengl::glfw::Viewer& viewer, int button, int modifier)
        {
            if(button == GLFW_MOUSE_BUTTON_LEFT)
            {
                if(mode==0) solver_mouse_down(viewer, button);
                else point_manager(mode, viewer, button, modifier);
            }

            return false;
        };

        viewer.callback_mouse_move = [](igl::opengl::glfw::Viewer& viewer, double x, double y)
        {
            return solver_mouse_move(viewer, x, y);
        };
        viewer.callback_mouse_up = [](igl::opengl::glfw::Viewer& viewer, int button, int modifier)
        {
            solver_mouse_up(viewer, button);
            return false;
        };

        if(ImGui::RadioButton("None", mode == 0))
        {
            mode = 0;
            draw_vertices(viewer,false,true,true);
        }

        if(ImGui::RadioButton("Handle Selection", mode == 1))
        {
            mode = 1;
            draw_vertices(viewer,true,true,true);
        }

        if(ImGui::RadioButton("Anchor Selection", mode == 2))
        {
            mode = 2;
            draw_vertices(viewer,true,true,true);
        }

        if(ImGui::RadioButton("Handle Removal", mode == 3))
        {
            mode = 3;
            draw_vertices(viewer,false,true,false);
        }

        if(ImGui::RadioButton("Anchor Removal", mode == 4))
        {
            mode = 4;
            draw_vertices(viewer,false,false,true);
        }


        if(ImGui::Button("Clear All"))
        {
            handles.clear();
            anchors.clear();

            for(int i = 0; i < V.rows(); i++)
                available.insert(i);

            mode = 0;
            draw_vertices(viewer,false,false,false);
        }
    };
}