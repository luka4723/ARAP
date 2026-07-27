#include "menu.h"
#include "point_manager.h"
#include "globals.h"
#include <GLFW/glfw3.h>
#include "solver.h"

int frame_count = 0;
double last_time = glfwGetTime();
double current_fps = 0.0;
bool shows_energy = true;

void setup_menu(
    igl::opengl::glfw::Viewer& viewer,
    igl::opengl::glfw::imgui::ImGuiMenu& menu
)
{
    draw_vertices(viewer,false,true,true);
    viewer.callback_mouse_down = [&](igl::opengl::glfw::Viewer& viewer, int button, int modifier)
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
    
    viewer.callback_pre_draw = [](igl::opengl::glfw::Viewer& viewer)
    {
        
        double current_time = glfwGetTime();
        frame_count++;
        if (current_time - last_time >= 1.0) {
            current_fps = frame_count;            
            frame_count = 0;
            last_time = current_time;
        }
        solver_pre_draw(viewer);
        return false;
    };
    viewer.callback_mouse_up = [](igl::opengl::glfw::Viewer& viewer, int button, int modifier)
    {
        solver_mouse_up(viewer, button);
        return false;
    };
    
    menu.callback_draw_viewer_menu = [&]()
    {
        
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
        
        
        if(ImGui::Button("Reset All"))
        {
            // for(int n : anchors) std::cout << "ANCHORS: " << n << "\n";
            // for(int n : handles) std::cout << "HANDLES: " << n << "\n";
            
            handles.clear();
            anchors.clear();
            vertex_type.assign(V.rows(),0);
            V_new = V;
            viewer.data().set_vertices(V_new);
            energy_flag = false;
            mode = 0;
            draw_vertices(viewer,false,false,false);
        }
        ImGui::Separator();
        ImGui::SliderInt("Iterations", &number_of_iterations, 1, 100);
        if (ImGui::RadioButton("Custom ARAP", algorithm == 0)) algorithm = 0;
        if (ImGui::RadioButton("libigl ARAP", algorithm == 1)) algorithm = 1;
        ImGui::Text("FPS: %.1f", current_fps);
        ImGui::Checkbox("Show energy", &shows_energy);
        double E;
        if(energy_flag) E = calculate_energy();
        else E = 0.0;
        if(shows_energy) ImGui::Text("Energy %.2f", E);
        else ImGui::Text("FPS: N/A");
        //ImGui::End();
    };
}