#include "menu.h"
#include "point_manager.h"
#include <GLFW/glfw3.h>
#include "solver.h"
#include "ImGuiFileDialog.h"

int frame_count = 0;
double last_time = 0.0;
double current_fps = 0.0;
bool shows_energy = true;
std::array<double,3> energy = {0.0, 0.0, 0.0};

void setup_menu(igl::opengl::glfw::Viewer& viewer, igl::opengl::glfw::imgui::ImGuiMenu& menu,
                MeshContext& context)
{
    draw_vertices(viewer,context,false,true,true);
    viewer.callback_mouse_down = [&](igl::opengl::glfw::Viewer& viewer, int button, int modifier)
    {
        if(button == GLFW_MOUSE_BUTTON_LEFT)
        {
            if(context.mode==0){
                context.selected_vertex = point_picker(viewer, context);
                if (context.selected_vertex != -1 && context.vertex_type[context.selected_vertex] == 1)
                    prepare_drag_session(context, viewer.core().view);
                else context.selected_vertex = -1;
            }
            else point_manager(context, viewer, button, modifier);
        }
        return false;
    };
    viewer.callback_mouse_move = [&context](igl::opengl::glfw::Viewer& /*viewer*/, double x, double y) {
            if (context.is_dragging && context.selected_vertex != -1)
            {
                context.needs_draw = true;
                context.mouse_x = x;
                context.mouse_y = y;
                return true;
            }
            return false;
        };
    
    viewer.callback_pre_draw = [&context](igl::opengl::glfw::Viewer& viewer)
    {
        double current_time = glfwGetTime();
        frame_count++;
        if (current_time - last_time >= 1.0) {
            current_fps = frame_count;            
            frame_count = 0;
            last_time = current_time;
        }
        if (context.needs_draw) {
            Eigen::RowVector3d target_pos = mouse_to_plane(context.mouse_x, context.mouse_y, viewer.core().view,
                                                           viewer.core().proj, viewer.core().viewport,
                                                           context.drag_plane_point, context.drag_plane_normal);
            solve_arap_step(context, target_pos, shows_energy);
            if (shows_energy)
            {
                #pragma omp parallel for
                for (int i = 0; i < context.V.rows(); i++) context.cells[i].find_rotation(context.V_new, context.halfedges);
                energy = context.calculate_energy();
                viewer.data().set_colors(context.C);
            }
            viewer.data().set_vertices(context.V_new);
            draw_vertices(viewer, context, false, true, true);
            context.needs_draw = false;
        }
        return false;
    };

    viewer.callback_mouse_up = [&context](igl::opengl::glfw::Viewer& /*viewer*/, int button, int modifier) {
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            if(context.is_dragging && context.selected_vertex >= 0) context.last_selected = context.selected_vertex;
            context.is_dragging = false;
            context.needs_draw = false;
            context.selected_vertex = -1;
        }
        return false;
    };
    
    menu.callback_draw_viewer_menu = [&context, &viewer]()
    {
        if (ImGui::Button("Open Mesh"))
        {
            ImGuiFileDialog::Instance()->OpenDialog(
                "MeshDlg",          
                "Open Mesh",        
                ".off,.obj,.ply",   
                "."                 
            );
        }

        if (ImGuiFileDialog::Instance()->Display("MeshDlg"))
        {
            if (ImGuiFileDialog::Instance()->IsOk())
            {
                std::string file = ImGuiFileDialog::Instance()->GetFilePathName();
                context.load_mesh(file);
                viewer.data().clear();
                viewer.data().set_mesh(context.V_new, context.F);
            }

            ImGuiFileDialog::Instance()->Close();
        }
        if(ImGui::RadioButton("None", context.mode == 0))
        {
            context.mode = 0;
            draw_vertices(viewer,context,false,true,true);
        }
        
        if(ImGui::RadioButton("Handle Selection", context.mode == 1))
        {
            context.mode = 1;
            draw_vertices(viewer,context,true,true,true);
        }
        
        if(ImGui::RadioButton("Anchor Selection", context.mode == 2))
        {
            context.mode = 2;
            draw_vertices(viewer,context,true,true,true);
        }
        
        if(ImGui::RadioButton("Handle Removal", context.mode == 3))
        {
            context.mode = 3;
            draw_vertices(viewer,context,false,true,false);
        }
        
        if(ImGui::RadioButton("Anchor Removal", context.mode == 4))
        {
            context.mode = 4;
            draw_vertices(viewer,context,false,false,true);
        }
        if(ImGui::Button("Reset Vertices"))
        {
            context.reset_vertices();
            draw_vertices(viewer,context,false,false,false);
        }
        if(ImGui::Button("Reset Mesh"))
        {
            context.reset_mesh();
            viewer.data().set_vertices(context.V_new);
            draw_vertices(viewer,context,false,true,true);
        }
        if(ImGui::Button("Save Vertices"))context.save_config();
        if(ImGui::Button("Load Config")){
            context.load_config();
            draw_vertices(viewer, context, false, true, true);
            context.mode = 0;
        }
        ImGui::Separator();
        ImGui::SliderInt("Iterations", &context.number_of_iterations, 1, 100);
        if (ImGui::SliderInt("Smoothness factor", &context.lambda, 0, 99)) 
        {
            context.build_left_side();
            if (context.algorithm == 0 && context.last_selected >= 0)
            {
                context.selected_vertex = context.last_selected;
                Eigen::RowVector3d target = context.V_new.row(context.selected_vertex);

                context.factorize_left_side();
                solve_arap_step(context, target, shows_energy);
                
                #pragma omp parallel for
                for(int i = 0; i < context.V.rows(); i++)
                context.cells[i].find_rotation(context.V_new, context.halfedges);

                energy = context.calculate_energy();

                viewer.data().set_vertices(context.V_new);
                draw_vertices(viewer, context, false, true, true);

                context.selected_vertex = -1;
            }
        }
        if (ImGui::RadioButton("Custom ARAP", context.algorithm == 0)) context.algorithm = 0;
        if (ImGui::RadioButton("libigl ARAP", context.algorithm == 1)) context.algorithm = 1;
        ImGui::Text("FPS: %.1f", current_fps);
        ImGui::Checkbox("Show energy", &shows_energy);
        if (ImGui::SliderInt("Color factor", &context.energy_color_coeff, 1, 100)) energy = context.calculate_energy();
        if(shows_energy) {
            ImGui::Text("ARAP Energy %.2f", energy[0]);
            ImGui::Text("Smooth Energy %.2f", energy[1]);   
            ImGui::Text("Total Energy %.2f", energy[2]);   
            viewer.data().set_colors(context.C);
        }
        else {
            ImGui::Text("ARAP Energy: N/A");
            ImGui::Text("Smooth Energy: N/A");
            ImGui::Text("Total Energy: N/A");
            context.C.col(0).setConstant(0.0);
            context.C.col(2).setConstant(1.0);
            viewer.data().set_colors(context.C);
        }
    };
}