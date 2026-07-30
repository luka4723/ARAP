cmake_minimum_required(VERSION 3.16)

project(ARAP)

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(external/libigl)

igl_include(glfw)
igl_include(opengl)
igl_include(imgui)

find_package(OpenMP REQUIRED)

add_library(imguifiledialog
    external/ImGuiFileDialog/ImGuiFileDialog.cpp
)

target_include_directories(imguifiledialog PUBLIC
    external/ImGuiFileDialog
)

target_link_libraries(imguifiledialog PUBLIC
    igl::imgui
)

add_executable(ARAP
    main.cpp
    mesh_context.cpp
    menu.cpp
    point_manager.cpp
    cell.cpp
    solver.cpp
)

target_link_libraries(ARAP PRIVATE
    igl::core
    igl::glfw
    igl::opengl
    igl::imgui
    OpenMP::OpenMP_CXX
    imguifiledialog
)

add_custom_command(TARGET ARAP POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/meshes
    $<TARGET_FILE_DIR:ARAP>/meshes
)