cmake_minimum_required(VERSION 3.16)

project(ARAP)

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(external/libigl)


add_executable(ARAP
main.cpp
globals.cpp
menu.cpp
point_manager.cpp
cell.cpp
solver.cpp
)

igl_include(glfw)
igl_include(opengl)
igl_include(imgui)

target_link_libraries(ARAP PRIVATE igl::core igl::glfw igl::opengl igl::imgui)
add_custom_command(TARGET ARAP POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/meshes
    $<TARGET_FILE_DIR:ARAP>/meshes
)