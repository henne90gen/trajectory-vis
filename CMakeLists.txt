cmake_minimum_required(VERSION 3.20)

project(trajectory-vis)

# Third Party Dependencies
include(FetchContent)
FetchContent_Declare(
        cgv
        GIT_REPOSITORY https://github.com/sgumhold/cgv.git
        GIT_TAG ef10a43cd2a2ca17905bdb4e98c41529b6d913c3
)

FetchContent_MakeAvailable(
        cgv
)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(trajectory_vis SHARED
    src/traj_line_renderer.cxx
    src/traj_ribbon_3d_renderer_gpu.cxx
    src/traj_ribbon_3d_renderer.cxx
    src/traj_ribbon_renderer.cxx
    src/traj_tube_renderer.cxx
    src/traj_velocity_renderer.cxx
    src/plugin.cxx
    src/math_utils.cxx
    src/lighting.cxx
    src/ellipsoid_instanced_renderer.cxx
    src/data.cxx)
target_include_directories(trajectory_vis PRIVATE include)
target_link_libraries(trajectory_vis PRIVATE cgv_gl annf)
target_compile_definitions(trajectory_vis PRIVATE ETV_EXPORTS)
add_dependencies(trajectory_vis cgv_viewer crg_stereo_view crg_grid cg_fltk)

set_plugin_execution_params(trajectory_vis "plugin:cg_fltk plugin:crg_stereo_view plugin:crg_grid \"type(shader_config):shader_path='${CGV_DIR}/libs/cgv_gl/glsl'\" plugin:trajectory_vis")

configure_file(run_plugin.sh.in ${CMAKE_BINARY_DIR}/run_plugin.sh
        FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
        @ONLY)
