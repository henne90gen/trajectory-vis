cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(cg_trajectory_vis LANGUAGES CXX)

option(BUILD_SHARED_LIBS "Create shared libraries" TRUE)

message(STATUS "build type: ${CMAKE_BUILD_TYPE}")

# Must use GNUInstallDirs to install libraries into correct locations on all platforms.
include(GNUInstallDirs)

# Use the CGV framework as CMake subproject instead of an installed version. The
# git submodule ensures that a user always uses a compatible version of CGV.
add_subdirectory(cgv)

set(SHADERS
    shader/traj_line_shader.glfs
    shader/traj_line_shader.glpr
    shader/traj_line_shader.glvs
    shader/traj_ellipsoid_shader.glpr
    shader/traj_ellipsoid_shader.glfs
    shader/traj_ellipsoid_shader.glvs
    shader/traj_math.glsl
    shader/traj_light.glsl
    shader/traj_ribbon_shader.glfs
    shader/traj_ribbon_shader.glpr
    shader/traj_ribbon_shader.glvs
    shader/traj_velocity_shader.glfs
    shader/traj_velocity_shader.glvs
    shader/traj_velocity_shader.glgs
    shader/traj_velocity_shader.glpr
    shader/traj_tube_shader.glfs
    shader/traj_tube_shader.glpr
    shader/traj_tube_shader.glvs
    shader/traj_ribbon_3d_gpu_shader.glfs
    shader/traj_ribbon_3d_gpu_shader.glgs
    shader/traj_ribbon_3d_gpu_shader.glpr
    shader/traj_ribbon_3d_gpu_shader.glvs
    shader/traj_ribbon_3d_shader.glvs
    shader/traj_ribbon_3d_shader.glfs
    shader/traj_ribbon_3d_shader.glpr)

add_library(trajectory_vis
    # Source Files
    src/plugin.cxx
    src/data.cxx
    src/traj_line_renderer.cxx
    src/traj_ribbon_renderer.cxx
    src/traj_tube_renderer.cxx
    src/traj_ribbon_3d_renderer.cxx
    src/traj_ribbon_3d_renderer_gpu.cxx
    src/traj_velocity_renderer.cxx
    src/ellipsoid_instanced_renderer.cxx
    src/lighting.cxx
    src/math_utils.cxx

    # Public Headers
    include/data.h
    include/ellipsoid_instanced_renderer.h
    include/lighting.h
    include/math_utils.h
    include/plugin.h
    include/traj_line_renderer.h
    include/traj_ribbon_3d_renderer_gpu.h
    include/traj_ribbon_3d_renderer.h
    include/traj_ribbon_renderer.h
    include/traj_tube_renderer.h
    include/traj_velocity_renderer.h
    include/types.h

    # Shader Files
    ${SHADERS})

# Add an alias target for the cgv:: namespace such that other components can use this target like
# other imported targets with namespace prefixes.
add_library(cgv::cg_trajectory_vis ALIAS trajectory_vis)

# Define headers for this library. PUBLIC headers are used for compiling the library, and will be
# added to consumers' build paths.
target_include_directories(trajectory_vis
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )

# Dependencies on other components, plugins and libraries.
target_link_libraries(trajectory_vis
    PUBLIC
        cgv::base
        cgv::cgv_gl
        cgv::utils
        cgv::gui
        cgv::signal
        cgv::render
        )


# -----------------------
# Directory Configuration
# -----------------------

# Build library in directories matching the installation layout. If multi-config generators are
# used, build targets in a config-specific subdirectory.
if(CMAKE_CONFIGURATION_TYPES)
    set_target_properties(trajectory_vis PROPERTIES
        ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/$<CONFIG>/${CMAKE_INSTALL_LIBDIR}
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/$<CONFIG>/${CMAKE_INSTALL_LIBDIR}
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/$<CONFIG>/${CMAKE_INSTALL_BINDIR})
else()
    set_target_properties(trajectory_vis PROPERTIES
        ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
endif()

# Move shader files into "Shaders" folder in Visual Studio.
source_group("Shaders" FILES ${SHADERS})


# ---------------
# Shader handling
# ---------------

# TODO: Respect REGISTER_SHADERS or BUILD_SHARED_LIBS CMake options and register shader statically

# Get the output directory for executables. By using the property of the target, we do not need to
# distinguish different platforms.
get_target_property(RUNTIME_OUTPUT trajectory_vis RUNTIME_OUTPUT_DIRECTORY)

# Just copy all shader files after successful build into shader/ next to bin/ directory.
foreach(shader IN LISTS SHADERS)
    add_custom_command(TARGET trajectory_vis POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/${shader}
            ${RUNTIME_OUTPUT}/../${shader})
endforeach()


# ------------------------------
# Compile Properties and Options
# ------------------------------

set_target_properties(trajectory_vis PROPERTIES 
            LINK_FLAGS "-Wl,--no-undefined")

target_compile_options(trajectory_vis PRIVATE
            "-Wall"
            "-O2")

if(${BUILD_SHARED_LIBS})
    target_compile_definitions(trajectory_vis PUBLIC TRAJECTORY_vis_EXPORTS)
endif()

if (${PERF_STATS})
    target_compile_definitions(trajectory_vis PRIVATE PERF_STATS)
endif()


# ------------
# Installation
# ------------

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS trajectory_vis EXPORT trajectory_visTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows

# Install header files of library
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Install shader files
install(DIRECTORY shader/ DESTINATION ${CMAKE_INSTALL_BINDIR}/../shader)

# This makes the project importable from the install directory. Put config file in per-project dir
# (name MUST match).
install(EXPORT trajectory_visTargets NAMESPACE cgv:: DESTINATION cmake/trajectory_vis)
install(FILES trajectory_visConfig.cmake DESTINATION cmake/trajectory_vis)

# This makes the project importable from the build directory
export(TARGETS trajectory_vis NAMESPACE cgv:: FILE ${CMAKE_BINARY_DIR}/cmake/trajectory_visTargets.cmake)
file(COPY trajectory_visConfig.cmake DESTINATION ${CMAKE_BINARY_DIR}/cmake)
