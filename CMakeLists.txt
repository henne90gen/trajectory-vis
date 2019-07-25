cmake_minimum_required(VERSION 2.6)
set(PROJECT_NAME "cg_trajectory_vis")
project(${PROJECT_NAME} LANGUAGES C CXX)

# Add CGV framework
# - configure
option(BUILD_EXAMPLE_PLUGIN "Build the example plugin" FALSE)
mark_as_advanced(FORCE BUILD_EXAMPLE_PLUGIN)
mark_as_advanced(FORCE BUILD_VR_SUPPORT)
set(LIBS)
# - include in build
set(cgv_DIR ${CMAKE_SOURCE_DIR}/cgv)
add_subdirectory(${CMAKE_SOURCE_DIR}/cgv ext_build/cgv)
# - prepare for use by plugin targets
find_package(cgv 
	COMPONENTS render gl gui base type reflect utils signal data media
	HINTS ${cgv_DIR}
	)
find_package(OpenGL)
find_package(shader_test)
find_package(cgv_viewer)
# - reference cgv packages that are commonly used by almost all plugins
cgv_find_package(GLEW)
cgv_find_package(cgv_gl)
cgv_find_package(cg_fltk)
cgv_find_package(cg_icons)
cgv_find_package(cg_ext)
cgv_find_package(cmi_io)
cgv_find_package(crg_stereo_view)
cgv_get_viewer_locations(VIEWER_EXE VIEWER_DEBUG_EXE)
get_filename_component(FINAL_BUILD_BIN_DIR ${VIEWER_EXE} DIRECTORY)
# - add common CGV include paths used by almost all plugins
include_directories(
	${cgv_INCLUDE_DIRS} 
	${CMAKE_SOURCE_DIR}/cgv/libs 
	${cgv_gl_INCLUDE_DIRS}
)

# Add cg_trajectory_vis plugin
add_subdirectory(src)

# Launch configurations for various IDEs that are not Visual Studio
cgv_launchconfig_singlefile_vscode()
