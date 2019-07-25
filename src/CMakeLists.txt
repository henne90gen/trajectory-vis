
# Setup launch prerequisites
set(PLUGINS
	cg_fltk
	cg_icons
	cg_ext
	cmi_io
	crg_stereo_view
)
set(SHADER_PATH ${CMAKE_SOURCE_DIR}/shader)
set(GUI_FILES "")
set(CONFIG_FILES "")

# Source files
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.cxx")
# Header files
file(GLOB_RECURSE HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "${CMAKE_SOURCE_DIR}/include/*.h*")
# Shader files
file(GLOB_RECURSE SHADER_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "${CMAKE_SOURCE_DIR}/shader/*.gl*")

# Additional include paths
include_directories(
	${CMAKE_SOURCE_DIR}/include
)

# Setup application plugin target
cgv_add_module(${PROJECT_NAME} ${SOURCES} ${IMG_SOURCES} ${HEADERS} ${SHADER_SOURCES} ${IMAGES})
target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES} ${cgv_LIBRARIES})
cgv_test_shaders(${PROJECT_NAME} ${SHADER_SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "${FOLDER_NAME_APPLICATION_PLUGINS}")

cgv_launchconfig_prepare(${PROJECT_NAME})
cgv_launchconfig_setup_vs(${PROJECT_NAME})
cgv_launchconfig_script_sh(${PROJECT_NAME})

cgv_write_find_file(${PROJECT_NAME})
