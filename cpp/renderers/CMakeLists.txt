add_library(renderers line_renderer.cpp grid.cpp point_renderer.cpp point_cloud_renderer.cpp vectorfield_renderer.cpp)
target_include_directories(renderers PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${NANOGUI_EXTRA_INCS} ${CMAKE_BINARY_DIR})
target_link_libraries(renderers nanogui surface_mesh)
add_dependencies(renderers generated_headers)