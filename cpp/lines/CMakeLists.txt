add_library(lines line_renderer.cpp grid.cpp ${CMAKE_BINARY_DIR}/shaders_gen.h)
target_include_directories(lines PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${NANOGUI_EXTRA_INCS} ${CMAKE_BINARY_DIR})
target_link_libraries(lines nanogui surface_mesh)