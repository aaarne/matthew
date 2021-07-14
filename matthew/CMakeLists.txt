add_executable(matt main.cpp)
add_library(matthew SHARED
        matthew.cpp
        meshiew.cpp
        pointiew.cpp
        color_window.cpp)
target_include_directories(matthew PUBLIC
        ${NANOGUI_EXTRA_INCS}
        ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(matthew surface_mesh matthew_io timercpp)
add_dependencies(matthew generated_headers)
target_link_libraries(matthew nanogui ${NANOGUI_EXTRA_LIBS} renderers)
target_link_libraries(matt matthew)
