add_executable(matthew main.cpp)
add_library(matthewlib matthew.cpp meshiew.cpp pointiew.cpp)
target_include_directories(matthewlib PUBLIC
        ${NANOGUI_EXTRA_INCS}
        ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(matthewlib surface_mesh matthew_io timercpp)
add_dependencies(matthewlib generated_headers)
target_link_libraries(matthewlib nanogui ${NANOGUI_EXTRA_LIBS} lines)
target_link_libraries(matthew matthewlib)

