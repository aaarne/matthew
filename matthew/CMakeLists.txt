file(GLOB shaders "../shaders/*.glsl")

set(SHADER_HEADER_NAME ${CMAKE_BINARY_DIR}/shaders_gen.h)
add_custom_command(
        OUTPUT ${SHADER_HEADER_NAME}
        COMMAND shader2header ${SHADER_HEADER_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/../shaders
        DEPENDS ${shaders};${CMAKE_CURRENT_SOURCE_DIR}/../externals/shader2header/shader2header.cpp
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

include_directories(${CMAKE_BINARY_DIR}) # For generated shader header

add_executable(matthew main.cpp)
add_library(matthewlib matthew.cpp meshiew.cpp pointiew.cpp ${SHADER_HEADER_NAME})
target_include_directories(matthewlib PUBLIC
        ${NANOGUI_EXTRA_INCS}
        ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(matthewlib surface_mesh matthew_io timercpp)
target_link_libraries(matthewlib nanogui ${NANOGUI_EXTRA_LIBS} lines)
target_link_libraries(matthew matthewlib)

