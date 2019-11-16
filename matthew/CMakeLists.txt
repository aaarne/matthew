file(GLOB_RECURSE SOURCES "*.cpp")
file(GLOB_RECURSE HEADERS "*.h")

file(GLOB shaders "../shaders/*.glsl")

set(SHADER_HEADER_NAME ${CMAKE_BINARY_DIR}/shaders_gen.h)
add_custom_command(
        OUTPUT ${SHADER_HEADER_NAME}
        COMMAND shader2header ${SHADER_HEADER_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/../shaders
        DEPENDS ${shaders};${CMAKE_CURRENT_SOURCE_DIR}/../externals/shader2header/shader2header.cpp
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

include_directories(${CMAKE_BINARY_DIR})

add_executable(matthew ${SOURCES} ${HEADERS} ${SHADER_HEADER_NAME})
target_link_libraries(matthew surface_mesh)
target_link_libraries(matthew nanogui ${NANOGUI_EXTRA_LIBS})

