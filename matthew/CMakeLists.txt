file(GLOB_RECURSE SOURCES "*.cpp")
file(GLOB_RECURSE HEADERS "*.h")

file(GLOB shaders "shaders/*.glsl")

foreach(file ${shaders})
    get_filename_component(SHADER_NAME ${file} NAME_WE)
    set(HEADER_NAME ${CMAKE_BINARY_DIR}/${SHADER_NAME}.h)
    add_custom_command(
            OUTPUT ${HEADER_NAME}
            COMMAND embedfile ${SHADER_NAME} ${file} ${HEADER_NAME}
            DEPENDS ${file} ${CMAKE_SOURCE_DIR}/externals/embedfile/embedfile.cpp
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
    list(APPEND SHADER_HEADERS ${HEADER_NAME})
endforeach()

include_directories(${CMAKE_BINARY_DIR})

add_executable(matthew ${SOURCES} ${HEADERS} ${SHADER_HEADERS})
target_link_libraries(matthew surface_mesh)
target_link_libraries(matthew nanogui ${NANOGUI_EXTRA_LIBS})

