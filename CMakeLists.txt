cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

get_filename_component(PROJECTNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
project(${PROJECTNAME})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
LIST(APPEND CMAKE_PREFIX_PATH "/volume/software/common/cmake/$ENV{DLRRM_HOST_PLATFORM}")

find_package(NanoGUI)
find_package(SurfaceMesh)
find_package(Protobuf)

add_definitions(${NANOGUI_EXTRA_DEFS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/externals/cli)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/externals/csv)
include_directories(include)

file(GLOB shaders "shaders/*.glsl")

include_directories(${CMAKE_BINARY_DIR}) # For generated shader header

# Deploy files once to the executable dir
macro(copy_files_once)
    set(FILES_LIST "${ARGN}")
    foreach(SOME_FILE ${FILES_LIST})
        message(STATUS "Copying ${SOME_FILE} to build folder")
        file(COPY ${SOME_FILE} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/data)
    endforeach()
endmacro()

#--- Dummy target just to have data files appear in IDE
file(GLOB_RECURSE DATA "${PROJECT_SOURCE_DIR}/data/*")
add_custom_target(DGP_DATA SOURCES ${DATA})
copy_files_once(${DATA})

add_subdirectory(externals/shader2header)
add_subdirectory(externals/sockpp)
add_subdirectory(externals/timercpp)
set(SHADER_HEADER_NAME ${CMAKE_BINARY_DIR}/shaders_gen.h)
add_custom_command(
        OUTPUT ${SHADER_HEADER_NAME}
        COMMAND shader2header ${SHADER_HEADER_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/shaders
        DEPENDS ${shaders};${CMAKE_CURRENT_SOURCE_DIR}/externals/shader2header/shader2header.cpp
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

add_custom_target(generated_headers DEPENDS ${SHADER_HEADER_NAME})

add_subdirectory(matthew)
add_subdirectory(cpp)

if (${Protobuf_FOUND})
    add_subdirectory(mimp_viewer)
endif()

add_subdirectory(embedding_solver)