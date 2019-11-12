cmake_minimum_required(VERSION 3.0)

get_filename_component(PROJECTNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
project(${PROJECTNAME})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include(cmake/ConfigureCompiler.cmake)
find_package(NanoGUI)
find_package(SurfaceMesh)

add_definitions(${NANOGUI_EXTRA_DEFS})
include_directories(${NANOGUI_EXTRA_INCS})

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

add_subdirectory(meshiew)
