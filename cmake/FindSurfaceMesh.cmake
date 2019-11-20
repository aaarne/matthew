set(SURFACEMESH_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/externals/surface_mesh")
add_subdirectory(${SURFACEMESH_INCLUDE_DIR})
target_link_libraries(surface_mesh ${LIBRARIES})
target_include_directories(surface_mesh PUBLIC
        ${SURFACEMESH_INCLUDE_DIR})