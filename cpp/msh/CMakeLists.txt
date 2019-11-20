add_library(mshloader pymesh/MshLoader.cpp pymesh/IOUtils.cpp myloadmsh.cpp)
target_include_directories(mshloader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(mshloader surface_mesh)
target_include_directories(mshloader PRIVATE
        ../../externals/nanogui/ext/eigen
        ${CMAKE_CURRENT_SOURCE_DIR}/pymesh
)