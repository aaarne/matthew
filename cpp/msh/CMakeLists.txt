add_library(mshloader MshLoader.cpp IOUtils.cpp myloadmsh.cpp)
target_include_directories(mshloader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(mshloader surface_mesh)
target_include_directories(mshloader PRIVATE
        ../../externals/nanogui/ext/eigen
)