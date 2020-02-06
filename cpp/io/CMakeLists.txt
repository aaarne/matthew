add_library(matthew_io pymesh/MshLoader.cpp pymesh/IOUtils.cpp myloadmsh.cpp trajectory_reader.cpp simulink_receiver.cpp)
target_include_directories(matthew_io PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(matthew_io surface_mesh sockpp)
target_include_directories(matthew_io PRIVATE
        ../../externals/nanogui/ext/eigen
        ${CMAKE_CURRENT_SOURCE_DIR}/pymesh
)