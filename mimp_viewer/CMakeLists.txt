add_executable(mimp_viewer mimp_viewer.cpp ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp mimp_state.pb.cc)
target_include_directories(mimp_viewer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(mimp_viewer matthew ${Protobuf_LIBRARY})