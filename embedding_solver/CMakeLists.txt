add_executable(embedding_solver embedding_solver.cpp main.cpp)
target_include_directories(embedding_solver PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
        ${NANOGUI_EXTRA_INCS})
target_link_libraries(embedding_solver matthew)
