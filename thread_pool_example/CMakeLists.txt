cmake_minimum_required(VERSION 2.8)

project(example)
add_compile_options(-pthread)
find_package (Threads)
add_executable(thread_pool thread_pool_example.cpp)
target_link_libraries(thread_pool ${CMAKE_THREAD_LIBS_INIT})
