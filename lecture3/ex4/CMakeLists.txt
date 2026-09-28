cmake_minimum_required(VERSION 3.22)

project(ex4)

find_package(Threads REQUIRED)
add_executable(ex4 main.cpp)
target_link_libraries(ex4 PRIVATE Threads::Threads)
