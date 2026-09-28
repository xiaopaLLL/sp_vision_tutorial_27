cmake_minimum_required(VERSION 3.22)

project(ex3)

find_package(OpenCV 4 REQUIRED)
add_executable(ex3 main.cpp)
target_include_directories(ex3 PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ex3 PRIVATE ${OpenCV_LIBS})
