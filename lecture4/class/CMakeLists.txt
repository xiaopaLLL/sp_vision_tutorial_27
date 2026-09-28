cmake_minimum_required(VERSION 3.16.3)

project(lecture4)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)

# OpenVINO 路径只在顶层定义一次，子目录不再各自硬编码
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")
find_package(OpenVINO REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/logs)

add_executable(main src/main.cpp)
target_link_libraries(main
    ${OpenCV_LIBS} fmt::fmt yaml-cpp spdlog::spdlog tools io auto_aim)
