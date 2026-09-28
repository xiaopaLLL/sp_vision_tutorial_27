cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)

add_library(io STATIC
    camera.cpp
    video_camera.cpp
)

# 相机路径默认关闭：hikrobot.cpp 要链 MVS SDK 的 .so，多数学生机器上没有。
# 关掉它才能保证 video 路径人人能编。需要真相机时用 -DWITH_HIKROBOT=ON 打开。
option(WITH_HIKROBOT "Build the HikRobot camera source (requires the MVS SDK)" OFF)

if(WITH_HIKROBOT)
  target_sources(io PRIVATE hikrobot/hikrobot.cpp)
  target_compile_definitions(io PUBLIC WITH_HIKROBOT)
  target_include_directories(io PUBLIC hikrobot/include)

  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    target_link_directories(io PUBLIC hikrobot/lib/amd64)
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_link_directories(io PUBLIC hikrobot/lib/arm64)
  else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}!")
  endif()

  target_link_libraries(io PUBLIC MvCameraControl usb-1.0)
endif()

target_link_libraries(io PUBLIC yaml-cpp tools)
