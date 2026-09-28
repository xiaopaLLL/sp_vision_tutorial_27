cmake_minimum_required(VERSION 3.16)

# OpenVINO_DIR 由顶层 CMakeLists.txt 统一提供

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_library(auto_aim OBJECT
    armor.cpp
    yolo.cpp
    yolos/yolov5.cpp
)

target_link_libraries(auto_aim io openvino::runtime )
