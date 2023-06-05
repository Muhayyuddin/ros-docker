cmake_minimum_required(VERSION 3.0.2)
project(spin_base VERSION 0.0.1)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

add_library(spin_base src/vehicle.cpp src/threaded_flag.cpp src/canopen_msg.cpp)
target_include_directories(spin_base PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/)
target_link_libraries(spin_base PUBLIC pthread)
