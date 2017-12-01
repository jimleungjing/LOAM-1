cmake_minimum_required(VERSION 2.8.3)
project(odometry)
find_package(PCL REQUIRED) #配置PCL库
include_directories(${PCL_INCLUDE_DIRS}) #配置头文件路径..例#include
add_executable(odometry odometry.cpp main.cpp)
target_link_libraries(odometry ${PCL_LIBRARIES}) #链接PCL库
set( CMAKE_BUILD_TYPE Debug ) #如果需要调试就加上这行
