# 设置 CMake 最低版本要求
cmake_minimum_required(VERSION 3.16)

# 设置项目名称
project(qi_sdk_project)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

# 添加可执行文件
add_executable(qi_sdk_project test.cpp)
target_link_libraries(qi_sdk_project qi_sdk)

