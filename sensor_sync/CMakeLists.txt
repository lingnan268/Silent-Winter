cmake_minimum_required(VERSION 3.10)
project(sensor_sync)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 线程库
find_package(Threads REQUIRED)

# 头文件目录
include_directories(${PROJECT_SOURCE_DIR}/include)

# 可执行文件
add_executable(sensor_sync src/main.cpp)
target_link_libraries(sensor_sync Threads::Threads)
