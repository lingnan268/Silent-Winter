cmake_minimum_required(VERSION 3.8)
project(energy_rune)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(rune_detect src/rune_detect.cpp)
target_link_libraries(rune_detect ${OpenCV_LIBS})
