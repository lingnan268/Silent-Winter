cmake_minimum_required(VERSION 3.8)
project(camera_calib)

find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

add_executable(calibrate src/calibrate.cpp)
target_link_libraries(calibrate ${OpenCV_LIBS})

add_executable(undistort src/undistort.cpp)
target_link_libraries(undistort ${OpenCV_LIBS})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
