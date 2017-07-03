cmake_minimum_required(VERSION 3.5)
project(EKF_RADAR_LIDAR)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp ExtendedKalmanFilter.h ExtendedKalmanFilter.cpp measurement.h Tracking.cpp Tracking.h)
add_executable(EKF_RADAR_LIDAR ${SOURCE_FILES})


