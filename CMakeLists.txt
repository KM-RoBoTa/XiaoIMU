cmake_minimum_required(VERSION 3.20)
project(sensors)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)

# Source files
add_executable(sensors
            main.cpp
            xiao_imu.cpp
            time.cpp
)