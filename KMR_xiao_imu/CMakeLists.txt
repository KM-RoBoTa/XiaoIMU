cmake_minimum_required(VERSION 3.20)
project(KMR_xiao_imu)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)

# Source files
add_library(${PROJECT_NAME} xiao_imu.cpp)

# Generate example executables
add_executable(example example/example.cpp)
target_link_libraries(example ${PROJECT_NAME})