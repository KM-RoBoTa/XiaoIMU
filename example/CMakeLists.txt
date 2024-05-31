cmake_minimum_required(VERSION 3.20)

# Set project name
project(imu)

message(STATUS "Project name: " ${PROJECT_NAME})
message(STATUS "Source dir: " ${PROJECT_SOURCE_DIR})
message(STATUS "Header files: " ${PROJECT_SOURCE_DIR}/../readIMU_cppCode)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)

# Create list of additional sources in a different folder than the main.cpp
set(additional_sources
    xiao_imu.cpp
)
list(TRANSFORM additional_sources PREPEND "../readIMU_cppCode/")

# Source files
add_executable(${PROJECT_NAME}
            main.cpp
            ${additional_sources}
)

# Add header files
include_directories(
    ${PROJECT_SOURCE_DIR}/../readIMU_cppCode
)
