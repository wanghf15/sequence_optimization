cmake_minimum_required(VERSION 3.8)
project(optimization)

set(CMAKE_CXX_STANDARD 11)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

set(SOURCE_FILES main.cpp json.h)
add_executable(optimization ${SOURCE_FILES})
target_link_libraries(optimization ${CERES_LIBRARIES})