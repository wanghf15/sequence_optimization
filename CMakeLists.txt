cmake_minimum_required(VERSION 3.8)
project(optimization)

set(CMAKE_CXX_STANDARD 11)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

set(SOURCE_FILES main.cpp src/json.h draw_util.cpp draw_util.h src/utils.h include/ls_optimization.cpp src/ls_optimization.h src/system_parameters.h include/log_error_info.cpp src/log_error_info.h src/cost_function.h)
add_executable(ls_optimization ${SOURCE_FILES})
target_link_libraries(ls_optimization ${CERES_LIBRARIES})