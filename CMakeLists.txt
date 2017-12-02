cmake_minimum_required(VERSION 3.8)
project(optimization)

set(CMAKE_CXX_STANDARD 11)

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(include ${CERES_INCLUDE_DIRS})

AUX_SOURCE_DIRECTORY(src DIR_SRCS)
#file(GLOB SOURCES src/*.cpp)
#add_library(optimization ${SOURCES})

#set(SOURCE ${DIR_SRCS})
add_executable(${PROJECT_NAME} ${DIR_SRCS})
target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES} ${OpenCV_LIBS})


