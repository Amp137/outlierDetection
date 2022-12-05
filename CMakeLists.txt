cmake_minimum_required(VERSION 3.0)

project(outlierDetection)

SET(CMAKE_BUILD_TYPE Debug)
ADD_COMPILE_OPTIONS(-std=c++11 -g)
MESSAGE(STATUS "BUILD TYPE:" ${CMAKE_BUILD_TYPE})

find_package(Eigen3 REQUIRED)
include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} SHARED 
    src/Frame.cpp
        src/OrientationSolver.cpp
)

target_link_libraries(${PROJECT_NAME} ${ALL_TARGET_LIBRARIES})

add_executable(main src/main.cpp)
target_link_libraries(main ${PROJECT_NAME})

