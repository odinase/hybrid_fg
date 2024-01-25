cmake_minimum_required(VERSION 3.22)
project(hybrid_fg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(range-v3 REQUIRED)
find_package(GTSAM REQUIRED)

add_subdirectory(src)