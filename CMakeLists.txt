cmake_minimum_required(VERSION 3.21)
project(astar)

set(CMAKE_CXX_STANDARD 17)


include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(astar main.cpp)

