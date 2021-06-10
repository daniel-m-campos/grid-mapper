cmake_minimum_required(VERSION 3.14)
project()

set(CMAKE_CXX_STANDARD 17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

include_directories(src)
add_subdirectory(src)
add_subdirectory(test)