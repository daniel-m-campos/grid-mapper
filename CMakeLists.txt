cmake_minimum_required(VERSION 3.14)
project(GridMapper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
string(APPEND CMAKE_CXX_FLAGS "-Wall")
set(BUILD_SHARED_LIBS YES)

include_directories(src)

option(BUILD_PLOTS "Build plot target")
if (BUILD_PLOTS)
    message(STATUS "Building Plots")
    find_package(
            Python 3.6
            REQUIRED Interpreter Development
            OPTIONAL_COMPONENTS NumPy
    )
    include(FetchContent)
    FetchContent_Declare(
            matplotlibcpp
            GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
            GIT_TAG ef0383f1315d32e0156335e10b82e90b334f6d9f
    )
    FetchContent_MakeAvailable(matplotlibcpp)
    add_library(MATPLOTLIB INTERFACE)
    target_include_directories(
            MATPLOTLIB INTERFACE
            ${matplotlibcpp_SOURCE_DIR}
            ${Python_INCLUDE_DIRS}
    )
    target_link_libraries(MATPLOTLIB INTERFACE ${Python_LIBRARIES})
endif ()

add_subdirectory(src)
add_subdirectory(test)

file(COPY data DESTINATION .)