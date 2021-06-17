FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG release-1.10.0
)
FetchContent_MakeAvailable(googletest)

add_executable(
        UNIT_TEST
        test.cpp
        test_grid_mapper.cpp
        test_grid_mapper_io.cpp
)
target_link_libraries(
        UNIT_TEST
        PRIVATE
        pthread
        gtest
        gtest_main
        MAPPER
        MAPPER_IO
)
if (BUILD_PLOTS)
    target_sources(UNIT_TEST PRIVATE test_plot.cpp)
    target_link_libraries(UNIT_TEST PRIVATE MATPLOTLIB)
endif ()