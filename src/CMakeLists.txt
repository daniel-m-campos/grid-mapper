add_library(
        MAPPER
        grid_mapper.cpp
)
add_library(
        MAPPER_IO
        grid_mapper_io.cpp
)
target_link_libraries(MAPPER_IO PRIVATE MAPPER)

if (BUILD_PLOTS)
    add_executable(PLOT plot.cpp)
    target_link_libraries(PLOT MAPPER MAPPER_IO MATPLOTLIB)
endif ()
