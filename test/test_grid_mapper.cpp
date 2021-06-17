#include <cmath>
#include <fstream>

#include "grid_mapper.h"
#include "grid_mapper_io.h"
#include "gtest/gtest.h"

using namespace grid_mapper;
using namespace grid_mapper_io;

class MapperFixture : public ::testing::Test {
 public:
  static constexpr double range_max = 5000, range_min = 170;
  static constexpr double cell_width = 100, cell_height = 100;
  static constexpr double map_width = 30000, map_height = 15000;
  static constexpr double robot_width = map_width / 5,
                          robot_height = map_height / 3;
  void AssertEqual(const OccupancyGrid& actual, const OccupancyGrid& expected) {
    ASSERT_EQ(actual.size(), expected.size());
    ASSERT_EQ(actual[0].size(), expected[0].size());
    for (size_t i = 0; i < expected.size(); ++i) {
      for (size_t j = 0; j < expected[0].size(); ++j) {
        ASSERT_NEAR(actual[i][j], expected[i][j], 1e-6);
      }
    }
  }
};

TEST_F(MapperFixture, DataTestWithConstantModel) {
  std::ifstream pose_stream{"../data/pose.txt"};
  std::ifstream measurement_stream{"../data/measurement.txt"};

  MapGrid map_grid{cell_width, cell_height, map_width, map_height};
  Robot robot{robot_width, robot_height, {range_min, range_max}};
  auto model = [](const Pose& pose, const Measurement& measurement,
                  const Coordinate& coordinate,
                  const SensorRange& sensor_range) -> double { return 0.4; };
  while (pose_stream.good() && measurement_stream.good()) {
    robot.SetPose(grid_mapper_io::ReadPose(pose_stream));
    UpdateOccupancyGrid(map_grid, robot,
                        grid_mapper_io::ReadMeasurement(measurement_stream),
                        model);
  }
  AssertEqual(map_grid.GetOccupancyGrid(), ReadGrid("../data/grid1.txt"));
}

TEST_F(MapperFixture, DataTestWithDefaultModel) {
  std::ifstream pose_stream{"../data/pose.txt"};
  std::ifstream measurement_stream{"../data/measurement.txt"};
  auto expected = ReadGrid("../data/grid2.txt");

  MapGrid map_grid{cell_width, cell_height, map_width, map_height};
  Robot robot{robot_width, robot_height, {range_min, range_max}};
  while (pose_stream.good() && measurement_stream.good()) {
    robot.SetPose(grid_mapper_io::ReadPose(pose_stream));
    UpdateOccupancyGrid(map_grid, robot,
                        grid_mapper_io::ReadMeasurement(measurement_stream));
  }
  AssertEqual(map_grid.GetOccupancyGrid(), ReadGrid("../data/grid2.txt"));
}