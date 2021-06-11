#include <cmath>
#include <fstream>

#include "grid_mapper.h"
#include "grid_mapper_io.h"
#include "gtest/gtest.h"

using namespace grid_mapper;
using namespace grid_mapper_io;

class MapperFixture : public ::testing::Test {
 public:
  // Sensor characteristic: Min and Max ranges of the beams
  static constexpr double range_max = 5000, range_min = 170;
  // Defining free cells(lfree), occupied cells(locc), unknown
  // cells(l0) log odds values
  static constexpr double l0 = 0, locc = 0.4, lfree = -0.4;
  // OccupancyGrid dimensions
  static constexpr double cell_width = 100, cell_height = 100;
  // OccupancyGrid dimensions
  static constexpr double map_width = 30000, map_height = 15000;
  // Robot size with respect to the map
  static constexpr double robot_width = map_width / 5,
                          robot_height = map_height / 3;
};

TEST_F(MapperFixture, DataTest) {
  std::ifstream pose_stream{"data/pose.txt"};
  std::ifstream measurement_stream{"data/measurement.txt"};
  auto expected = ReadGrid("data/grid1.txt");
  MapGrid map_grid{cell_width, cell_height, map_width, map_height, l0};
  Robot robot{robot_width, robot_height, range_min, range_max};
  while (pose_stream.good() && measurement_stream.good()) {
    robot.SetPose(grid_mapper_io::ReadPose(pose_stream));
    UpdateOccupancyGrid(map_grid, robot,
                        grid_mapper_io::ReadMeasurement(measurement_stream));
  }
  const auto actual = map_grid.GetOccupancyGrid();
  ASSERT_EQ(actual.size(), expected.size());
  ASSERT_EQ(actual[0].size(), expected[0].size());
  for (int i = 0; i < map_grid.SizeRows(); ++i) {
    for (int j = 0; j < map_grid.SizeColumns(); ++j) {
      ASSERT_NEAR(actual[i][j], expected[i][j], 1e-6);
    }
  }
}