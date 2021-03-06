#include <cmath>

#include "grid_mapper_io.h"
#include "gtest/gtest.h"

using namespace grid_mapper;
using namespace grid_mapper_io;

TEST(TestGridMapperIO, TestReadMeasurement) {
  std::ifstream file_stream{"../data/measurement.txt"};
  auto actual = ReadMeasurement(file_stream);
  Measurement expected{1686487,
                       {5110, 5110, 2320, 2360, 5110, 5110, 2160, 1190}};
  ASSERT_EQ(actual, expected);
}

TEST(TestGridMapperIO, TestReadPose) {
  std::ifstream file_stream{"../data/pose.txt"};
  std::string line;
  int line_number = 47;
  for (int i = 0; i < line_number; ++i) {
    getline(file_stream, line);
  }
  auto actual = ReadPose(file_stream);
  Pose expected{1697590, 10, 0, (3600 / 10) * (M_PI / 180)};
  ASSERT_EQ(actual, expected);
}

TEST(TestGridMapperIO, TestReadGrid) {
  auto grid = ReadGrid("../data/grid1.txt");
  ASSERT_EQ(grid.size(), 300);
  ASSERT_EQ(grid[0].size(), 150);
  ASSERT_NEAR(grid[23][16], 19.6, 1e-6);
}