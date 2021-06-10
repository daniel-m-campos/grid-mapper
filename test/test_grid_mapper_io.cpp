#include <cmath>

#include "grid_mapper_io.h"
#include "gtest/gtest.h"

TEST(TestGridMapperIO, TestReadMeasurement) {
  using namespace grid_mapper;
  using namespace grid_mapper_io;
  std::ifstream file_stream{"data/measurement.txt"};
  auto actual = ReadMeasurement(file_stream);
  Measurement expected{1686487,
                       {5110, 5110, 2320, 2360, 5110, 5110, 2160, 1190}};
  ASSERT_EQ(actual, expected);
}