#include "gtest/gtest.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

TEST(TestMatplotlibcpp, SimplePlotTest) {
  plt::plot({1.0, 3.0, 2.0, 4.0});
  plt::save("../data/test.png");
}