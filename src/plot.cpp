#include <fstream>

#include "grid_mapper.h"
#include "grid_mapper_io.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

void Plot(grid_mapper::MapGrid& map_grid) {
  plt::title("Occupancy Grid");
  plt::xlim<size_t>(0, map_grid.SizeRows());
  plt::ylim<size_t>(0, map_grid.SizeColumns());

  std::vector<double> x(1, 0), y(1, 0);
  const auto& occupancy_grid = map_grid.GetOccupancyGrid();
  for (size_t i = 0; i < map_grid.SizeRows(); i++) {
    for (size_t j = 0; j < map_grid.SizeColumns(); j++) {
      x[0] = i, y[0] = j;
      if (occupancy_grid[i][j] < 0) {
        plt::plot(x, y, "w.");
      } else if (occupancy_grid[i][j] > 0) {
        plt::plot(x, y, "r.");
      } else {
        plt::plot(x, y, "k.");
      }
    }
  }
}

void Simulate(std::ifstream& pose_stream, std::ifstream& measurement_stream,
              grid_mapper::MapGrid& map_grid, grid_mapper::Robot& robot) {
  while (pose_stream.good() && measurement_stream.good()) {
    robot.SetPose(grid_mapper_io::ReadPose(pose_stream));
    UpdateOccupancyGrid(map_grid, robot,
                        grid_mapper_io::ReadMeasurement(measurement_stream));
  }
}

int main() {
  const double range_max = 5000, range_min = 170;
  const double cell_width = 100, cell_height = 100;
  const double map_width = 30000, map_height = 15000;
  const double robot_width = map_width / 5, robot_height = map_height / 3;

  std::ifstream pose_stream{"../data/pose.txt"};
  std::ifstream measurement_stream{"../data/measurement.txt"};

  grid_mapper::MapGrid map_grid{cell_width, cell_height, map_width, map_height};
  grid_mapper::Robot robot{robot_width, robot_height, {range_min, range_max}};

  Simulate(pose_stream, measurement_stream, map_grid, robot);
  Plot(map_grid);
  plt::save("occupancy_grid.png");
  plt::show();
}
