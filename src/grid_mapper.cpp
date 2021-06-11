#include "grid_mapper.h"

#include <cmath>
#include <vector>

grid_mapper::MapGrid::MapGrid(double cell_width, double cell_height,
                              double map_width, double map_height,
                              double log_odds_unk)
    : cell_width_{cell_width},
      cell_height_{cell_height},
      num_rows_{static_cast<size_t>(ceil(map_width / cell_width))},
      num_columns_{static_cast<size_t>(ceil(map_height / cell_height))},
      log_odds_unk_{log_odds_unk},
      grid_{num_rows_, std::vector<double>(num_columns_, log_odds_unk_)} {}

grid_mapper::Coordinate grid_mapper::MapGrid::CellCenter(int i, int j) const {
  auto x = (i + 0.5) * cell_width_;
  auto y = -(j + 0.5) * cell_height_;
  return {x, y};
}

size_t grid_mapper::MapGrid::SizeRows() const { return num_rows_; }

size_t grid_mapper::MapGrid::SizeColumns() const { return num_columns_; }

grid_mapper::OccupancyGrid& grid_mapper::MapGrid::GetOccupancyGrid() {
  return grid_;
}

void grid_mapper::UpdateOccupancyGrid(
    grid_mapper::MapGrid& map_grid, Robot robot,
    const grid_mapper::Measurement& measurement) {
  for (int i = 0; i < map_grid.SizeRows(); ++i) {
    for (int j = 0; j < map_grid.SizeColumns(); ++j) {
      auto cell_center = map_grid.CellCenter(i, j);
      cell_center = robot.ToRobotFrame(cell_center);
      if (robot.InRange(cell_center)) {
        map_grid.GetOccupancyGrid()[i][j] +=
            InverseSensorModel(robot.GetPose(), Measurement(), Coordinate());
      }
    }
  }
}

double grid_mapper::InverseSensorModel(const Pose& pose,
                                       Measurement measurement,
                                       Coordinate coordinate) {
  // You will be coding this section in the upcoming concept!
  return 0.4;
}

grid_mapper::Robot::Robot(double width, double height, double range_min,
                          double range_max)
    : width_{width},
      height_{height},
      range_min_{range_min},
      range_max_{range_max},
      pose_{} {}

bool grid_mapper::Robot::InRange(grid_mapper::Coordinate coordinate) const {
  const auto d =
      sqrt(pow(coordinate.x - pose_.x, 2) + pow(coordinate.y - pose_.y, 2));
  return d <= range_max_;
}

void grid_mapper::Robot::SetPose(grid_mapper::Pose new_pose) {
  pose_ = new_pose;
}

const grid_mapper::Pose& grid_mapper::Robot::GetPose() { return pose_; }

grid_mapper::Coordinate grid_mapper::Robot::ToRobotFrame(
    const grid_mapper::Coordinate& coordinate) const {
  const auto x_prime = coordinate.x - width_;
  const auto y_prime = coordinate.y + height_;
  return {x_prime, y_prime};
}
