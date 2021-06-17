#include "grid_mapper.h"

#include <cmath>
#include <vector>

grid_mapper::MapGrid::MapGrid(double cell_width, double cell_height,
                              double map_width, double map_height)
    : cell_width_{cell_width},
      cell_height_{cell_height},
      log_odds_unk_{DefaultLogOdds::kUnknown},
      num_rows_{static_cast<size_t>(ceil(map_width / cell_width))},
      num_columns_{static_cast<size_t>(ceil(map_height / cell_height))},
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

void grid_mapper::UpdateOccupancyGrid(MapGrid& map_grid, Robot& robot,
                                      const Measurement& measurement,
                                      InverseSensorModel model) {
  if (not model) model = DefaultInverseSensorModel;

  for (size_t i = 0; i < map_grid.SizeRows(); ++i) {
    for (size_t j = 0; j < map_grid.SizeColumns(); ++j) {
      auto cell_center = map_grid.CellCenter(i, j);
      cell_center = robot.ToRobotFrame(cell_center);
      if (robot.InRange(cell_center)) {
        map_grid.GetOccupancyGrid()[i][j] +=
            model(robot.GetPose(), measurement, cell_center,
                  robot.GetSensorRange()) -
            DefaultLogOdds::kUnknown;
      }
    }
  }
}

double grid_mapper::DefaultInverseSensorModel(const Pose& pose,
                                              const Measurement& measurement,
                                              const Coordinate& coordinate,
                                              const SensorRange& sensor_range) {
  double z_k, theta_k, sensor_theta;
  double min_delta = -1;
  double alpha = 200, beta = 20;

  const auto dx = coordinate.x - pose.x;
  const auto dy = coordinate.y - pose.y;
  const auto r = sqrt(pow(dx, 2) + pow(dy, 2));
  const auto phi = atan2(dy, dx) - pose.theta;

  // Scaling Measurement to [-90 -37.5 -22.5 -7.5 7.5 22.5 37.5 90]
  for (int i = 0; i < 8; i++) {
    if (i == 0) {
      sensor_theta = -90 * (M_PI / 180);
    } else if (i == 1) {
      sensor_theta = -37.5 * (M_PI / 180);
    } else if (i == 6) {
      sensor_theta = 37.5 * (M_PI / 180);
    } else if (i == 7) {
      sensor_theta = 90 * (M_PI / 180);
    } else {
      sensor_theta = (-37.5 + (i - 1) * 15) * (M_PI / 180);
    }

    if (fabs(phi - sensor_theta) < min_delta || min_delta == -1) {
      z_k = measurement.sensor_data[i];
      theta_k = sensor_theta;
      min_delta = fabs(phi - sensor_theta);
    }
  }
  if (r > fmin(sensor_range.range_max, z_k + alpha / 2) ||
      fabs(phi - theta_k) > beta / 2 || z_k > sensor_range.range_max ||
      z_k < sensor_range.range_min) {
    return DefaultLogOdds::kUnknown;
  } else if (z_k < sensor_range.range_max && fabs(r - z_k) < alpha / 2) {
    return DefaultLogOdds::kOccupied;
  } else if (r <= z_k) {
    return DefaultLogOdds::kFree;
  } else {
    return DefaultLogOdds::kUnknown;
  }
}

grid_mapper::Robot::Robot(double width, double height, SensorRange senor_range)
    : width_{width}, height_{height}, sensor_range_{senor_range}, pose_{} {}

bool grid_mapper::Robot::InRange(Coordinate coordinate) const {
  const auto d =
      sqrt(pow(coordinate.x - pose_.x, 2) + pow(coordinate.y - pose_.y, 2));
  return d <= sensor_range_.range_max;
}

void grid_mapper::Robot::SetPose(Pose new_pose) { pose_ = new_pose; }

const grid_mapper::Pose& grid_mapper::Robot::GetPose() { return pose_; }

grid_mapper::Coordinate grid_mapper::Robot::ToRobotFrame(
    const Coordinate& coordinate) const {
  const auto x_prime = coordinate.x - width_;
  const auto y_prime = coordinate.y + height_;
  return {x_prime, y_prime};
}

const grid_mapper::SensorRange& grid_mapper::Robot::GetSensorRange() {
  return sensor_range_;
}
