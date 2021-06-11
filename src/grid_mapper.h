#ifndef BINARY_BAYES_FILTER_H_
#define BINARY_BAYES_FILTER_H_

#include <vector>

namespace grid_mapper {

using OccupancyGrid = std::vector<std::vector<double>>;

struct Coordinate {
  double x;
  double y;
};

struct Measurement {
  int timestamp;
  std::vector<double> sensor_data;
  bool operator==(const Measurement& rhs) const {
    return timestamp == rhs.timestamp && sensor_data == rhs.sensor_data;
  }
  bool operator!=(const Measurement& rhs) const { return !(rhs == *this); }
};

struct Pose {
  int timestamp;
  double x, y, theta;
  bool operator==(const Pose& rhs) const {
    return timestamp == rhs.timestamp && x == rhs.x && y == rhs.y &&
           theta == rhs.theta;
  }
  bool operator!=(const Pose& rhs) const { return !(rhs == *this); }
};

class MapGrid {
 public:
  MapGrid(double cell_width, double cell_height, double map_width,
          double map_height, double log_odds_unk);
  Coordinate CellCenter(int i, int j) const;
  OccupancyGrid& GetOccupancyGrid();
  size_t SizeRows() const;
  size_t SizeColumns() const;

 private:
  double cell_width_;
  double cell_height_;
  double log_odds_unk_;
  size_t num_rows_;
  size_t num_columns_;
  OccupancyGrid grid_;
};

class Robot {
 public:
  Robot(double width, double height, double range_min, double range_max);
  const Pose& GetPose();
  void SetPose(Pose new_pose);
  Coordinate ToRobotFrame(const Coordinate& coordinate) const;
  bool InRange(Coordinate coordinate) const;

 private:
  const double width_;
  const double height_;
  const double range_min_;
  const double range_max_;
  Pose pose_;
};

void UpdateOccupancyGrid(MapGrid& map_grid, Robot robot,
                         const Measurement& measurement);

double InverseSensorModel(const Pose& pose, Measurement measurement,
                          Coordinate coordinate);
}  // namespace grid_mapper

#endif  // BINARY_BAYES_FILTER_H_
