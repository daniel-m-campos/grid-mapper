#ifndef GRID_MAPPER_H_
#define GRID_MAPPER_H_

#include <vector>

namespace grid_mapper {

struct DefaultLogOdds {
  static constexpr double kUnknown = 0;
  static constexpr double kOccupied = 0.4;
  static constexpr double kFree = -0.4;
};

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
          double map_height);
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

struct SensorRange {
  double range_min;
  double range_max;
};

class Robot {
 public:
  Robot(double width, double height, SensorRange senor_range);
  const Pose& GetPose();
  const SensorRange& GetSensorRange();
  void SetPose(Pose new_pose);
  Coordinate ToRobotFrame(const Coordinate& coordinate) const;
  bool InRange(Coordinate coordinate) const;

 private:
  const double width_;
  const double height_;
  const SensorRange sensor_range_;
  Pose pose_;
};

typedef double (*InverseSensorModel)(const Pose& pose,
                                     const Measurement& measurement,
                                     const Coordinate& coordinate,
                                     const SensorRange& sensor_range);
void UpdateOccupancyGrid(MapGrid& map_grid, Robot robot,
                         const Measurement& measurement,
                         InverseSensorModel model = nullptr);

double DefaultInverseSensorModel(const Pose& pose,
                                 const Measurement& measurement,
                                 const Coordinate& coordinate,
                                 const SensorRange& sensor_range);
}  // namespace grid_mapper

#endif  // GRID_MAPPER_H_
