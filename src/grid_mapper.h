#ifndef BINARY_BAYES_FILTER_H_
#define BINARY_BAYES_FILTER_H_

#include <vector>
namespace grid_mapper {

using LogOddsGrid = std::vector<std::vector<double>>;

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

void OccupancyGridMapping(Measurement measurement, Pose pose,
                          LogOddsGrid& grid);

double InverseSensorModel(double x, double y, double theta, double xi,
                          double yi, double sensor_data[]);
}  // namespace grid_mapper

#endif  // BINARY_BAYES_FILTER_H_
