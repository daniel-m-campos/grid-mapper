#include "grid_mapper_io.h"

#include <sstream>

grid_mapper::Measurement grid_mapper_io::ReadMeasurement(
    std::ifstream &file_stream) {
  int timestamp = 0;
  std::vector<double> sensor_data;
  std::string line;
  if (getline(file_stream, line)) {
    std::istringstream string_stream{line};
    if (not(string_stream >> timestamp)) return grid_mapper::Measurement();
    double sensor;
    while (string_stream >> sensor) {
      sensor_data.push_back(sensor);
    }
  }
  return grid_mapper::Measurement{timestamp, sensor_data};
}

grid_mapper::Pose grid_mapper_io::ReadPose(std::ifstream &file_stream) {
  int timestamp = 0;
  double x = 0, y = 0, theta = 0;
  std::string line;
  if (getline(file_stream, line)) {
    std::istringstream string_stream{line};
    if (not(string_stream >> timestamp >> x >> y >> theta))
      return grid_mapper::Pose();
  }
  return grid_mapper::Pose{timestamp, x, y, theta};
}

grid_mapper::OccupancyGrid grid_mapper_io::ReadGrid(
    const std::string &filename) {
  std::ifstream file_stream{filename};
  grid_mapper::OccupancyGrid grid;
  if (not file_stream) return grid;
  std::string line;
  double value;
  while (getline(file_stream, line)) {
    std::istringstream string_stream{line};
    std::vector<double> row;
    while (string_stream >> value) {
      row.emplace_back(value);
    }
    grid.emplace_back(row);
  }
  return grid;
}
