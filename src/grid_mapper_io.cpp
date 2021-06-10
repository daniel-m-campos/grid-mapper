#include "grid_mapper_io.h"

#include <sstream>

grid_mapper::Measurement grid_mapper_io::ReadMeasurement(
    std::ifstream &file_stream) {
  int timestamp = 0;
  std::vector<double> sensor_data;
  std::string line;
  if (getline(file_stream, line)) {
    std::istringstream string_stream(line);
    if (not (string_stream >> timestamp)) return grid_mapper::Measurement();
    double sensor;
    while (string_stream >> sensor) {
      sensor_data.push_back(sensor);
    }
  }
  return grid_mapper::Measurement{timestamp, sensor_data};
}
