#ifndef GRID_MAPPER_IO_H_
#define GRID_MAPPER_IO_H_

#include "fstream"
#include "grid_mapper.h"

namespace grid_mapper_io {

grid_mapper::Measurement ReadMeasurement(std::ifstream& file_stream);

grid_mapper::Pose ReadPose(std::ifstream& file_stream);

grid_mapper::OccupancyGrid ReadGrid(const std::string& filename);

};  // namespace grid_mapper_io

#endif  // GRID_MAPPER_IO_H_
