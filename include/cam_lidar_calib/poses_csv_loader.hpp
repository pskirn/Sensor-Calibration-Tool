#ifndef CAM_LIDAR_CALIB_POSES_CSV_LOADER_HPP
#define CAM_LIDAR_CALIB_POSES_CSV_LOADER_HPP

#include <string>
#include <vector>
#include "cam_lidar_calib/types.hpp"

namespace cam_lidar_calib {

// Parses ACFR/MATLAB-style poses.csv (19 lines per pose, units in mm).
// Returns one PlaneObservation per pose for each sensor.
// Both vectors have equal length; index i is the matched pair for pose i.
struct PosesCsvData {
    std::vector<PlaneObservation> camera;
    std::vector<PlaneObservation> lidar;
};

PosesCsvData loadPosesCsv(const std::string& path);

}  // namespace cam_lidar_calib

#endif
