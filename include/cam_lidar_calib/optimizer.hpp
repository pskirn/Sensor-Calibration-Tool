#ifndef CAM_LIDAR_CALIB_OPTIMIZER_HPP
#define CAM_LIDAR_CALIB_OPTIMIZER_HPP

#include <vector>
#include "cam_lidar_calib/types.hpp"

namespace cam_lidar_calib {

// Solves for the rigid transform mapping LiDAR frame -> camera frame:
//   p_camera = R * p_lidar + t
// Input vectors must be the same length; index i is the matched pose.
CalibrationResult solveExtrinsic(const std::vector<PlaneObservation>& cam_obs,
                                 const std::vector<PlaneObservation>& lid_obs);

}  // namespace cam_lidar_calib

#endif
