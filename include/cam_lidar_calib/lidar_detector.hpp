#ifndef LIDAR_DETECTOR_HPP
#define LIDAR_DETECTOR_HPP

#include "types.hpp"
#include <pcl-1.12/pcl/point_types.h>
#include <pcl-1.12/pcl/point_cloud.h>

#include <optional>
#include <string>

namespace cam_lidar_calib 
{

    class LidarDetector {

        public:
            LidarDetector(const CalibrationConfig& config);

            std::optional<PlaneObservation> detect(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, int frame_index);

            std::optional<PlaneObservation> detectFromFile(const std::string& filepath, int frame_index);


        private:

            CalibrationConfig config_;
            
    };
}

#endif