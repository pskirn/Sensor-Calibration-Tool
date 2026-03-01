#ifndef CAMERA_DETECTOR_HPP
#define CAMERA_DETECTOR_HPP

#include "types.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <optional>

namespace cam_lidar_calib {

    class CameraDetector {

        public:
            CameraDetector(const cam_lidar_calib::CalibrationConfig& config, 
                            const cam_lidar_calib::CameraIntrinsics& intrinsics);

            
            std::optional<cam_lidar_calib::PlaneObservation> detect(const cv::Mat& image, int frame_index);


            cv::Mat drawDetection(const cv::Mat& image, 
                                    const std::vector<cv::Point2f>& corners) const ;        



        private:
            // Stored input configurations 
            cam_lidar_calib::CalibrationConfig config_;
            cam_lidar_calib::CameraIntrinsics intrinsics_;

            // OpenCV-format versions
            cv::Mat camera_matrix_;
            cv::Mat dist_coeffs_;

            // Board geometry 
            cv::Size board_size_;
            double square_size_;

            // Precomputed 3D object points 
            std::vector<cv::Point3f> object_points_;

    };

}

#endif