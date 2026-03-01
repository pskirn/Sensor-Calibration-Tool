#include <optional>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "cam_lidar_calib/camera_detector.hpp"

namespace cam_lidar_calib {

    CameraDetector::CameraDetector(const CalibrationConfig& config, 
                                            const CameraIntrinsics& intrinsics) {

                                        

                config_ = config;
                intrinsics_ = intrinsics;

                board_size_ = cv::Size(config.cols, config.rows);
                square_size_ = config.square;

                object_points_.clear();


                for (int rows = 0; rows < board_size_.height; ++rows) {

                    for (int cols = 0; cols < board_size_.width; ++cols) {

                        cv::Point3f point(
                            static_cast<float>(cols) * square_size_,
                            static_cast<float>(rows) * square_size_,
                            0.0f
                        );

                        object_points_.push_back(point);

                    }
                }

                camera_matrix_ = cv::Mat(3, 3, CV_64F);

                for (int i = 0; i < 3; ++i) {

                    for (int j = 0; j <3; ++j) {

                        camera_matrix_.at<double>(i, j) = intrinsics_.K(i, j);
                    }
                }

                const int n = static_cast<int>(intrinsics_.distortionCoeffs.size());

                dist_coeffs_= cv::Mat::zeros(1, n, CV_64F);

                for (int i = 0; i < n; ++i) {

                    dist_coeffs_.at<double>(0, i) = intrinsics_.distortionCoeffs(i);
                }
    }

    cv::Mat CameraDetector::drawDetection(const cv::Mat& image, 
                                            const std::vector<cv::Point2f>& corners) const {


                        cv::Mat output = image.clone();

                        if (output.channels() == 1) {
                            
                            cv::cvtColor(output, output, cv::COLOR_BGR2GRAY);
                        }

                    if (!corners.empty() && corners.size() == static_cast<size_t>(board_size_.area())) {
                            cv::drawChessboardCorners(output, board_size_, corners, true);
                        }

                        return output;

    }


    std::optional<cam_lidar_calib::PlaneObservation> CameraDetector::detect(const cv::Mat& image, int frame_index) {

        if (image.empty()) {
            return std::nullopt;
        }


        cv::Mat gray;

        if (image.channels() == 3) {
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        } else {
            gray = image;
        }

        std::vector<cv::Point2f> image_points;
        int flags = cv::CALIB_CB_ADAPTIVE_THRESH |
                    cv::CALIB_CB_NORMALIZE_IMAGE |
                    cv::CALIB_CB_FAST_CHECK;

        bool found = cv::findChessboardCorners(gray, board_size_, image_points, flags);

        if (!found || image_points.size() != static_cast<size_t>(board_size_.area())) {
            return std::nullopt;
        }

        // Refine corner locations (strongly recommended)
        cv::TermCriteria criteria(
            cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER,
            30, 0.001
        );
        cv::cornerSubPix(gray, image_points,
                        cv::Size(11,11), cv::Size(-1,-1),
                        criteria);

        // Solve PnP
        cv::Vec3d rvec, tvec;
        bool ok = cv::solvePnP(object_points_, image_points,
                            camera_matrix_, dist_coeffs_,
                            rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

        if (!ok) {
            return std::nullopt;
        }

        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        
        Eigen::Vector3d normal(
            rotation_matrix.at<double>(0, 2),
            rotation_matrix.at<double>(1, 2),
            rotation_matrix.at<double>(2, 2)
        );
        normal.normalize();

        Eigen::Vector3d tvec_eigen(tvec[0], tvec[1], tvec[2]);
        double distance = normal.dot(tvec_eigen);

        if (distance < 0) {

            normal = -normal;
            distance = -distance;
        }

        cam_lidar_calib::PlaneObservation obs(
            normal,
            distance,
            cam_lidar_calib::SensorType::CAMERA,
            {},                     // points — fill later if needed
            frame_index
        );
        
        return obs;
    }

}

        