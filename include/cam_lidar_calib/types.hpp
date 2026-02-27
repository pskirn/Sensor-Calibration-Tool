#ifndef TYPES_HPP
#define TYPES_HPP

#include <iostream>
#include <stdlib.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
#include <string>
#include <optional>

#include <opencv2/core.hpp>

#define EIGEN_MAKE_ALIGNED_OPERATOR_NEW


namespace cam_lidar_calib {

    enum class SensorType{
        CAMERA, 
        LIDAR
    };

    struct CameraIntrinsics
    {
        /* data */
        Eigen::Matrix3d K;
        Eigen::VectorXd distortionCoeffs;
        int height, width;

        bool isValid() const{

            if ((K(0,0) <= 0) || K(1,1) <= 0)
                return false;

            if ((width == 0) || (height == 0))
                return false;

            return true;

        };

        static CameraIntrinsics fromYaml(const std::string& path);

    };

    struct PlaneObservation
    {
        Eigen::Vector3d normal;
        double distance;

        SensorType sensor_type;

        std::vector<Eigen::Vector3d> points;

        int frame_index;

        PlaneObservation (const Eigen::Vector3d& normal_in,
                            double distance_in,
                        SensorType sensor_type_in,
                        std::vector<Eigen::Vector3d> raw_points_in,
                        int frame_index_in)

            : normal(normal_in.normalized()),
            distance(distance_in),
            sensor_type(sensor_type_in),
            points(raw_points_in),
            frame_index(frame_index_in)
        {
            const double norm = normal.norm();
            if (norm == 0.0)
            {
                throw std::runtime_error("PlaneObservation: normal vector has zero length");
            }

            normal.normalize();

        };


    };

    struct PlanePair
    {
        int camera_obs, lidar_obs;

        bool isValid() const {

            if ((camera_obs >= 0) && (lidar_obs >= 0))
                return true;
        };
    };
    
    struct CalibrationResult
    {
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        Eigen::Vector3d  t;
        

        double error;
        int poses;

        std::vector<double> residuals;

        Eigen::Matrix4d getHomogeneous() const
        {
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

            T.block<3,3>(0, 0) = R;
            T.block<3,1>(0, 3) = t;
            return T;
        }

        Eigen::Quaterniond getQuaternion() const
        {
            return Eigen::Quaterniond(R);
             
        }

        Eigen::Vector3d getEulerDegrees() const
        {
            Eigen::Vector3d rpy = R.eulerAngles(0, 1, 2);
            rpy = rpy * (180.0 / M_PI);
            return rpy;
        }

        CalibrationResult inverse() const
        {
            CalibrationResult output;
            Eigen::Matrix3d R_inv = R.transpose();
            Eigen::Vector3d t_inv = - R_inv * t;
            output.R = R_inv;
            output.t = t_inv;

            output.error = this->error;
            output.residuals = this->residuals;
            output.poses = this->poses;

            return output;
        }

    };


    struct CalibrationConfig {

        int rows, cols;
        double square;

        std::string cameraIntrinsicsPath;
        std::string imagesDir;
        std::string pointcloudsDir;

        double x_min, y_min, z_min, x_max, y_max, z_max;

        double ransacThreshold;
        int ransacMaxIterations;

        std::string output;
        bool visual = false;

        static CalibrationConfig fromYaml(const std::string& path);

    };

}


#endif