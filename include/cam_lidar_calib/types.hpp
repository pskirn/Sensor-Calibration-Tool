#pragma once

#include <iostream>
#include <stdlib.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <vector>
#include <string>
#include <optional>

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
         // Eigen::Vector3d ;
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
        Eigen::Vector3d  T;
        

        double error;
        int poses;

        std::vector<double> residuals;

        Eigen::Matrix4d getHomogeneous() const
        {
            Eigen::Matrix4d Transform = Eigen::Matrix4d::Identity();

            Transform.block<3,3>(0, 0) = R;
            Transform.block<3,1>(0, 3) = T;
            return Transform;
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
            Eigen::Vector3d T_inv = - R_inv * T;
            output.R = R_inv;
            output.T = T_inv;

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