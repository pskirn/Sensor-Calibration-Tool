#include <yaml-cpp/yaml.h>
#include "cam_lidar_calib/types.hpp"

namespace cam_lidar_calib {

    CameraIntrinsics CameraIntrinsics::fromYaml(const std::string& path)
        {
            CameraIntrinsics intrinsics;
            YAML::Node root = YAML::LoadFile(path);

            intrinsics.width = root["image_width"].as<int>();
            intrinsics.height = root["image_height"].as<int();

            auto K_data = root["camera_matrix"]["data"].as<std::vector<double>>();
            intrinsics.K = Eigen::Matrix3d::Identity();
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    intrinsics.K(i, j) = K_data[i * 3 + j];


            auto dist_data = root["distortion_coefficients"]["data"].as<std::vector<double>>();
            intrinsics.distortionCoeffs = Eigen::VectorXd(dist_data.size());

            for (size_t i = 0; i < dist_data.size(); ++i)
                intrinsics.distortionCoeffs(i) = dist_data[i];

            
            return intrinsics;
        }

    CalibrationConfig CalibrationConfig::fromYaml(const std::string& path)
    {

        // LIDAR
        // std::cout << "lidar section " << std::endl;
        YAML::Node lid = root["lidar"];

        cfg.x_min = lid["roi_min"]["x"].as<double>();
        cfg.y_min = lid["roi_min"]["y"].as<double>();
        cfg.z_min = lid["roi_min"]["z"].as<double>();

        cfg.x_max = lid["roi_max"]["x"].as<double>();
        cfg.y_max = lid["roi_max"]["y"].as<double>();
        cfg.z_max = lid["roi_max"]["z"].as<double>();

        cfg.ransacThreshold = lid["ransac_threshold"].as<double>();
        cfg.ransacMaxIterations = lid["ransac_max_iterations"].as<int>();


        // DATA
        // std::cout << "data section " << std::endl;
        YAML::Node data = root["data"];

        // cfg.imagesDir = data["images_dir"].as<std::string>();
        cfg.pointcloudsDir = data["pointclouds_dir"].as<std::string>();

        


        // OUTPUT
        YAML::Node output = root["output"];

        cfg.output = output["result_file"].as<std::string>();
        
        if (output["generate_visualization"])
            cfg.visual = output["generate_visualization"].as<bool>();

        
        return cfg;

    };
}