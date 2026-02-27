#include <yaml-cpp/yaml.h>
#include "cam_lidar_calib/types.hpp"

namespace cam_lidar_calib {

    CalibrationConfig CalibrationConfig::fromYaml(const std::string& path)
    {

        CalibrationConfig cfg;

        YAML::Node root = YAML::LoadFile(path);

        
        // CAMERA
        std::cout << "camera section " << std::endl;
        YAML::Node cam = root["camera"];

        cfg.cameraIntrinsicsPath = cam["intrinsics_file"].as<std::string>();
        cfg.rows = cam["board_rows"].as<int>();
        cfg.cols = cam["board_cols"].as<int>();
        cfg.square = cam["square_size"].as<double>();
        

        // LIDAR
        std::cout << "lidar section " << std::endl;
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
        std::cout << "data section " << std::endl;
        YAML::Node data = root["data"];

        cfg.imagesDir = data["images_dir"].as<std::string>();
        cfg.pointcloudsDir = data["pointclouds_dir"].as<std::string>();


        // OUTPUT
        YAML::Node output = root["output"];

        cfg.output = output["result_file"].as<std::string>();
        
        if (output["generate_visualization"])
            cfg.visual = output["generate_visualization"].as<bool>();

        
        return cfg;

    };
}