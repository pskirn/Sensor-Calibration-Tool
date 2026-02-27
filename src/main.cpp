#include <iostream>
#include <string>
#include "cam_lidar_calib/types.hpp"
#include <yaml-cpp/yaml.h>



int main()
{
	YAML::Node configNode = YAML::LoadFile("../config/params.yaml");

	cam_lidar_calib::CalibrationConfig  cfg = cam_lidar_calib::CalibrationConfig::fromYaml("../config/params.yaml");


	std::cout << "Board size: rows " <<  cfg.rows << " cols: " << cfg.cols << std::endl;
	std::cout << "Square size: " << cfg.square << std::endl;
	std::cout << "Camera intrinsics path " << cfg.cameraIntrinsicsPath << std::endl;
	std::cout << "Images and pointcloud directories: " << cfg.imagesDir << ", " << "pointcloudDir: " << cfg.pointcloudsDir << std::endl;
	std::cout << "Lidar ROI min: " << cfg.x_min << ", " << cfg.y_min  << ", " << cfg.z_min << std::endl;
	std::cout << "Lidar ROi max: " << cfg.x_max << ", " << cfg.y_max <<", " << cfg.z_max << std::endl;
	std::cout << "RANSAC threshold: " << cfg.ransacThreshold << "RANSAC Max Iterations: " << cfg.ransacMaxIterations << std::endl;
	std::cout << "Output file and visulization success: " << cfg.output << cfg.visual << std::endl;
	return 0;
}
