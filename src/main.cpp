#include <iostream>
#include <string>
#include <filesystem>

#include <opencv2/opencv.hpp>
#include "cam_lidar_calib/types.hpp"
#include <yaml-cpp/yaml.h>
#include "cam_lidar_calib/camera_detector.hpp"



int main()
{
	YAML::Node configNode = YAML::LoadFile("../config/params.yaml");

	cam_lidar_calib::CalibrationConfig  cfg = cam_lidar_calib::CalibrationConfig::fromYaml("../config/params.yaml");

	std::filesystem::path project_root = std::filesystem::current_path().parent_path();

	std::cout << "Project root resolved to: " << project_root << std::endl;


	std::cout << "Board size: rows " <<  cfg.rows << " cols: " << cfg.cols << std::endl;
	std::cout << "Square size: " << cfg.square << std::endl;
	std::cout << "Camera intrinsics path " << cfg.cameraIntrinsicsPath << std::endl;
	std::cout << "Images and pointcloud directories: " << cfg.imagesDir << ", " << "pointcloudDir: " << cfg.pointcloudsDir << std::endl;

	std::filesystem::path image_path = project_root / cfg.imagesDir;
	

	// std::cout << "Lidar ROI min: " << cfg.x_min << ", " << cfg.y_min  << ", " << cfg.z_min << std::endl;
	// std::cout << "Lidar ROi max: " << cfg.x_max << ", " << cfg.y_max <<", " << cfg.z_max << std::endl;
	// std::cout << "RANSAC threshold: " << cfg.ransacThreshold << "RANSAC Max Iterations: " << cfg.ransacMaxIterations << std::endl;
	std::cout << "Output file: " << cfg.output << std::endl;

	cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_COLOR);

	if(image.empty()) {
		std::cerr << "Failed to load image: " << image_path << std::endl;
		return 1;
	}

	std::cout << "Image loaded: " << image.cols << "x" << image.rows << std::endl;


	std::filesystem::path output_path = std::filesystem::path(cfg.output).parent_path();
	std::filesystem::create_directories(output_path);
	std::filesystem::path vis_path = output_path/ "chessboard_detected.png";
	cv::imwrite(vis_path.string(), image);
	std::cout << "Saved visualization to " << vis_path << std::endl;

	return 0;
}
