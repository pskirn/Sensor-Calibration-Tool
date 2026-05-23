#include "cam_lidar_calib/lidar_detector.hpp"

#include <iostream>
#include <vector>

#include <pcl-1.12/pcl/io/pcd_io.h>
#include <pcl-1.12/pcl/memory.h>
#include <pcl-1.12/pcl/PointIndices.h>
#include <pcl-1.12/pcl/filters/passthrough.h>
#include <pcl-1.12/pcl/segmentation/sac_segmentation.h>

namespace cam_lidar_calib
{
    LidarDetector::LidarDetector(const CalibrationConfig& config)
        : config_(config)
    {
    }

    std::optional<PlaneObservation> LidarDetector::detectFromFile(const std::string& filepath, int frame_index)
    {
        auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(filepath, *cloud) == -1) {
            std::cerr << "Could not read file: " << filepath << std::endl;
            return std::nullopt;
        }

        // ROI filtering using config bounds before plane detection
        auto cloud_x  = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        auto cloud_xy = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        auto cloud_filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

        pcl::PassThrough<pcl::PointXYZI> pass;

        pass.setInputCloud(cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(config_.x_min, config_.x_max);
        pass.filter(*cloud_x);

        pass.setInputCloud(cloud_x);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(config_.y_min, config_.y_max);
        pass.filter(*cloud_xy);

        pass.setInputCloud(cloud_xy);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(config_.z_min, config_.z_max);
        pass.filter(*cloud_filtered);

        return detect(cloud_filtered, frame_index);
    }

    std::optional<PlaneObservation> LidarDetector::detect(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered, int frame_index)
    {
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(config_.ransacThreshold);
        seg.setMaxIterations(config_.ransacMaxIterations);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty())
            return std::nullopt;

        if (static_cast<int>(inliers->indices.size()) < 20)
            return std::nullopt;

        Eigen::Vector3d normal(
            static_cast<double>(coefficients->values[0]),
            static_cast<double>(coefficients->values[1]),
            static_cast<double>(coefficients->values[2])
        );

        double distance = static_cast<double>(coefficients->values[3]);

        std::vector<Eigen::Vector3d> points;
        points.reserve(inliers->indices.size());
        for (const auto& idx : inliers->indices) {
            const auto& pt = (*cloud_filtered)[idx];
            points.emplace_back(
                static_cast<double>(pt.x),
                static_cast<double>(pt.y),
                static_cast<double>(pt.z)
            );
        }

        return PlaneObservation(normal, distance, SensorType::LIDAR, points, frame_index);
    }

}
