#include "cam_lidar_calib/lidar_detector.hpp"

#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

#include <pcl-1.12/pcl/io/pcd_io.h>
#include <pcl-1.12/pcl/memory.h>
#include <pcl-1.12/pcl/PointIndices.h>
#include <pcl-1.12/pcl/filters/passthrough.h>
#include <pcl-1.12/pcl/sample_consensus/ransac.h>
#include <pcl-1.12/pcl/sample_consensus/sac_model_plane.h>
#include <pcl-1.12/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.12/pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;


namespace cam_lidar_calib
{
    LidarDetector::LidarDetector(const CalibrationConfig& config)
    {
        config_ = config;
    
        auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        auto cloud_x = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        auto cloud_xy = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        auto cloud_filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        auto final = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

        auto viewer = std::make_shared<pcl::visualization::PCLVisualizer>("Viewer");

        if (pcl::io::loadPCDFile<pcl::PointXYZI> ("data.pcd", *cloud) == -1)
        {
            PCL_ERROR ("Couldn't read file data.pcd \n" );
            throw std::runtime_error("Couldn't read file");
        }

        std::cout << "Loaded "
                    << cloud->width * cloud->height
                    << "data points from \"filename\" with the following fields: "
                    << std::endl;

        for (const auto& point: *cloud)
        std::cout << "   " << point.x
                    << " " << point.y
                    << " " << point.z 
                    << " " << point.intensity
                    << std::endl;
        
        
        // Create a filtering object
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud (cloud);

        // PassThrough X
        pass.setFilterFieldName ("x");
        pass.setFilterLimits(0.0, 1.0);
        pass.filter (*cloud_x);

        // PassThrough Y
        pass.setFilterFieldName ("y");
        pass.setFilterLimits(0.0, 1.0);
        pass.filter (*cloud_xy);

        // PassThrough Z
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.0, 1.0);


        // pass.setNegative (true);
        pass.filter(*cloud_filtered);

        //filtered point cloud
        std::cout << " Filtered Point cloud " 
                    << cloud_filtered->width * cloud_filtered->height
                    << "data points from original cloud with following fields"     
                    << std::endl;

        for (const auto& point: *cloud_filtered) {
            std::cout << "    " << point.x
                        << " " << point.y
                        << " " << point.z
                        << " " << point.intensity
                        << std::endl;
        }
        
        std::vector<int> inliers;

        // RANSAC object and compute the appropriated model
        pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZI> (cloud_filtered));
        pcl::RandomSampleConsensus<pcl::PointXYZI> ransac (model_p);
        ransac.setDistanceThreshold (0.01);
        ransac.computeModel();
        ransac.getInliers(inliers);

        // copies all inliers of the model computed to another PointCloud
        pcl::copyPointCloud (*cloud_filtered, inliers, *final);

        // create the visulaization object and adds for both of our original cloud and all inliers cloud
        viewer->addPointCloud<pcl::PointXYZI>(cloud, "cloud");
        viewer->addPointCloud<pcl::PointXYZI>(final, "final");

        while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
            std::this_thread::sleep_for(100ms);
        }
    }

    std::optional<cam_lidar_calib::PlaneObservation> extractPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered, int frame_index, const CalibrationConfig& config)
    {
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(config.ransacThreshold);
        seg.setMaxIterations(config.ransacMaxIterations);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        seg.setInputCloud(cloud_filtered);

        seg.segment(*inliers, *coefficients);

        // Check if plane is found
        if (inliers->indices.empty())
            return std::nullopt;

        if (inliers->indices.size() < 20)
            return std::nullopt;

        Eigen::Vector3d normal(
            static_cast<double>(coefficients->values[0]),
            static_cast<double>(coefficients->values[1]),
            static_cast<double>(coefficients->values[2])
        );

        double distance = static_cast<double>(coefficients->values[3]);

        // Convert inliers to Eigen points
        std::vector<Eigen::Vector3d> points;
        points.reserve(inliers->indices.size());

        for (const auto& idx : inliers->indices)
        {
            const auto& pt = (*cloud_filtered)[idx];
            points.emplace_back(
                static_cast<double>(pt.x),
                static_cast<double>(pt.y),
                static_cast<double>(pt.z)
            );
        }

        // Build result
        cam_lidar_calib::PlaneObservation lidar_obs(
            normal,
            distance,
            SensorType::LIDAR,
            points,
            frame_index
        );
       
        return lidar_obs;


    }


} 



