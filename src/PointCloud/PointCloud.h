/**
 * @file PointCloud.h
 * @brief Contains the definition for the PointCloud class methods.
*/

#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <thread>
#include <string>
#include "libsynexens3/libsynexens3.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include "open3d/Open3D.h"

// Forward declaration of Facade class
class Facade;

/**
 * @class PointCloud
 * @brief Provides functionalities for managing PointCloud methods.
*/
class PointCloud {

    public:
        PointCloud(void);

        static void depthFrameToPointCloud(sy3::depth_frame *frame, sy3::pipeline *pipelineStruct, pcl::PointCloud<pcl::PointXYZ>::Ptr pcd);
        static void depthFrameToPointCloud(sy3::depth_frame *frame, sy3::pipeline *pipelineStruct, std::shared_ptr<open3d::geometry::PointCloud> pcd);

        static void writePointCloudToFile(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd, std::string filePath);
        static void writePointCloudToFile(open3d::geometry::PointCloud o3dCloud, std::string filePath);

        static std::shared_ptr<open3d::geometry::PointCloud> pclToOpen3d(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud);
        static pcl::PointCloud<pcl::PointXYZ>::Ptr open3dToPcl(const std::shared_ptr<open3d::geometry::PointCloud> &o3d_cloud);

        static std::shared_ptr<open3d::geometry::PointCloud> readPointCloud(std::string filePath);
    private:
        static pcl::PCDWriter pcdWriter;
};