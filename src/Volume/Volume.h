/**
 * @file Volume.h
 * @brief Header file for the Volume class, which provides methods for advanced point cloud processing, including filtering, segmentation, and volume estimation.
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
#include "../PointCloud/PointCloud.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iomanip>

// #define POINT_CLOUD_SLICING     ///< Control whether the point cloud is sliced

// Forward declaration of Facade class
class Facade;

/**
 * @class Volume
 * @brief Provides functionalities for managing and processing 3D point cloud data using tools from PCL (Point Cloud Library) and Open3D.
 */
class Volume {

    public:
        Volume(Facade *facade);
        std::shared_ptr<open3d::geometry::PointCloud> downsamplePointCloud(std::shared_ptr<open3d::geometry::PointCloud> o3dCloud, double voxelSize);
        std::shared_ptr<open3d::geometry::PointCloud> statisticalOutlierFilter(std::shared_ptr<open3d::geometry::PointCloud> o3dCloud, int numberNeighbours, double stdMultiplier, bool getOutliers);
        std::vector<int> dbscanPointCloud(std::shared_ptr<open3d::geometry::PointCloud> o3dCloud, int densityParameter, int minimumPoints);
        std::shared_ptr<open3d::geometry::PointCloud> pointCloudDistanceFilter(std::shared_ptr<open3d::geometry::PointCloud> o3dCloud, const std::shared_ptr<open3d::geometry::PointCloud> o3dCloudBackground, int minimumDistance);
        double getConvexHullVolume(std::shared_ptr<open3d::geometry::PointCloud> o3dCloud);
        double getBackgroundDepth(std::string backgroundFilePath);
        double getBackgroundDepth(std::shared_ptr<open3d::geometry::PointCloud> backgroundPcd);
        std::shared_ptr<open3d::geometry::PointCloud> segmentPlane(std::shared_ptr<open3d::geometry::PointCloud> o3dCloud, double distanceThreshold, int ransacNum, int numIterations, bool getOutliers);
        double getMeanZ(std::shared_ptr<open3d::geometry::PointCloud> o3dCloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr eucledianClusterExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd, double clusterTolerance, int minClusterSize, int maxClusterSize);
        std::vector<std::shared_ptr<open3d::geometry::PointCloud>> multiOrderRansac(std::shared_ptr<open3d::geometry::PointCloud> o3dCloud, double distanceThreshold, int ransacNum, int numIterations, bool getOutliers);
        std::vector<std::shared_ptr<open3d::geometry::PointCloud>> splitPointCloud(std::shared_ptr<open3d::geometry::PointCloud> o3dCloud, double chunk_size_x, double chunk_size_y);
        double calculateStockPileVolume(std::shared_ptr<open3d::geometry::PointCloud> o3dCloud, std::shared_ptr<open3d::geometry::PointCloud> backgroundPcd, bool showGeometries);

    private:
        Facade *facade;     ///< Pointer to Facade class object
        double backgroundDepth = 0;

};