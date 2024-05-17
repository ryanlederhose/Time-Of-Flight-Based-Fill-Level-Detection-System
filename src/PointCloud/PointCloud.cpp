/**
 * @file PointCloud.cpp
 * @brief Contains the implementation for the PointCloud class methods
*/

#include "PointCloud.h"
#include "../Facade/Facade.h"

pcl::PCDWriter PointCloud::pcdWriter;   ///< Static definition of pcdWriter

/**
 * @brief Default constructor for PointCloud class
*/
PointCloud::PointCloud() {
}

/**
 * @brief Convert a depth frame from the CS-20 ToF camera to a Point Cloud Library PointCloud object
 * @param frame Pointer to depth frame from CS-20 camera
 * @param pipelineStrct Pointer to ToF camera pipeline struct
 * @param pcd Pointer to output PointCloud from the Point Cloud Library
*/
void PointCloud::depthFrameToPointCloud(sy3::depth_frame *frame, sy3::pipeline *pipelineStruct, pcl::PointCloud<pcl::PointXYZ>::Ptr pcd) {
    sy3::sy3_error errors;
    int height = frame->get_height();
    int width = frame->get_width();

    // Compute the points from the depth frame
    sy3::points *depthFramePoints = pipelineStruct->get_process_engin(errors)->comptute_points(frame, false, errors);
    if (depthFramePoints != nullptr) {
        // Get the points in a float array
        float *pointsArr = depthFramePoints->get_points();
        if (pointsArr != nullptr) {
            size_t pointsLength = depthFramePoints->get_length();

            // Set up the PCL PointCloud
            pcd->clear();
            pcd->width = width;
            pcd->height = height;
            pcd->is_dense = false;
            pcd->points.resize(width * height);

            // Load up the points
            int index = 0;  
            while (index < pointsLength) {
                pcl::PointXYZ point;
                point.x = pointsArr[index];
                point.y = pointsArr[index + 1];
                point.z = pointsArr[index + 2];
                pcd->push_back(point);
                index = index + 3;
            }
        }
    }
}

/**
 * @brief Convert a depth frame from the CS-20 ToF camera to a Open3D PointCloud object
 * @param frame Pointer to depth frame from CS-20 camera
 * @param pipelineStrct Pointer to ToF camera pipeline struct
 * @param pcd Pointer to output PointCloud from Open3D
*/
void PointCloud::depthFrameToPointCloud(sy3::depth_frame *frame, sy3::pipeline *pipelineStruct, std::shared_ptr<open3d::geometry::PointCloud> pcd) {
    sy3::sy3_error errors;
    int height = frame->get_height();
    int width = frame->get_width();

    // Compute the points from the depth frame
    sy3::points *depthFramePoints = pipelineStruct->get_process_engin(errors)->comptute_points(frame, false, errors);
    if (depthFramePoints != nullptr) {
        float *pointsArr = depthFramePoints->get_points();
        if (pointsArr != nullptr) {
            size_t pointsLength = depthFramePoints->get_length();

            // Open3D uses Eigen::Vector3d for storing points, so we create a vector of these
            std::vector<Eigen::Vector3d> points;
            points.reserve(pointsLength / 3); // Reserve space for all points to avoid reallocations

            for (size_t index = 0; index < pointsLength; index += 3) {
                points.emplace_back(pointsArr[index], pointsArr[index + 1], pointsArr[index + 2]);
            }

            // Set the points to the point cloud
            pcd->points_ = points;
        }
    }
}

/**
 * @brief Write a PCL PointCloud object to file
 * @param pcd PointCloud object to write to file
 * @param filePath The file path of the pcd to write to (not including .pcd)
*/
void PointCloud::writePointCloudToFile(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd, std::string filePath) {
    pcdWriter.write(filePath + ".pcd", *pcd);
}

/**
 * @brief Convert a PCL PointCloud to a Open3D PointCloud
 * @param pcl_cloud PCL PointCloud to convert
 * @retval The converted Open3D point cloud object
*/
std::shared_ptr<open3d::geometry::PointCloud> PointCloud::pclToOpen3d(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud) {
    // Create an empty Open3D PointCloud
    auto o3d_cloud = std::make_shared<open3d::geometry::PointCloud>();

    // Reserve space for points
    o3d_cloud->points_.reserve(pcl_cloud->size());

    // Copy points from PCL to Open3D
    for (const auto& point : *pcl_cloud) {
        o3d_cloud->points_.push_back({point.x, point.y, point.z});
    }

    return o3d_cloud;
}

/**
 * @brief Write a Open3D PointCloud object to file
 * @param pcd PointCloud object to write to file
 * @param filePath The file path of the pcd to write to (not including .pcd)
*/
void PointCloud::writePointCloudToFile(open3d::geometry::PointCloud o3dCloud, std::string filePath) {
    open3d::io::WritePointCloud(filePath + ".pcd", o3dCloud);
}

/**
 * @brief Convert an Open3D PointCloud to a PCL PointCloud
 * @param o3d_cloud Open3D PointCloud to convert
 * @retval The converted PCL PointCloud object
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud::open3dToPcl(const std::shared_ptr<open3d::geometry::PointCloud>& o3d_cloud) {
    // Create an empty PCL PointCloud
    auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    // Reserve space for points
    pcl_cloud->reserve(o3d_cloud->points_.size());

    // Copy points from Open3D to PCL
    for (const auto& point : o3d_cloud->points_) {
        pcl_cloud->push_back(pcl::PointXYZ(static_cast<float>(point(0)), static_cast<float>(point(1)), static_cast<float>(point(2))));
    }

    return pcl_cloud;
}

/**
 * @brief Read a pcd file
 * @param filePath Path of pcd file (not including .pcd)
 * @retval Pointer to Open3D PointCloud object
*/
std::shared_ptr<open3d::geometry::PointCloud> PointCloud::readPointCloud(std::string filePath) {
    auto pcd = std::make_shared<open3d::geometry::PointCloud>();
    if (open3d::io::ReadPointCloud(filePath + ".pcd", *pcd)) {
        return pcd;
    } else {
        // Handle the error case where the point cloud could not be read
        return nullptr;
    }
}