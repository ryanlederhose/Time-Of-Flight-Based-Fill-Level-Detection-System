/**
 * @file Volume.cpp
 * @brief Contains the Volume class implementation, which includes methods for point cloud processing.
 */

#include "Volume.h"
#include "../Facade/Facade.h"

/**
 * @brief Default constructor for Volume.
 * @param facade Pointer to the Facade class object which acts as an interface to the subsystem.
 */
Volume::Volume(Facade *facade): facade(facade) {
}


/**
 * @brief Filters a point cloud by removing points that are closer than a specified distance from a background point cloud.
 * @param o3dCloud Shared pointer to the original point cloud to filter.
 * @param o3dCloudBackground Shared pointer to the background point cloud used as a reference.
 * @param minimumDistance Minimum distance threshold for filtering the points.
 * @return Filtered point cloud with points further than the minimum distance.
 */
std::shared_ptr<open3d::geometry::PointCloud> Volume::pointCloudDistanceFilter(std::shared_ptr<open3d::geometry::PointCloud> o3dCloud, const std::shared_ptr<open3d::geometry::PointCloud> o3dCloudBackground, int minimumDistance) {

    // Check for nullptr
    if ((o3dCloud == nullptr) || (o3dCloudBackground == nullptr)) {
        return std::shared_ptr<open3d::geometry::PointCloud>();
    }

    // Compute point cloud distances
    std::vector<double> dists = o3dCloud->ComputePointCloudDistance(*o3dCloudBackground);

    // Indices for points that are farther than minimumDistance
    std::vector<size_t> ind;
    for (size_t i = 0; i < dists.size(); ++i) {
        if (dists[i] > minimumDistance) {
            ind.push_back(i);
        }
    }

    // Select points by index (those that are farther than the threshold)
    auto filteredPcd = o3dCloud->SelectByIndex(ind, false);
    return filteredPcd;
}

/**
 * @brief Downsample a point cloud usng the Open3D library
 * @param o3dCloud Point cloud to downsample
 * @param voxelSize The voxel size to use to downsample
 * @retval Downsampled point cloud
*/
std::shared_ptr<open3d::geometry::PointCloud> Volume::downsamplePointCloud(std::shared_ptr<open3d::geometry::PointCloud> o3dCloud, double voxelSize) {
    // Check for nullptr
    if (o3dCloud == nullptr) {
        return std::shared_ptr<open3d::geometry::PointCloud>();
    }
    
    auto downsampledCloud = o3dCloud->VoxelDownSample(voxelSize);
    return downsampledCloud;
}

/**
 * @brief Filters out statistical outliers from a point cloud based on the number of neighbors and standard deviation multiplier.
 * @param o3dCloud Point cloud to filter.
 * @param numberNeighbours Number of neighbors to consider for determining if a point is an outlier.
 * @param stdMultiplier Standard deviation multiplier; points with a distance larger than this multiplier of the standard deviation are considered outliers.
 * @param getOutliers If true, returns the outliers instead of the filtered cloud.
 * @return Filtered or outlier point cloud based on the getOutliers flag.
 */
std::shared_ptr<open3d::geometry::PointCloud> Volume::statisticalOutlierFilter(std::shared_ptr<open3d::geometry::PointCloud> o3dCloud, int numberNeighbours, double stdMultiplier, bool getOutliers) {
    // Check for nullptr
    if (o3dCloud == nullptr) {
        return std::shared_ptr<open3d::geometry::PointCloud>();
    }

    auto indicesTuple = o3dCloud->RemoveStatisticalOutliers(numberNeighbours, stdMultiplier);
    auto filteredPcd = o3dCloud->SelectByIndex(std::get<1>(indicesTuple), getOutliers);
    return filteredPcd;
}

/**
 * @brief Performs DBSCAN clustering to segment a point cloud into groups based on spatial proximity.
 * @param o3dCloud Point cloud to cluster.
 * @param densityParameter The maximum distance between two points for one to be considered as in the neighborhood of the other.
 * @param minimumPoints The minimum number of points required to form a cluster.
 * @return Vector of labels where each element represents the cluster label of the corresponding point in the point cloud.
 */
std::vector<int> Volume::dbscanPointCloud(std::shared_ptr<open3d::geometry::PointCloud> o3dCloud, int densityParameter, int minimumPoints) {
    // Check for nullptr
    if (o3dCloud == nullptr) {
        return {0};
    }
    
    auto labels = o3dCloud->ClusterDBSCAN(densityParameter, minimumPoints);
    return labels;
}

/**
 * @brief Calculates the volume of the convex hull formed by the points in the point cloud.
 * @param o3dCloud Point cloud to process.
 * @return Volume of the convex hull.
 */
double Volume::getConvexHullVolume(std::shared_ptr<open3d::geometry::PointCloud> o3dCloud) {
    if (o3dCloud != nullptr) {
        try {
            // Compute the convex hull
            auto hull = o3dCloud->ComputeConvexHull();
            auto hullMesh = std::get<0>(hull);
            if (hullMesh != nullptr) {
                // Create LineSet from TriangleMesh
                auto hull_ls = open3d::geometry::LineSet::CreateFromTriangleMesh(*hullMesh);
                if (hull_ls != nullptr) {
                    hull_ls->PaintUniformColor({1, 0, 0});
                }

                // Get the volume of the convex hull
                auto volume = hullMesh->GetVolume();
                return volume;
            } else {
                std::cerr << "Error: Convex hull mesh is null." << std::endl;
                return 0;
            }
        } catch (const std::exception& e) {
            std::cerr << "Exception caught: " << e.what() << std::endl;
            return 0;
        }
    } else {
        std::cerr << "Error: Point cloud is null." << std::endl;
        return 0;
    }
}

/**
 * @brief Computes the background depth from a point cloud file by processing it through various filtering stages.
 * @param backgroundFilePath Path to the point cloud file used as the background.
 * @return Depth of the background.
 */
double Volume::getBackgroundDepth(std::string backgroundFilePath) {
    auto backgroundPcd = PointCloud::readPointCloud(backgroundFilePath);
    backgroundPcd = Volume::downsamplePointCloud(backgroundPcd, 5);
    backgroundPcd = this->statisticalOutlierFilter(backgroundPcd, 32, 0.5, false);
    backgroundPcd = this->segmentPlane(backgroundPcd, 10, 20, 100, false);
    this->backgroundDepth = this->getMeanZ(backgroundPcd);
    return this->getMeanZ(backgroundPcd);
}

/**
 * @brief Computes the background depth from a point cloud file by processing it through various filtering stages.
 * @param backgroundPcd Point cloud file used as the background.
 * @return Depth of the background.
 */
double Volume::getBackgroundDepth(std::shared_ptr<open3d::geometry::PointCloud> backgroundPcd) {
    auto downsampledCloud = this->downsamplePointCloud(backgroundPcd, 5);
    downsampledCloud = this->statisticalOutlierFilter(downsampledCloud, 32, 0.5, false);
    downsampledCloud = this->segmentPlane(downsampledCloud, 10, 20, 100, false);
    return this->getMeanZ(downsampledCloud);
}

/**
 * @brief Segments a plane from the point cloud using RANSAC and returns either the points on the plane or the remaining points.
 * @param o3dCloud Point cloud to segment.
 * @param distanceThreshold Maximum distance a point may have from the model to be considered as an inlier.
 * @param ransacNum Minimum number of inliers a candidate plane must have to be considered as the model.
 * @param numIterations Number of iterations the RANSAC algorithm will run.
 * @param getOutliers If true, returns the outliers (points not fitting the plane); otherwise, returns the inliers.
 * @return Point cloud of the inliers or outliers based on the getOutliers flag.
 */
std::shared_ptr<open3d::geometry::PointCloud> Volume::segmentPlane(std::shared_ptr<open3d::geometry::PointCloud> o3dCloud, double distanceThreshold, int ransacNum, int numIterations, bool getOutliers) {
    // Check for nullptr
    if (o3dCloud == nullptr) {
        return std::shared_ptr<open3d::geometry::PointCloud>();
    }
    
    auto segmentation = o3dCloud->SegmentPlane(distanceThreshold, ransacNum, numIterations);
    auto inliers = o3dCloud->SelectByIndex(std::get<1>(segmentation), false);
    auto outliers = o3dCloud->SelectByIndex(std::get<1>(segmentation), true);
    inliers->PaintUniformColor({0, 1, 0});
    outliers->PaintUniformColor({1, 0, 0});
    if (getOutliers) {
        return outliers;
    } else {
        return inliers;
    }
}

/**
 * @brief Segments a plane from the point cloud using RANSAC and returns either the points on the plane or the remaining points.
 * @param o3dCloud Point cloud to segment.
 * @param distanceThreshold Maximum distance a point may have from the model to be considered as an inlier.
 * @param ransacNum Minimum number of inliers a candidate plane must have to be considered as the model.
 * @param numIterations Number of iterations the RANSAC algorithm will run.
 * @param getOutliers If true, returns the outliers (points not fitting the plane); otherwise, returns the inliers.
 * @return Point cloud of the inliers or outliers based on the getOutliers flag.
 */
std::vector<std::shared_ptr<open3d::geometry::PointCloud>> Volume::multiOrderRansac(std::shared_ptr<open3d::geometry::PointCloud> o3dCloud, double distanceThreshold, int ransacNum, int numIterations, bool getOutliers) {
    // Check for nullptr
    if (o3dCloud == nullptr) {
        return std::vector<std::shared_ptr<open3d::geometry::PointCloud>>();
    }
    
    size_t pcdSize = o3dCloud->points_.size();
    size_t updatedSize = pcdSize;
    auto copyCloud = *o3dCloud;
    std::vector<std::shared_ptr<open3d::geometry::PointCloud>> inliersVector;

    while (updatedSize > (pcdSize * 0.3)) {
        auto segmentation = copyCloud.SegmentPlane(distanceThreshold, ransacNum, numIterations);
        auto inliers = copyCloud.SelectByIndex(std::get<1>(segmentation), false);
        auto outliers = copyCloud.SelectByIndex(std::get<1>(segmentation), true);
        inliers->PaintUniformColor({0, 1, 0});
        outliers->PaintUniformColor({1, 0, 0});
        open3d::visualization::DrawGeometries({inliers, outliers});
        inliersVector.push_back(inliers);
        updatedSize = outliers->points_.size();
        copyCloud = *outliers;
    }
    return inliersVector;
}

/**
 * @brief Computes the mean Z value (depth) of the points in a point cloud.
 * @param o3dCloud Point cloud to analyze.
 * @return Mean Z value of the point cloud.
 */
double Volume::getMeanZ(std::shared_ptr<open3d::geometry::PointCloud> o3dCloud) {
    // Check for nullptr
    if (o3dCloud == nullptr) {
        return 0;
    }
    
    double sumZ = 0.0;
    for (const auto &p: o3dCloud->points_) {
        sumZ += p.z();
    }
    double meanZ = sumZ / o3dCloud->points_.size();
    return meanZ;
}

/**
 * @brief Performs Euclidean cluster extraction to segment large point clouds into clusters based on spatial proximity.
 * @param pcd Point cloud to segment into clusters.
 * @param clusterTolerance The spatial distance tolerance between points for them to be considered in the same cluster.
 * @param minClusterSize The minimum number of points that a cluster needs to contain.
 * @param maxClusterSize The maximum number of points that a cluster can contain.
 * @return The largest cluster extracted from the point cloud.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr Volume::eucledianClusterExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd, double clusterTolerance, int minClusterSize, int maxClusterSize) {
    // Creating KdTree object for the search method of extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(pcd);

    std::vector<pcl::PointIndices> clusterIndices;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusterPointClouds;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> eucledianCluster;
    eucledianCluster.setClusterTolerance(clusterTolerance);
    eucledianCluster.setMinClusterSize(minClusterSize);
    eucledianCluster.setMaxClusterSize(maxClusterSize);
    eucledianCluster.setSearchMethod(tree);
    eucledianCluster.setInputCloud(pcd);
    eucledianCluster.extract(clusterIndices);

    if (clusterIndices.size() == 0) {
        std::cout << "Eucledian Cluster Extraction Failed. " << clusterIndices.size() << " data points." << std::endl;
    }

    int j = 0;
    for (const auto &cluster: clusterIndices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &idx: cluster.indices) {
            cloudCluster->push_back((*pcd)[idx]);
        }
        cloudCluster->width = cloudCluster->size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        clusterPointClouds.push_back(cloudCluster);
    }

    int pcdSize = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &cluster: clusterPointClouds) {
        if (cluster->size() > pcdSize) {
            pcdSize = cluster->size();
            cloudCluster = cluster;
        }
    }

    return cloudCluster;
}

/**
 * @brief Splits a point cloud into smaller chunks based on specified chunk sizes along the x and y axes.
 * @param o3dCloud The point cloud to split.
 * @param chunk_size_x The size of each chunk along the x-axis.
 * @param chunk_size_y The size of each chunk along the y-axis.
 * @return std::vector<std::shared_ptr<open3d::geometry::PointCloud>> A vector of point cloud chunks.
 */
std::vector<std::shared_ptr<open3d::geometry::PointCloud>> Volume::splitPointCloud(std::shared_ptr<open3d::geometry::PointCloud> o3dCloud,
                     double chunk_size_x, double chunk_size_y) {
    // Check for nullptr
    if (o3dCloud == nullptr) {
        return std::vector<std::shared_ptr<open3d::geometry::PointCloud>>();
    }
    
    // Determine the bounds of the point cloud
    Eigen::Vector3d min_bound = o3dCloud->GetMinBound();
    Eigen::Vector3d max_bound = o3dCloud->GetMaxBound();

    // Calculate number of chunks in each dimension
    int chunks_x = static_cast<int>(std::ceil((max_bound.x() - min_bound.x()) / chunk_size_x));
    int chunks_y = static_cast<int>(std::ceil((max_bound.y() - min_bound.y()) / chunk_size_y));

    // Create a vector to hold chunks
    std::vector<std::shared_ptr<open3d::geometry::PointCloud>> chunks(chunks_x * chunks_y);

    // Initialize each chunk
    for (int i = 0; i < chunks.size(); ++i) {
        chunks[i] = std::make_shared<open3d::geometry::PointCloud>();
    }

    // Assign points to chunks
    for (const auto& point : o3dCloud->points_) {
        int idx_x = std::min(static_cast<int>((point.x() - min_bound.x()) / chunk_size_x), chunks_x - 1);
        int idx_y = std::min(static_cast<int>((point.y() - min_bound.y()) / chunk_size_y), chunks_y - 1);

        int index = idx_y * chunks_x + idx_x;
        chunks[index]->points_.push_back(point);
    }

    return chunks;
}

/**
 * @brief Calculates the volume of a stockpile by comparing it to a reference background point cloud.
 * @param o3dCloud Point cloud representing the stockpile.
 * @param backgroundPcd Background point cloud for reference.
 * @param showGeometries Flag to determine whether to visually display the point cloud processing stages.
 * @return Estimated volume of the stockpile.
 */
double Volume::calculateStockPileVolume(std::shared_ptr<open3d::geometry::PointCloud> o3dCloud, std::shared_ptr<open3d::geometry::PointCloud> backgroundPcd, bool showGeometries) {
    // Check for nullptr
    if (o3dCloud == nullptr) {
        return 0;
    }

    // Downsample point cloud
    double voxelSize = 5;
    o3dCloud = this->downsamplePointCloud(o3dCloud, voxelSize);
    if (showGeometries) {
        this->facade->debugPrintLn("1) Showing downsampled point cloud.");
        open3d::visualization::DrawGeometries({o3dCloud});
    }

    // Statistical outlier removal
    int numberNeighbours = 16;
    double stdMultiplier = 0.5;
    bool getOutliers = false;
    o3dCloud = this->statisticalOutlierFilter(o3dCloud, numberNeighbours, stdMultiplier, getOutliers);
    if (showGeometries) {
        this->facade->debugPrintLn("2) Statistical outlier filter");
        open3d::visualization::DrawGeometries({o3dCloud});
    }

    // Remove differences with background pcd (distance filter)
    o3dCloud = this->pointCloudDistanceFilter(o3dCloud, backgroundPcd, 20);
    if (showGeometries) {
        this->facade->debugPrintLn("3) Distance filter");
        open3d::visualization::DrawGeometries({o3dCloud});
    }

    // Statistical outlier removal on towel stack
    numberNeighbours = 16;
    stdMultiplier = 0.5;
    getOutliers = false;
    o3dCloud = this->statisticalOutlierFilter(o3dCloud, numberNeighbours, stdMultiplier, getOutliers); 
    if (showGeometries) {
        this->facade->debugPrintLn("4) Statistical outlier filter");
        open3d::visualization::DrawGeometries({o3dCloud});
    }

    // Use DBSCAN to segment towel stacks from top planes
    double densityParameters = 100;
    int minimumPoints = 1000;
    auto dbscanLabels = this->dbscanPointCloud(o3dCloud, densityParameters, minimumPoints);

    // Iterate through the clusters, calculating their volumes
    double totalVolume = 0.0;
    int maxLabel = *std::max_element(dbscanLabels.begin(), dbscanLabels.end());
    for (int i = 0; i <= maxLabel; ++i) {
        std::vector<size_t> indices;
        for (size_t j = 0; j < dbscanLabels.size(); ++j) {
            if (dbscanLabels[j] == i) {
                indices.push_back(j);
            }
        }
        open3d::geometry::PointCloud clusterPcd;
        for (auto idx: indices) {
            clusterPcd.points_.push_back(o3dCloud->points_[idx]);
        }

        if (showGeometries) {
            this->facade->debugPrintLn("5) DBSCAN Cluster");
            open3d::visualization::DrawGeometries({std::make_shared<open3d::geometry::PointCloud>(clusterPcd)});
        }

#ifndef POINT_CLOUD_SLICING
        // Create a new vector to hold the modified points
        std::vector<Eigen::Vector3d> newPoints;

        // Iterate through each point in the original point cloud
        for (const auto& point : clusterPcd.points_) {
            // Replace the z-coordinate with backgroundDepth
            newPoints.push_back({point(0), point(1), this->backgroundDepth});
        }

        // Append new points to the existing points in the point cloud
        clusterPcd.points_.insert(clusterPcd.points_.end(), newPoints.begin(), newPoints.end());

        // Calculate volume of chunk
        double clusterVol = this->getConvexHullVolume(std::make_shared<open3d::geometry::PointCloud>(clusterPcd));
        totalVolume += clusterVol;
#else
        // Use slicing method to calculate volume of cluster
        double chunkSizeX = 100;
        double chunkSizeY = 100;
        std::vector<std::shared_ptr<open3d::geometry::PointCloud>> chunksVector(chunkSizeX * chunkSizeY);
        chunksVector = this->splitPointCloud(std::make_shared<open3d::geometry::PointCloud>(clusterPcd), chunkSizeX, chunkSizeY);
        for (auto& chunk : chunksVector) {
            if (chunk->points_.size() > 0) {
                // Create a new vector to hold the modified points
                std::vector<Eigen::Vector3d> newPoints;

                // Iterate through each point in the original point cloud
                for (const auto& point : chunk->points_) {
                    // Replace the z-coordinate with backgroundDepth
                    newPoints.push_back({point(0), point(1), this->backgroundDepth});
                }

                // Append new points to the existing points in the point cloud
                chunk->points_.insert(chunk->points_.end(), newPoints.begin(), newPoints.end());

                // Calculate volume of chunk
                double clusterVol = this->getConvexHullVolume(chunk);
                totalVolume += clusterVol;
            }
        }
#endif
    }

    return totalVolume;
}