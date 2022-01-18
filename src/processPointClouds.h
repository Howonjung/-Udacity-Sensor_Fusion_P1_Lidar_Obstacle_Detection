// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

// Newly added due to the error of "‘filesystem’ is not a member of ‘boost’GCC"
#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <unordered_set>
// for clustering
#include "quiz/cluster/kdtree.h"
// for random smapling
#include <iostream>
#include <random>

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    // Input cloud filtering
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> FilterCloud(typename pcl::PointCloud<PointT>::Ptr inputCloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    // SegmentPlane
    std::unordered_set<int> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold, bool pclLibrary);
    
    // Clustering
    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize, bool pclLibrary);
    void clusterHelper(typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& pointsCheck , int pointIdx, KdTree* tree, float distanceTol, unsigned char pointDim);
    std::vector<std::vector<int>> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, unsigned char pointDim);
    

    // Genral 
    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);
    int numPoints1(typename pcl::PointCloud<PointT>::Ptr cloud);
    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);
    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);
    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);
    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */
