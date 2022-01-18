// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr inputCloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    std::cout << "=====Input cloud filtering process=====" << "\n";
    std::cout << "inputCloud size: " << inputCloud->size() << "\n";

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // VoxelGrid filtering
    // Create the filtering object
    typename pcl::PointCloud<PointT>::Ptr voxelGrid_filtered (new pcl::PointCloud<PointT>());
    typename pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (inputCloud);
    // Define voxel size, 0.5 -> 50cm
    vg.setLeafSize (filterRes, filterRes, filterRes);
    vg.filter (*voxelGrid_filtered);
    std::cout << "voxelGrid_filtered size: " << voxelGrid_filtered->size()<< "\n";

    // Cropbox filtering
    // ref: https://stackoverflow.com/questions/45790828/remove-points-outside-defined-3d-box-inside-pcl-visualizer

    typename pcl::PointCloud<PointT>::Ptr boxROI_filtered (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr roofROI_filtered (new pcl::PointCloud<PointT>());
    typename pcl::CropBox<PointT> boxFilter;
    boxFilter.setMin(minPoint);
    boxFilter.setMax(maxPoint);
    boxFilter.setInputCloud(voxelGrid_filtered);
    boxFilter.filter(*boxROI_filtered);
    std::cout << "boxROI_filtered size: " << boxROI_filtered->size()<< "\n";

    // Crop points on roof
    std::vector<int> roof_indices; 
    typename pcl::CropBox<PointT> roofFilter;
    roofFilter.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roofFilter.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
    roofFilter.setInputCloud(boxROI_filtered);
    roofFilter.filter(roof_indices);
    roofFilter.filter(*roofROI_filtered);
    
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    for (int point: roof_indices)
        inliers->indices.push_back(point);
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (boxROI_filtered);
    extract.setIndices (inliers);
    // remove points in inliers
    extract.setNegative (true);
    extract.filter(*boxROI_filtered);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> filterResult(boxROI_filtered,roofROI_filtered);
    return filterResult;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    // Create the filtering object

    typename pcl::ExtractIndices<PointT> extract;
    typename pcl::PointCloud<PointT>::Ptr planeCloud  (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    
    // Extract the inliers1
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*planeCloud);

    // Extract the inliers2    
    // for(int index : inliers->indices){
    //     planeCloud->points.push_back(cloud->points[index]);
    // }
    
    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*obstCloud);
    std::cout <<"obstCloud size :" << obstCloud->size() << "\n";
    std::cout <<"planeCloud size :" << planeCloud->size() << "\n";

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud,planeCloud);
    return segResult;
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{	
	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	std::unordered_set<int> inliersTempResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	int max_idx = (*cloud).size();
	int firstRanIdx = -1;
	int secondRanIdx = -1;
	int thirdRanIdx = -1;
	float dist = -1;
	int numinliers = 0;
	// variable for plane formula
	float a,b,c,d ;
	// variable for x,y,z point
	PointT firstRanPoint, secondRanPoint, thirdRanPoint, Point; 
	for (int i = 0; i<maxIterations; i++){
        inliersTempResult.clear();

		// Randomly sample subset and fit line
		// Generate random_device to get a seed.
		std::random_device rd;

		// Init random number generator through random_device.
		std::mt19937 gen(rd());

		// Define uniform distribution to pick random number in btw 0 to max_idx
		std::uniform_int_distribution<int> dis(0, max_idx);
		
		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		while(inliersTempResult.size() < 3){
			inliersTempResult.insert(dis(gen));
		}
		auto iter = inliersTempResult.begin();
		firstRanIdx =  *iter;
		firstRanPoint = cloud->points[firstRanIdx];
		inliersTempResult.insert(firstRanIdx);

		iter++;
		secondRanIdx = *iter;
		secondRanPoint = cloud->points[secondRanIdx];
		inliersTempResult.insert(secondRanIdx);

		iter++;
		thirdRanIdx = *iter;
		thirdRanPoint = cloud->points[thirdRanIdx];
		inliersTempResult.insert(thirdRanIdx);

		a = (secondRanPoint.y - firstRanPoint.y)*(thirdRanPoint.z - firstRanPoint.z) - (secondRanPoint.z - firstRanPoint.z)*(thirdRanPoint.y - firstRanPoint.y);
		b = (secondRanPoint.z - firstRanPoint.z)*(thirdRanPoint.x - firstRanPoint.x) - (secondRanPoint.x - firstRanPoint.x)*(thirdRanPoint.z - firstRanPoint.z);
		c = (secondRanPoint.x - firstRanPoint.x)*(thirdRanPoint.y - firstRanPoint.y) - (secondRanPoint.y - firstRanPoint.y)*(thirdRanPoint.x - firstRanPoint.x);
		d = -1 * (a*firstRanPoint.x + b*firstRanPoint.y + c*firstRanPoint.z);
		for (int j = 0; j<max_idx; j++){
			if (j!= firstRanIdx && j!= secondRanIdx){
				Point = (*cloud).points[j];
				dist = std::fabs(a*Point.x + b*Point.y + c*Point.z + d) / std::sqrt(a*a + b*b + c*c);
				if (dist < distanceTol)
					inliersTempResult.insert(j);
			}
		}
		// printf("i th: %d \t inliersTempResult.size(): %d \n",i, inliersTempResult.size());
		// printf("i th: %d \t inliersResult.size(): %d \n",i, inliersResult.size());
		// Return indicies of inliers from fitted line with most inliers
		if (inliersTempResult.size() > numinliers){
			
			inliersResult.clear();
			numinliers = inliersTempResult.size();

			std::unordered_set<int>::iterator iter;
			for(iter = inliersTempResult.begin(); iter != inliersTempResult.end(); iter++){
				inliersResult.insert(*iter);
			}
			inliersTempResult.clear();
		}
	}

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;


	return inliersResult;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold, bool pclLibrary)
{   
    std::cout << "=====Plane segmentation process=====" << "\n";

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.
    // SegmentPlane using pcl Library
    if (pclLibrary){
        // Create the segmentation object
        pcl::SACSegmentation<PointT> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (maxIterations);
        seg.setDistanceThreshold (distanceThreshold);

        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl << std::endl;
        }

        
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "(PCL lib)plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

        std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
        return segResult;
    }
    // SegmentPlane using custom function
    else{

        std::unordered_set<int> inliers = Ransac3D(cloud, 100 , 0.17);

        typename pcl::PointCloud<PointT>::Ptr  planeCloud(new pcl::PointCloud<PointT>());
        typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());

        for(int index = 0; index < cloud->points.size(); index++)
        {   
            PointT point = cloud->points[index];
            if(inliers.count(index))
                planeCloud->points.push_back(point);
            else
                obstCloud->points.push_back(point);
        }

        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "(Custom)plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

        std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud,planeCloud);
        return segResult;
    }
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& pointsCheck , int pointIdx, KdTree* tree, float distanceTol, unsigned char pointDim)
{	
	pointsCheck[pointIdx] = true;
	cluster.push_back(pointIdx);
    std::vector<float> point(3);
    point[0] = cloud->points[pointIdx].x;
    point[1] = cloud->points[pointIdx].y;
    point[2] = cloud->points[pointIdx].z;
	std::vector<int> nearByPointIdicies = tree->search(point, distanceTol, pointDim);
	for(int nearByIdx:nearByPointIdicies){
		if (pointsCheck[nearByIdx]==false)
			clusterHelper(cloud, cluster, pointsCheck, nearByIdx, tree, distanceTol, pointDim);
	}
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, unsigned char pointDim)
{
	// TODO: Fill out this function to return list of indices for each cluster
	std::vector<std::vector<int>> clusters;
	std::vector<bool> pointsCheck(cloud->points.size(), false);
	for( int i =0; i<cloud->points.size(); i++ ){
		if (pointsCheck[i] == false){
			std::vector<int> cluster;
			clusterHelper(cloud, cluster, pointsCheck, i, tree, distanceTol, pointDim);
			clusters.push_back(cluster);
		}
	}
	return clusters;
}



template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize, bool pclLibrary)
{

    std::cout << "=====Clustering process=====" << "\n";

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clustersPCD;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Clustering using pcl Library
    if(pclLibrary){
        
        // Creating the KdTree object for the search method of the extraction
        // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        tree->setInputCloud (cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance (clusterTolerance); // If it's 1, it means 1m.
        ec.setMinClusterSize (minSize);
        ec.setMaxClusterSize (maxSize);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud);
        ec.extract (cluster_indices);

        int cluster_idx = 0;
        for (pcl::PointIndices index:cluster_indices )
        {   
            // new를 이용하여 메모리 동적할당을 하지 않고 사용하면, Segmentation fault가 발생함 !! 
            typename pcl::PointCloud<PointT>::Ptr cluster_pointData (new pcl::PointCloud<PointT>);
            for (int idx : index.indices){
                // std::cout <<"idx: "<< idx << " \tcloud->points[idx]: \t " << cloud->points[idx] <<"\n";
                cluster_pointData->points.push_back(cloud->points[idx]);
            }
            cluster_pointData->width =  cluster_pointData->points.size();
            cluster_pointData->height = 1;
            cluster_pointData->is_dense = true;
            clustersPCD.push_back(cluster_pointData);
        }       
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "(PCL lib)clustering took " << elapsedTime.count() << " milliseconds and found " << clustersPCD.size() << " clustersPCD" << std::endl << std::endl;
        return clustersPCD;
    }
    // Clustering using custom function
    else{

        // Creating the KdTree object for the search method of the extraction
        KdTree* tree = new KdTree();
        unsigned char pointDimension = 3;
        std::vector<float> point(3);
        for (int i=0; i<cloud->points.size(); i++){
            point[0] = cloud->points[i].x;
            point[1] = cloud->points[i].y;
            point[2] = cloud->points[i].z;
    	    tree->insert(point, i, pointDimension); 
        }
        std::vector<std::vector<int>> clusters = euclideanCluster(cloud, tree, clusterTolerance, pointDimension);
        
        for(std::vector<int> cluster : clusters){
            typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
            for(int indice: cluster)
                clusterCloud->points.push_back(PointT(cloud->points[indice].x,cloud->points[indice].y,cloud->points[indice].z));
            if (clusterCloud->points.size() > minSize && clusterCloud->points.size() < maxSize)
            {
                clusterCloud->width =  clusterCloud->points.size();
                clusterCloud->height = 1;
                clusterCloud->is_dense = true;
                clustersPCD.push_back(clusterCloud);
            }
        }
        
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "(Custom)clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl << std::endl;

        return clustersPCD;
   
    }

    
}



template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);
    // std::cout <<"boundBox minPoint.x " << minPoint.x << "\n";
    // std::cout <<"boundBox maxPoint.x " << maxPoint.x << "\n";
    // std::cout <<"boundBox minPoint.y " << minPoint.y << "\n";
    // std::cout <<"boundBox maxPoint.y " << maxPoint.y << "\n";
    // std::cout <<"boundBox minPoint.z " << minPoint.z << "\n";
    // std::cout <<"boundBox maxPoint.z " << maxPoint.z << "\n";
    
    
    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl<<std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}