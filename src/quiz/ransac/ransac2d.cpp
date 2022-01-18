/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

// for random smapling
#include <iostream>
#include <random>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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
	float dist = -1;
	int numinliers = 0;
	// variable for line formula
	float a,b,c;
	// variable for x,y,z point
	pcl::PointXYZ firstRanPoint, secondRanPoint, Point; 
	for (int i = 0; i<maxIterations; i++){
		// Randomly sample subset and fit line
		
		// Generate random_device to get a seed.
		std::random_device rd;

		// Init random number generator through random_device.
		std::mt19937 gen(rd());

		// Define uniform distribution to pick random number in btw 0 to max_idx
		std::uniform_int_distribution<int> dis(0, max_idx);
		
		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		firstRanIdx =  dis(gen);
		firstRanPoint = (*cloud).points[firstRanIdx];
		inliersTempResult.insert(firstRanIdx);
		secondRanIdx = dis(gen);
		secondRanPoint = (*cloud).points[secondRanIdx];
		inliersTempResult.insert(secondRanIdx);

		a = firstRanPoint.y - secondRanPoint.y;
		b = secondRanPoint.x - firstRanPoint.x;
		c = (firstRanPoint.x*secondRanPoint.y - secondRanPoint.x*firstRanPoint.y);
		for (int j = 0; j<max_idx; j++){
			if (j!= firstRanIdx && j!= secondRanIdx){
				Point = (*cloud).points[j];
				dist = std::fabs(a*Point.x + b*Point.y + c) / std::sqrt(a*a + b*b);
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
		else{inliersTempResult.clear();}
	}

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;


	return inliersResult;

}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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
	pcl::PointXYZ firstRanPoint, secondRanPoint, thirdRanPoint, Point; 
	for (int i = 0; i<maxIterations; i++){
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
		else{inliersTempResult.clear();}
	}

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;


	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// for 2D line.
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	// for 3D plane.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	// std::unordered_set<int> inliers = Ransac2D(cloud, 50 , 0.5);
	std::unordered_set<int> inliers = Ransac3D(cloud, 100 , 0.17);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
