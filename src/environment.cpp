/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // Whether functions are run based on pcl Library or not.
    bool pclLibrary = false;

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar *lidarSense1 = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_scanData = lidarSense1->scan();

    // renderRays(viewer, lidarSense1->position , pc_scanData );
    
    // renderPointCloud(viewer, pc_scanData, "input_pcScanData", Color(1,1,1));

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();

    // SegmentPlane function part
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(pc_scanData, 100, 0.2, pclLibrary);
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    // Clustering function part
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.2, 3, 50, pclLibrary);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1), Color(0,1,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {   
        // render obstacle cluster
        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        // render bounding box 
        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }


}

// void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{   
    // Whether functions are run based on pcl Library or not.
    bool pclLibrary = false;

    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------
 
    // Input data filtering 
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterInputCloud (new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterRoofCloud (new pcl::PointCloud<pcl::PointXYZI>());

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.2 , Eigen::Vector4f (-10, -6.5, -7, 1), Eigen::Vector4f ( 25, 7.5, 7, 1));
    filterInputCloud = filteredCloud.first;
    filterRoofCloud  = filteredCloud.second;
    renderPointCloud(viewer,filterInputCloud,"filterCloud");
    // Original data 
    // renderPointCloud(viewer,inputCloud,"inputCloud");
    
    // // render bounding box    
    // Box rootFilteredBox = pointProcessorI->BoundingBox(filterRoofCloud);
    // renderBox(viewer,rootFilteredBox,-1, Color(0.5, 0, 0.5));

    // // SegmentPlane function part
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterInputCloud, 100, 0.2, pclLibrary);
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    // Clustering function part
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.50, 15, 620, pclLibrary);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0.6,0.2,0), Color(0,0,1), Color(0,1,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {   
        // render obstacle cluster
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        // render bounding box 
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }

}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    // cityBlock(viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI (new ProcessPointClouds<pcl::PointXYZI>());
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    // just for 1 frame
    // while (!viewer->wasStopped ())
    // {
    //       // Clear viewer
    //     // viewer->removeAllPointClouds();
    //     // viewer->removeAllShapes();

    //     // // Load pcd and run obstacle detection process
    //     inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    //     cityBlock(viewer, pointProcessorI, inputCloudI);

    //     // streamIterator++;
    //     // if(streamIterator == stream.end())
    //     //     streamIterator = stream.begin();

    //     viewer->spinOnce ();
    // } 

    while (!viewer->wasStopped ())
    {
          // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}