
/**
 *  Subcribing RGB and Depth From camera_node.cpp
 * 
 *  don't forget to run roscore
 */


#include <ros/ros.h>
#include <time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <string>


#include "fann.h"
#include "floatfann.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/recognition/linemod/line_rgbd.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>

#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>


#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>

#include "object_recognition_pkg/trackbar.h"

using namespace std;

typedef pcl::PointXYZRGB XYZRGB;
typedef pcl::PointCloud<XYZRGB> pclXYZRGB;
typedef pcl::PointCloud<XYZRGB>::Ptr pclXYZRGBptr;
typedef pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> VFHEstimationType;


struct info{
  int no;
  std::string category;
  float score;
  std::vector<float> histogram;
  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhFeature;
} ;
typedef std::pair<info, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> modelsDetail;


bool save_cloud_ = false; //for capturing dataset
int counter_ = 0; //for capturing dataset
std::string pcd_filename_; //for capturing dataset

ros::Publisher pclpub;
ros::Subscriber pclsub;
ros::Subscriber strsub;

pclXYZRGBptr acquiredCloud (new pclXYZRGB());
pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhFeature(new pcl::PointCloud<pcl::VFHSignature308>());
std::vector<std::string> listDataSet;
cv::Mat acquiredImage, acquiredImageRotate;
sensor_msgs::PointCloud2Ptr cloudMsg(new sensor_msgs::PointCloud2());
// ros::init();
// ros::NodeHandle nh ;//(new ros::NodeHandle());
// ros::Publisher pclpub;// = nh.advertise<sensor_msgs::PointCloud2>("/RSpclPreprocessing", 1);
object_recognition_pkg::trackbar trackbarVal;


inline float PackRGB(uint8_t r, uint8_t g, uint8_t b) {
  uint32_t color_uint = ((uint32_t)r << 16 | (uint32_t) g << 8 | (uint32_t)b);
  return *reinterpret_cast<float *>(&color_uint);
}

//for capturing dataset
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void){
  if (event.getKeySym() == "m" && event.keyDown()){
    save_cloud_ = true;
  }
}

std::shared_ptr<pcl::visualization::PCLVisualizer> createXYZRGBVisualizer(pclXYZRGBptr cloud, pclXYZRGBptr cloud_f) {
	std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL RS 3D Viewer"));


    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0.12, 0.12, 0.12, v1);
    viewer->addText("original", 10, 10, "v1 text", v1);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "original", v1);
	// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, v1);

    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.12, 0.12, 0.12, v2);
    viewer->addText("output", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_f(cloud_f);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_f, rgb_f, "output", v2);
	// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, v2);

	viewer->initCameraParameters();
	viewer->addCoordinateSystem(1.0);
	return (viewer);
}

pcl::visualization::PCLHistogramVisualizer createHistogramVisualizer (pcl::PointCloud<pcl::VFHSignature308>::Ptr tmpVfhFeatures) 
{
    pcl::visualization::PCLHistogramVisualizer tmpViewerHistogram;

    tmpViewerHistogram.addFeatureHistogram(*tmpVfhFeatures, 308);

    return (tmpViewerHistogram);
}


void imageShow(const cv::Mat image){
	cv::imshow("OCV RS 2D Viewer", image);
	cv::waitKey(30);
}



void ConditionalRemovalFieldPOS(pclXYZRGBptr input, pclXYZRGBptr output)//, Eigen::Vector4f mindis, Eigen::Vector4f maxdis)
{
  pcl::ConditionAnd<XYZRGB>::Ptr ca (new pcl::ConditionAnd<XYZRGB>());
  //Set Positive
  ca->addComparison(pcl::FieldComparison<XYZRGB>::ConstPtr (new pcl::FieldComparison<XYZRGB>("x", pcl::ComparisonOps::GT, trackbarVal.minX)));
  ca->addComparison(pcl::FieldComparison<XYZRGB>::ConstPtr (new pcl::FieldComparison<XYZRGB>("x", pcl::ComparisonOps::LT, trackbarVal.maxX)));
  ca->addComparison(pcl::FieldComparison<XYZRGB>::ConstPtr (new pcl::FieldComparison<XYZRGB>("y", pcl::ComparisonOps::GT, trackbarVal.minY)));
  ca->addComparison(pcl::FieldComparison<XYZRGB>::ConstPtr (new pcl::FieldComparison<XYZRGB>("y", pcl::ComparisonOps::LT, trackbarVal.maxY)));
  ca->addComparison(pcl::FieldComparison<XYZRGB>::ConstPtr (new pcl::FieldComparison<XYZRGB>("z", pcl::ComparisonOps::GT, trackbarVal.minZ)));
  ca->addComparison(pcl::FieldComparison<XYZRGB>::ConstPtr (new pcl::FieldComparison<XYZRGB>("z", pcl::ComparisonOps::LT, trackbarVal.maxZ)));

  pcl::ConditionalRemoval<XYZRGB> cr;
  cr.setCondition (ca);
  cr.setInputCloud (input);
  cr.filter (*output);
}



void subMain()
{

    std::vector<modelsDetail> objectData; 
    /******************** filterinig-start ****************************/ 
    
    // // Filter Removing NaN data Pointcloud.
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*acquiredCloud, *acquiredCloud, mapping);

    // // Filter object crop by z coordinate
    // pcl::PassThrough<pcl::PointXYZRGB> pass;
    // pass.setInputCloud (acquiredCloud);
    // pass.setFilterFieldName ("z");
    // pass.setFilterLimits (0.0, 2.0);
    // //pass.setFilterLimitsNegative (true);
    // pass.filter (*acquiredCloud);
    // // std::cerr << "Pointcloud after cropped : " << cloud->size() << " data points." << std::endl;

    //filter by condifional removal (cropping)
    ConditionalRemovalFieldPOS(
        acquiredCloud,
        acquiredCloud
        // Eigen::Vector4f(trackbarVal.minX, trackbarVal.minY, trackbarVal.minZ, 1.0f), 
        // Eigen::Vector4f(trackbarVal.maxX,trackbarVal.maxY,trackbarVal.maxZ, 1.0f)
    );
    

    // // Filter downsample the dataset using a leaf size of 1cm
    // pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    // vg.setInputCloud (acquiredCloud);
    // // vg.setLeafSize (0.005f, 0.005f, 0.005f); // 0.5cm
    // vg.setLeafSize (
    //     trackbarVal.leaf_vox_gridX,
    //     trackbarVal.leaf_vox_gridY, 
    //     trackbarVal.leaf_vox_gridZ
    // ); // 0.5cm
    // vg.filter (*acquiredCloud);

    /******************** filterinig-end ****************************/ 
    

    // /******************** segmentation-end ****************************/ 

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations ( trackbarVal.seg_max_iteration); //100
    seg.setDistanceThreshold ( trackbarVal.seg_dist_thresh); //0.01f


    int i=0;
    int nr_points = acquiredCloud->size ();
    while (acquiredCloud->size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (acquiredCloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud (acquiredCloud);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        // std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*acquiredCloud);

        i++;
    }    
    // /******************** segmentation-end ****************************/

}

std::shared_ptr<pcl::visualization::PCLVisualizer> viewer = createXYZRGBVisualizer(acquiredCloud, acquiredCloud);
// pcl::visualization::PCLHistogramVisualizer viewerHistogram = createHistogramVisualizer(vfhFeature);


void cloudAcquirerReceive(const sensor_msgs::PointCloud2ConstPtr& cloudInMsg){
	pcl::fromROSMsg(*cloudInMsg, *acquiredCloud);


    int i = 640, j = 480, k;

    for (auto& it : acquiredCloud->points){
        it.x = it.x;
        it.y = it.y;
        it.z = it.z;
        it.rgb = PackRGB(
            acquiredImageRotate.at<cv::Vec3b>(j,i)[2],  //r
            acquiredImageRotate.at<cv::Vec3b>(j,i)[1], // g
            acquiredImageRotate.at<cv::Vec3b>(j,i)[0]  //b
        ); //acquiredImage explode
        i--;
        if(i <= 0)
        {
            i=640;
            j--;
        }
        if(j < 0)
        {
            break;
        }
    }

	viewer->updatePointCloud(acquiredCloud, "original");

    subMain();
    
    pcl::toROSMsg(*acquiredCloud, *cloudMsg);
    pclpub.publish(cloudMsg);

    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
    if(save_cloud_ == true){
        save_cloud_ = false;
        pcd_filename_ = "capture_dataset/new/water_can/pcl" + std::to_string(counter_) + ".pcd";
        pcl::io::savePCDFileASCII(pcd_filename_, *acquiredCloud);

        counter_++; 
    }

	viewer->updatePointCloud(acquiredCloud, "output");
    // viewerHistogram.updateFeatureHistogram(*vfhFeature, 308);

	viewer->spinOnce();
    // viewerHistogram.spin();

}

void imageAcquirerReceive(const sensor_msgs::ImageConstPtr& msg){
	acquiredImage = cv::Mat(cv_bridge::toCvShare(msg, "bgr8")->image); 
  cv::rotate(acquiredImage, acquiredImageRotate, cv::ROTATE_180);
	imageShow(acquiredImage);
}

void chatterReceive(const object_recognition_pkg::trackbar::ConstPtr& msg)
{

    trackbarVal.minX = msg->minX;
    trackbarVal.maxX = msg->maxX;
    trackbarVal.minY = msg->minY;
    trackbarVal.maxY = msg->maxY;
    trackbarVal.minZ = msg->minZ;
    trackbarVal.maxZ = msg->maxZ;
    
    trackbarVal.leaf_vox_gridX = msg->leaf_vox_gridX;
    trackbarVal.leaf_vox_gridY = msg->leaf_vox_gridY;
    trackbarVal.leaf_vox_gridZ = msg->leaf_vox_gridZ;
    
    trackbarVal.seg_max_iteration = msg->seg_max_iteration;
    trackbarVal.seg_dist_thresh = msg->seg_dist_thresh;
    
    
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ROSpclPreprocessing");
	ros::NodeHandle nh;
    // ros::Publisher 
    pclpub = nh.advertise<sensor_msgs::PointCloud2>("/RSpclPreprocessing", 1);

    // READ label/class trainned (dataset)
    std::ifstream listDataSetFile("listDataSetv2.1.txt");
    for (std::string line ; getline(listDataSetFile, line);)
    {

        std::vector<std::string> stringLabelParsed;
        boost::algorithm::split(stringLabelParsed, line, boost::is_any_of("/"));
        listDataSet.push_back(stringLabelParsed[1]);
    }


	image_transport::ImageTransport it(nh);

	cv::namedWindow("OCV RS 2D Viewer", cv::WINDOW_AUTOSIZE);
	// cv::startWindowThread();

    ros::Subscriber chatter = nh.subscribe("/chatter", 1, chatterReceive);
	image_transport::Subscriber sub = it.subscribe("/RSimgAcquisition", 1, imageAcquirerReceive);
    ros::Subscriber pclsubAcquirer = nh.subscribe("/RSpclAcquisition", 1, cloudAcquirerReceive);

	ros::spin();
	cv::destroyWindow("OCV RS 2D Viewer");
}