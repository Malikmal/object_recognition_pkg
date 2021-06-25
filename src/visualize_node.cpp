
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
#include <pcl/features/vfh.h>
#include <pcl/features/our_cvfh.h>
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

#include "object_recognition_pkg/data_completed.h"
#include "object_recognition_pkg/vfh.h"

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
std::vector<std::string> cubeIds;
std::vector<std::string> textIds;
std::vector<std::string> pcdClusteredIds;

ros::Publisher pclpub;
ros::Subscriber pclsub;
ros::Subscriber strsub;

pclXYZRGBptr acquiredCloud (new pclXYZRGB());
pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhFeature(new pcl::PointCloud<pcl::VFHSignature308>());
std::vector<std::string> listDataSet;
cv::Mat acquiredImage, acquiredImageRotate;


// ros::Publisher data_completed_pub;


object_recognition_pkg::data_completed data_completed_msg;

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

std::shared_ptr<pcl::visualization::PCLVisualizer> createXYZRGBVisualizer(pclXYZRGBptr cloud) {
	std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL RS 3D Viewer"));
	viewer->setBackgroundColor(0.12, 0.12, 0.12);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
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


std::shared_ptr<pcl::visualization::PCLVisualizer> viewer = createXYZRGBVisualizer(acquiredCloud);
// pcl::visualization::PCLHistogramVisualizer viewerHistogram = createHistogramVisualizer(vfhFeature);


void cloudAcquirerReceive(const sensor_msgs::PointCloud2ConstPtr& cloudInMsg){

    if(cubeIds.size())
    {
        for (auto it: cubeIds)
        {
            /* code */
            viewer->removeShape(it);
        }
        cubeIds.clear();
    }

    if(textIds.size())
    {
        for (auto it: textIds)
        {
            /* code */
            viewer->removeText3D(it);
        }
        textIds.clear();
    }


    // std::cout << "Teststst " << std::endl;
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

    

	viewer->updatePointCloud(acquiredCloud);

    // // subMain();

    
    /******************** remove cube, text, and colorized clustered cloud in visualizer if exis ****************************/ 
    

    // if(pcdClusteredIds.size())
    // {
    //     for(auto it : pcdClusteredIds)
    //     {
    //         viewer->removePointCloud(it);
    //     }
    //     pcdClusteredIds.clear();
    // }
    /******************** remove cube, text, and colorized clustered cloud in visualizer if exis-end ****************************/ 
    
    

    /******************** Visualize bounding box ****************************/ 

    //visualize
    i = 0;
    for(auto it:data_completed_msg.bboxs)
    {
        std::string bboxId = "BBOX" + i + std::to_string(rand() % 100) + std::to_string(rand() % 100) + std::to_string(rand() % 100);
        std::string textId = "TEXT" + i + std::to_string(rand() % 100) + std::to_string(rand() % 100) + std::to_string(rand() % 100);
        cubeIds.push_back(bboxId);
        textIds.push_back(textId);
        std::stringstream stream;
        stream << std::fixed << std::setprecision(2) << data_completed_msg.score[i];
        std::string textLabel = (data_completed_msg.label[i] + " " + stream.str() + "%"); 
        // std::cout << "bboxId : " << bboxId << std::endl;
        viewer->addCube(
            it.xmin, //minPt.x, 
            it.xmax, //maxPt.x, 
            it.ymin, //minPt.y, 
            it.ymax, //maxPt.y, 
            it.zmin, //minPt.z, 
            it.zmax, //maxPt.z, 
            1.0, 1.0, 1.0, bboxId 
        );
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, bboxId);
        // viewer->addText("asdasdasdasd asdasdasdasd adasdas das d", maxPt.x, maxPt.y, textId);

        viewer->addText3D(
            textLabel, 
            pcl::PointXYZRGB(it.xmax, it.ymax, it.zmax) , //pcl::PointXYZRGB(maxPt.x, maxPt.y, minPt.z), 
            0.01, 255.0, 1.0, 1.0,  textId
        );

        i++;
    }
    /******************** Visualize bounding box-end ****************************/ 
    

    // viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
    // if(save_cloud_ == true){
    //     save_cloud_ = false;
    //     pcd_filename_ = "capture_dataset/new/water_can/pcl" + std::to_string(counter_) + ".pcd";
    //     pcl::io::savePCDFileASCII(pcd_filename_, *acquiredCloud);

    //     counter_++; 
    // }

	// // viewer->updatePointCloud(acquiredCloud);
    // // viewerHistogram.updateFeatureHistogram(*vfhFeature, 308);

	viewer->spinOnce(); //moved to other node
    // // viewerHistogram.spin();

}


void imageAcquirerReceive(const sensor_msgs::ImageConstPtr& msg){
	acquiredImage = cv::Mat(cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::rotate(acquiredImage, acquiredImageRotate, cv::ROTATE_180);
	imageShow(acquiredImage);
}

void dataPredictedPublisherCallback(const object_recognition_pkg::data_completed& msg)
{
    data_completed_msg = msg;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "visualize_node");

    ros::NodeHandle nh;
    // data_completed_pub = nh.advertise<object_recognition_pkg::data_completed>("/data_predicted",10);


    // READ label/class trainned (dataset)
    std::ifstream listDataSetFile("newDatasetv6.5.txt");
    for (std::string line ; getline(listDataSetFile, line);)
    {

        // std::vector<std::string> stringLabelParsed;
        // boost::algorithm::split(stringLabelParsed, line, boost::is_any_of("/"));
        // listDataSet.push_back(stringLabelParsed[1]);
        listDataSet.push_back(line);
    }


	image_transport::ImageTransport it(nh);

	cv::namedWindow("OCV RS 2D Viewer", cv::WINDOW_AUTOSIZE);
	cv::startWindowThread();

	image_transport::Subscriber sub = it.subscribe("/RSimgAcquisition", 1, imageAcquirerReceive);
    ros::Subscriber pclsubAcquirer = nh.subscribe("/RSpclAcquisition", 1, cloudAcquirerReceive);

    ros::Subscriber dataPredictedSubcriber = nh.subscribe("/data_predicted", 1, dataPredictedPublisherCallback);

	ros::spin();
	// cv::destroyWindow("OCV RS 2D Viewer");
}