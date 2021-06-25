
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
#include "object_recognition_pkg/bbox.h"

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


ros::Publisher data_completed_pub;


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


void subMain()
{
    // /******************** remove cube, text, and colorized clustered cloud in visualizer if exis ****************************/ 
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

    if(pcdClusteredIds.size())
    {
        for(auto it : pcdClusteredIds)
        {
            viewer->removePointCloud(it);
        }
        pcdClusteredIds.clear();
    }
    // /******************** remove cube, text, and colorized clustered cloud in visualizer if exis-end ****************************/ 
    

    std::vector<modelsDetail> objectData; 
    /******************** filterinig-start ****************************/ 
    
    // // Filter Removing NaN data Pointcloud.
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*acquiredCloud, *acquiredCloud, mapping);

    // Filter object crop by z coordinate
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (acquiredCloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 2.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*acquiredCloud);
    // std::cerr << "Pointcloud after cropped : " << cloud->size() << " data points." << std::endl;

    // Filter downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (acquiredCloud);
    vg.setLeafSize (0.005f, 0.005f, 0.005f); // 0.5cm
    vg.filter (*acquiredCloud);
    // std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*

    /******************** filterinig-end ****************************/ 
    

    /******************** segmentation-end ****************************/ 

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.01);


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
    /******************** segmentation-end ****************************/ 



    /******************** clustering-start ****************************/ 
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (acquiredCloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clustered (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (acquiredCloud);
    ec.extract (cluster_indices);

    //// get the cluster models
    i = 0;
    // std::vector<modelsDetail> dataClustered;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->push_back ((*acquiredCloud)[*pit]); //*

        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;


        modelsDetail modelClustered;
        modelClustered.second = cloud_cluster;
        modelClustered.first.no = i; 
        objectData.push_back (modelClustered);
        

        //get points value of bounding box
        pcl::PointXYZRGB minPt, maxPt;
        pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);
        object_recognition_pkg::bbox bbox_msg;
        bbox_msg.xmin = minPt.x;
        bbox_msg.ymin = minPt.y;
        bbox_msg.zmin = minPt.z;
        bbox_msg.xmax = maxPt.x;
        bbox_msg.ymax = maxPt.y;
        bbox_msg.zmax = maxPt.z;
        data_completed_msg.bboxs.push_back(bbox_msg);

        // if(cloud_clustered->size() < cloud_cluster->size())
        // *cloud_clustered = *cloud_cluster;

        // BoundingBox(cloud_cluster, viewer);
        // viewer.addPointCloud(cloud_cluster);

        // pcl::PointXYZRGB minPt, maxPt;
        // pcl::getMinMax3D (*cloud_cluster, minPt, maxPt);
        // std::string bboxId = "BBOX" + std::to_string(rand() % 100);
        // viewer->addCube(minPt.x, maxPt.x,  -maxPt.y , -minPt.y, -maxPt.z, -minPt.z, 1.0, 1.0, 1.0, bboxId, 0 );
        // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, bboxId);
        // // viewer->addText("asdasdasdasd asdasdasdasd adasdas das d", maxPt.x, maxPt.y, textId);
    }
    // *acquiredCloud = *cloud_clustered;
    /******************** clustering-end ****************************/ 


    /******************** descriptoring-start ****************************/ 
    //// VFH DESCRIPTOR  
    std::vector<modelsDetail> dataAddVFH;
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
    
    VFHEstimationType vfhEstimation;
    i = 0;
    for(auto it : objectData )
    {

        //  Compute the normals
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
        normalEstimation.setInputCloud (it.second);
        normalEstimation.setSearchMethod (tree);
        normalEstimation.setRadiusSearch (0.01);// (0.03); //newDatasetv3.0 up version user 0.01
        pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);
        normalEstimation.compute (*cloudWithNormals);
        // std::cout << "Computed " << cloudWithNormals->points.size() << " normals." << std::endl;
        
        // Setup the feature computation
        // Provide the original point cloud (without normals)
        vfhEstimation.setInputCloud (it.second);    // Provide the point cloud with normals
        vfhEstimation.setInputNormals(cloudWithNormals);    // Use the same KdTree from the normal estimation
        vfhEstimation.setSearchMethod (tree);
        // vfhEstimation.setRadiusSearch (0.2); // With this, error: "Both radius (.2) and K (1) defined! Set one of them to zero first and then re-run compute()"
        // Actually compute the VFH features
        pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhFeatures(new pcl::PointCloud<pcl::VFHSignature308>);
        vfhEstimation.compute (*vfhFeatures);
        *vfhFeature = *vfhFeatures;
        // pcl::visualization::PCLHistogramVisualizer viewerHistogram = createHistogramVisualizer(vfhFeature);
        // std::cout << "output points.size (): " << vfhFeatures->points.size () << std::endl; // This outputs 1 - should be 397!

        // Display and retrieve the shape context descriptor vector for the 0th point.
        // pcl::VFHSignature308 descriptor = vfhFeatures->points[0];
        // VFHEstimationType::PointCloudOut::PointType descriptor2 = vfhFeatures->points[0];
        // std::cout << descriptor << std::endl;

        // save histogram VFH of clustered object
        // std::stringstream ss;
        // ss << "src/object_recognition_pkg/output/vfh/vfh_" << i << ".pcd";
        // writer.write<pcl::VFHSignature308> (ss.str (), *vfhFeatures, false); //*
        // i++;

        //push to vector 
        std::vector<float> VFHValue(
            vfhFeatures->points[0].histogram,
            vfhFeatures->points[0].histogram + 308 
        );

        it.first.histogram = VFHValue;
        it.first.vfhFeature = vfhFeatures;
        dataAddVFH.push_back(it);

        
        //push to object_recognition_pkg::data_completed_msg publisher
        object_recognition_pkg::vfh vfh_msg;
        int j = 0;
        for( j=0; j<308; j++)
        {
            vfh_msg.vfh[j] = VFHValue[j];
            
        }
        // vfh_msg.no = i;
        data_completed_msg.vfhs.push_back(vfh_msg);


        data_completed_msg.id.push_back(it.first.no);
        // sensor_msgs::PointCloud2Ptr cloudMsg (new sensor_msgs::PointCloud2());
        // pcl::toROSMsg(*it.second, *cloudMsg);
        // data_completed_msg.clustered.push_back(*cloudMsg);
        //end of push to object_recognition_pkg::data_completed_msg publisher
        

        // viewerHistogram.updateFeatureHistogram(*vfhFeatures, 308);
        
        // viewerHistogram.spinOnce();    // std::cout << 
        // std::cout << it.first.histogram.size() << " -";

    }
    // std::cout << std::endl;
    // std::cout << "jumlah vfh : " << VFHValues.size() << std::endl;

    /******************** descriptoring-end ****************************/ 

    //publish data_completed




    /******************** feature-matching-start ****************************/ 
    // ARTIFICIAL NEURAL NETOWRK
    std::vector<modelsDetail> dataCompleted;
    struct fann *ann = fann_create_from_file("fann/newDatasetv6.3.net"); // generated from training
    fann_type *calc_out;
    fann_type input[308]; //length of VFH Descriptor
    i = 0;
    for(auto it : dataAddVFH)
    {
        std::copy(it.first.histogram.begin(), it.first.histogram.end(), input);

        calc_out = fann_run(ann, input);

        //find max score 
        int maxIndex = 0, maxValue = 0;
        for(int i = 0 ; i < ann->num_output ; i++)
        {
            // std::cout << calc_out[i]  << " " ;//<< std::endl;
            //  std::cout << (calc_out[i] * 100) << " % " 
            //     << "   " << listDataSet[i] 
            //     << std::endl;

            if(calc_out[i] > maxValue){
                maxIndex = i;
            }

        }
        std::cout << i << ". " << "max : " << calc_out[maxIndex] * 100 << "% " << listDataSet[maxIndex] << std::endl;
        it.first.category = listDataSet[maxIndex];
        it.first.score = calc_out[maxIndex]*100.0;
        dataCompleted.push_back(it);
        i++;
        
    }
    // std::cout << std::endl;
    fann_destroy(ann);

    /******************** feature-matching-end ****************************/ 


    /******************** Visualize bounding box ****************************/ 

    //visualize
    for(auto it:dataCompleted)
    {
        pcl::PointXYZRGB minPt, maxPt;
        pcl::getMinMax3D (*it.second, minPt, maxPt);
        // std::cout << "Min x: " << minPt.x << std::endl;
        // std::cout << "Max x: " << maxPt.x << std::endl;
        // std::cout << "Min y: " << minPt.y << std::endl;
        // std::cout << "Max y: " << maxPt.y << std::endl;
        // std::cout << "Min z: " << minPt.z << std::endl;
        // std::cout << "Max z: " << maxPt.z << std::endl;
        std::string bboxId = "BBOX" + std::to_string(rand() % 100);
        std::string textId = "TEXT" + std::to_string(rand() % 100);
        cubeIds.push_back(bboxId);
        textIds.push_back(textId);
        std::stringstream stream;
        stream << std::fixed << std::setprecision(2) << it.first.score;
        std::string textLabel = (it.first.category + " " + stream.str() + "%"); 
        // std::cout << "bboxId : " << bboxId << std::endl;
        viewer->addCube(minPt.x, maxPt.x,  minPt.y , maxPt.y,  minPt.z, maxPt.z, 1.0, 1.0, 1.0, bboxId, 0 );
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, bboxId);
        // viewer->addText("asdasdasdasd asdasdasdasd adasdas das d", maxPt.x, maxPt.y, textId);

        viewer->addText3D(textLabel, pcl::PointXYZRGB(maxPt.x, maxPt.y, minPt.z), 0.01, 255.0, 1.0, 1.0,  textId);
    }
    /******************** Visualize bounding box-end ****************************/ 

    /******************** Visualize add clustered color ****************************/     

    if(dataCompleted.size())
    {
        for(auto it : dataCompleted)
        {
            std::string pcdClusteredId = "CLUSTER" + std::to_string(rand() % 100);
            pcdClusteredIds.push_back(pcdClusteredId);

            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color(it.second, 255, 0, 0);
            viewer->addPointCloud<pcl::PointXYZRGB>(it.second, color, pcdClusteredId);
        }
    }
    /******************** Visualize add clustered color-end ****************************/ 
}


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



	viewer->updatePointCloud(acquiredCloud);

    subMain();
    
    // data_completed_msg.rawRGBD = *cloudInMsg;
    data_completed_pub.publish(data_completed_msg);

    

    // viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
    // if(save_cloud_ == true){
    //     save_cloud_ = false;
    //     pcd_filename_ = "capture_dataset/new/water_can/pcl" + std::to_string(counter_) + ".pcd";
    //     pcl::io::savePCDFileASCII(pcd_filename_, *acquiredCloud);

    //     counter_++; 
    // }

	// viewer->updatePointCloud(acquiredCloud);
    // viewerHistogram.updateFeatureHistogram(*vfhFeature, 308);

	viewer->spinOnce(); //moved to other node
    // viewerHistogram.spin();

}

void imageAcquirerReceive(const sensor_msgs::ImageConstPtr& msg){
	acquiredImage = cv::Mat(cv_bridge::toCvShare(msg, "bgr8")->image);
  cv::rotate(acquiredImage, acquiredImageRotate, cv::ROTATE_180);
	imageShow(acquiredImage);
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "ROSpclVisualizer");

    ros::NodeHandle nh;
    data_completed_pub = nh.advertise<object_recognition_pkg::data_completed>("/data_completed",10);


    // READ label/class trainned (dataset)
    std::ifstream listDataSetFile("fann/newDatasetv6.3.txt");
    for (std::string line ; getline(listDataSetFile, line);)
    {

        // std::vector<std::string> stringLabelParsed;
        // boost::algorithm::split(stringLabelParsed, line, boost::is_any_of("/"));
        // listDataSet.push_back(stringLabelParsed[1]);
        listDataSet.push_back(line);
    }


	image_transport::ImageTransport it(nh);

	cv::namedWindow("OCV RS 2D Viewer", cv::WINDOW_AUTOSIZE);
	// cv::startWindowThread();

	image_transport::Subscriber sub = it.subscribe("/RSimgAcquisition", 1, imageAcquirerReceive);
    ros::Subscriber pclsubAcquirer = nh.subscribe("/RSpclAcquisition", 1, cloudAcquirerReceive);

	ros::spin();
	cv::destroyWindow("OCV RS 2D Viewer");
}