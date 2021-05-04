#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/String.h"
#include "object_recognition_pkg/trackbar.h"

int param[6] = {0};
int Xmin = 100; 
int Xmax = 100;
int Ymin = 100;
int Ymax = 100;
int Zmin = 100;
int Zmax = 100;
int vox_grid_X = 5; //default on subcriber 0.005f
int vox_grid_Y = 5; //default on subcriber 0.005f
int vox_grid_Z = 5; //default on subcriber 0.005f
int seg_max_iteration = 100; //default on subcriber 100
int seg_dist_thresh = 10; //default on subcriber 0.01
object_recognition_pkg::trackbar trackbarVal;

std_msgs::String::Ptr msg(new std_msgs::String());

void parsingParameter6(std::string inputString)
{
    size_t i = 0;
    std::string parse;
    std::stringstream ss(inputString);
    while (ss >> parse)
    {
        param[i] = std::stof(parse);
        i++;
    }
    std::cout << param[0] << " " << param[1] << " " << param[2] << " " << param[3] << " " << param[4] << " " << param[5] << std::endl;
}

int main(int argc, char **argv)
{
    // cv::Mat src = cv::imread("../Screenshot from 2021-04-12 00-16-30.png");
    cv::Mat src(10, 240, CV_8UC3, cv::Scalar(0, 0, 0));

    ros::init(argc, argv, "Array_pub");
    ros::NodeHandle nh;

    // ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("/chatter", 1);
    ros::Publisher chatter_pub = nh.advertise<object_recognition_pkg::trackbar>("/chatter", 1);    
    ros::Rate loop_rate(10);

    // std::string inputString = "52 51 55";

    if (!src.data)
    {
        std::cout << "Error loading the image" << std::endl;
        return -1;
    }
    cv::namedWindow("My Window", 1);

    cv::createTrackbar("-Xmin/100: ", "My Window", &Xmin, 100);
    cv::createTrackbar("Xmax/100: ", "My Window", &Xmax, 100);
    cv::createTrackbar("-Ymin/100: ", "My Window", &Ymin, 100);
    cv::createTrackbar("Ymax/100: ", "My Window", &Ymax, 100);
    cv::createTrackbar("-Zmin/100: ", "My Window", &Zmin, 100);
    cv::createTrackbar("Zmax/100: ", "My Window", &Zmax, 100);
    cv::createTrackbar("vox_grid_X/1000: ", "My Window", &vox_grid_X, 100);
    cv::createTrackbar("vox_grid_Y/1000: ", "My Window", &vox_grid_Y, 100);
    cv::createTrackbar("vox_grid_Z/1000: ", "My Window", &vox_grid_Z, 100);
    cv::createTrackbar("seg_max_it/1: ", "My Window", &seg_max_iteration, 100);
    cv::createTrackbar("seg_dist_th/1000: ", "My Window", &seg_dist_thresh, 1000);
    

    while (true)
    {
        cv::imshow("My Window", src);
        // std::stringstream ss;
        // ss << (float)Xmin/(float)100 << " " << (float)Xmax/(float)100 << " " << (float)Ymin/(float)100 << " " << (float)Ymax/(float)100 << " " << (float)Zmin/(float)100 << " " << (float)Zmax/(float)100;
        // // std::cout << ss.str() << std::endl;
        // msg->data = (ss.str());
        //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*

        // std_msgs::Float32 trackbarMsg;
        // trackbarMsg.data.xmin = (float)(Xmin/100);
        // trackbarMsg.xmax = (float)(Xmax/100);
        // trackbarMsg.ymin = (float)(Ymin/100);
        // trackbarMsg.ymax = (float)(Ymax/100);
        // trackbarMsg.zmin = (float)(Zmin/100);
        // trackbarMsg.zmax = (float)(Zmax/100);

        object_recognition_pkg::trackbar trackbarMsg;
        trackbarMsg.minX = -(float)Xmin/(float)100;
        trackbarMsg.maxX = (float)Xmax/(float)100;
        trackbarMsg.minY = -(float)Ymin/(float)100;
        trackbarMsg.maxY = (float)Ymax/(float)100;
        trackbarMsg.minZ = -(float)Zmin/(float)100;
        trackbarMsg.maxZ = (float)Zmax/(float)100;
        trackbarMsg.leaf_vox_gridZ = (float)vox_grid_X/(float)1000;
        trackbarMsg.leaf_vox_gridZ = (float)vox_grid_Y/(float)1000;
        trackbarMsg.leaf_vox_gridZ = (float)vox_grid_Z/(float)1000;
        trackbarMsg.seg_max_iteration = seg_max_iteration;
        trackbarMsg.seg_dist_thresh = (float)seg_dist_thresh/(float)1000;
        


        // chatter_pub.publish(msg);
        chatter_pub.publish(trackbarMsg);
        ros::spinOnce();
        loop_rate.sleep();

        // std::cout << X << " - " << Y << " - " << Z << " " << std::endl;
        int iKey = cv::waitKey(50);
        //if user press 'ESC' key
        if (iKey == 27 || iKey == 'q')
        {
            break;
        }
    }
    return 0;
}