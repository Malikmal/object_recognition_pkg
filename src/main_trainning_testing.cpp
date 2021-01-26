#include <iostream>
#include <vector>
#include <algorithm>
#include <string>

#include "fann.h"
#include "floatfann.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/histogram_visualizer.h>

// argument 1 => file ex : scene_mug_table.pcd 

int 
main (int argc, char** argv)
{

// get date label

    // /** percobaan **
    std::vector<std::string> listDataSet;
    std::ifstream listDataSetFile("listDataSetv2.1.txt");
    for (std::string line ; getline(listDataSetFile, line);)
    {

        std::vector<std::string> stringLabelParsed;
        boost::algorithm::split(stringLabelParsed, line, boost::is_any_of("/"));
        listDataSet.push_back(stringLabelParsed[1]);
    }

// Read in the cloud data
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    reader.read (argv[1], *cloud);
    std::cout << "PointCloud Readed: " << cloud->size () << " data points." << std::endl; //*


//// VFH DESCRIPTOR  
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    typedef pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> VFHEstimationType;
    VFHEstimationType vfhEstimation;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setInputCloud (cloud);
    normalEstimation.setSearchMethod (tree);
    normalEstimation.setRadiusSearch (0.03);
    pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);
    normalEstimation.compute (*cloudWithNormals);

    vfhEstimation.setInputCloud (cloud);    // Provide the point cloud with normals
    vfhEstimation.setInputNormals(cloudWithNormals);    // Use the same KdTree from the normal estimation
    vfhEstimation.setSearchMethod (tree);
    // vfhEstimation.setRadiusSearch (0.2); // With this, error: "Both radius (.2) and K (1) defined! Set one of them to zero first and then re-run compute()"
    // Actually compute the VFH features
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhFeatures(new pcl::PointCloud<pcl::VFHSignature308>);
    vfhEstimation.compute (*vfhFeatures);

    std::vector<float> VFHValue(
        vfhFeatures->points[0].histogram,
        vfhFeatures->points[0].histogram + 308 
    );

// ARTIFICIAL NEURAL NETOWRK
    struct fann *ann = fann_create_from_file("bbbbbbb.net"); // generated from training
    fann_type *calc_out;
    fann_type input[308]; //length of VFH Descriptor

    std::copy(VFHValue.begin(), VFHValue.end(), input);

    calc_out = fann_run(ann, input);

    for(int i = 0 ; i < ann->num_output ; i++)
    {
        std::cout << (calc_out[i] * 100) << " % " 
                << "   " << listDataSet[i] 
                << std::endl;
    }

    fann_destroy(ann);


    return (0);
}