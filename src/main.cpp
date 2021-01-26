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

struct info{
  int no;
  std::string category;
  std::vector<float> histogram;
} ;
typedef std::pair<info, pcl::PointCloud<pcl::PointXYZ>::Ptr> modelsDetail;

// typedef std::pair<info, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> modelRaw;


int 
main (int argc, char** argv)
{

    /** percobaan **
    // std::vector<std::string> listDataSet;
    // std::ifstream listDataSetFile("listDataSetv2.1.txt");
    // for (std::string line ; getline(listDataSetFile, line);)
    // {
    //     listDataSet.push_back(line);
    // }
    // // std::cout << listDataSet.size();
    // fann_type result[listDataSet.size()] = {0, 0, 0.2, 0.4, 0.9, 0.1, 0, 0, 0, 0, 0, 0, 0, 0};

    // std::vector< std::pair<std::string, std::vector<float> > > resultDataSet; 
    // for (int i = 0; i < listDataSet.size(); i++)
    // {
    //     // float tmp = (float);
    //     // result[i] = (2/i);
    //     // resultDataSet 
    //     std::pair<std::string, std::vector<float> > hist;
    //     std::vector<float> histBin;
    //     for(int j = 0; j < listDataSet.size(); j++)
    //     {
    //         histBin.push_back(result[j]);
    //     }
    //     hist.first = listDataSet[i];
    //     hist.second = histBin;
    //     resultDataSet.push_back(hist);
    // }

    // for ( auto it : resultDataSet)
    // {
    //     std::vector<std::string> resultsss;
    //     boost::algorithm::split(resultsss, it.first, boost::is_any_of("/"));
    //     std::cout << resultsss[1] << std::endl;
    //     it.first = resultsss[1];
    //     for(auto it2 : it.second)
    //     {
    //         std::cout << it2 << " " ;
    //     }
    //     std::cout << std::endl;
    //     std::cout << std::distance(it.second.begin(), std::max_element(it.second.begin(), it.second.end())) << std::endl;
    // }
*/
    

    // Read in the cloud data
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    reader.read (argv[1], *cloud);
    std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl; //*


    // Filter Removing NaN data Pointcloud.
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, mapping);
    std::cerr << "Pointcloud after remove NaN : " << cloud->size() << " data points." << std::endl;

    // Filter object crop by z coordinate
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 2.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_passthrough);
    std::cerr << "Pointcloud after cropped : " << cloud_passthrough->size() << " data points." << std::endl;
    //save passthrough filter
    pcl::io::savePCDFile("src/object_recognition_pkg/output/filter/cropping.pcd", *cloud_passthrough);


    // Filter downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud_passthrough);
    vg.setLeafSize (0.01f, 0.01f, 0.01f); // 1cm
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*
    //save the result of downsample voxel grid
    pcl::io::savePCDFile("src/object_recognition_pkg/output/filter/voxel_grid.pcd", *cloud_filtered);
    


    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    int i=0, nr_points = (int) cloud_filtered->size ();
    while (cloud_filtered->size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;


        //save planar [optional]
        std::stringstream ssPlanar;
        ssPlanar << "src/object_recognition_pkg/output/segmentation/planar" << i <<".pcd";
        writer.write<pcl::PointXYZ> (ssPlanar.str (), *cloud_plane, false); //*

        //save planar removed[optional]
        std::stringstream ssPlanarRemoved;
        ssPlanarRemoved << "src/object_recognition_pkg/output/segmentation/planar_removed" << i <<".pcd";
        writer.write<pcl::PointXYZ> (ssPlanarRemoved.str (), *cloud_f, false); //*

        i++;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    //// get the cluster models
    i = 0;
    std::vector<modelsDetail> dataClustered;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->push_back ((*cloud_filtered)[*pit]); //*
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;

        modelsDetail modelClustered;
        modelClustered.second = cloud_cluster;
        modelClustered.first.no = i; 
        dataClustered.push_back (modelClustered);

        //save file of clustered object
        std::stringstream ss;
        ss << "src/object_recognition_pkg/output/cluster/cloud_cluster_" << i << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
        i++;
    }


    //// VFH DESCRIPTOR  
    std::vector<modelsDetail> dataAddVFH;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    typedef pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> VFHEstimationType;
    VFHEstimationType vfhEstimation;
    i = 0;
    for(auto it : dataClustered )
    {
        //  Compute the normals
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        normalEstimation.setInputCloud (it.second);
        normalEstimation.setSearchMethod (tree);
        normalEstimation.setRadiusSearch (0.03);
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
        // std::cout << "output points.size (): " << vfhFeatures->points.size () << std::endl; // This outputs 1 - should be 397!

        // Display and retrieve the shape context descriptor vector for the 0th point.
        // pcl::VFHSignature308 descriptor = vfhFeatures->points[0];
        // VFHEstimationType::PointCloudOut::PointType descriptor2 = vfhFeatures->points[0];
        // std::cout << descriptor << std::endl;

        // save histogram VFH of clustered object
        std::stringstream ss;
        ss << "src/object_recognition_pkg/output/vfh/vfh_" << i << ".pcd";
        writer.write<pcl::VFHSignature308> (ss.str (), *vfhFeatures, false); //*
        i++;

        //push to vector 
        std::vector<float> VFHValue(
            vfhFeatures->points[0].histogram,
            vfhFeatures->points[0].histogram + 308 
        );

        it.first.histogram = VFHValue;
        dataAddVFH.push_back(it);

        /** backup code before
        // modelsVFH.push_back (vfhFeatures);

        // VFHValues.push_back(vfhFeatures->points[0].histogram);
        // std::vector<float> VFHValue;
        // float vfh[308] = vfhFeatures->points[0].histogram;
        // std::cout << "size historgam : " << it.first.histogram.size() << std::endl;

        // for(auto it2 : it.first.histogram)
        // {
        //     std::cout << it2 << " ";
        //     // it.first.histogram;
        // }
        // std::cout << std::endl;

            // std::end(vfhFeatures->points[0].histogram) - std::end(vfhFeatures->points[0].histogram)
        // for(auto it : vfhFeatures->points[0].histogram)
        // {
        //     // std::cout << it << std::endl;
        //     VFHValue.push_back(it);
        // }
        // VFHValues.push_back(VFHValue);
        */
    }
    // std::cout << "jumlah vfh : " << VFHValues.size() << std::endl;


    // ARTIFICIAL NEURAL NETOWRK
    struct fann *ann = fann_create_from_file("bbbbbbb.net"); // generated from training
    fann_type *calc_out;
    fann_type input[308]; //length of VFH Descriptor
    for(auto it : dataAddVFH)
    {
        std::copy(it.first.histogram.begin(), it.first.histogram.end(), input);

        calc_out = fann_run(ann, input);

        for(int i = 0 ; i < ann->num_output ; i++)
        {
            std::cout << calc_out[i]  << " " ;//<< std::endl;
        }
        std::cout << std::endl;
        
    }
    fann_destroy(ann);


    return (0);
}