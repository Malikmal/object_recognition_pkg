#include <iostream>
#include <vector>
#include <algorithm>

#include "fann.h"
#include "floatfann.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
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
    // Read in the cloud data
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    reader.read (argv[1], *cloud);
    std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl; //*

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*

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
    //   int j = 0;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> models;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->push_back ((*cloud_filtered)[*pit]); //*
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;

        models.push_back (cloud_cluster);
        // std::stringstream ss;
        // ss << "src/object_recognition_pkg/output/cloud_cluster_" << j << ".pcd";
        // writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
        // j++;
    }


    //// VFH DESCRIPTOR
    // std::vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> modelsVFH;
    std::vector<std::vector<float> > VFHValues;
    
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
        typedef pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> VFHEstimationType;
        VFHEstimationType vfhEstimation;

    for(auto it : models )
    {
        //  Compute the normals
        normalEstimation.setInputCloud (it);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        normalEstimation.setSearchMethod (tree);


        normalEstimation.setRadiusSearch (0.03);

        pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);
        normalEstimation.compute (*cloudWithNormals);

        std::cout << "Computed " << cloudWithNormals->points.size() << " normals." << std::endl;
        
        // Setup the feature computation

        // Provide the original point cloud (without normals)
        vfhEstimation.setInputCloud (it);

        // Provide the point cloud with normals
        vfhEstimation.setInputNormals(cloudWithNormals);

        // Use the same KdTree from the normal estimation
        vfhEstimation.setSearchMethod (tree);

        //vfhEstimation.setRadiusSearch (0.2); // With this, error: "Both radius (.2) and K (1) defined! Set one of them to zero first and then re-run compute()"

        // Actually compute the VFH features
        pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhFeatures(new pcl::PointCloud<pcl::VFHSignature308>);
        vfhEstimation.compute (*vfhFeatures);

        std::cout << "output points.size (): " << vfhFeatures->points.size () << std::endl; // This outputs 1 - should be 397!

        // Display and retrieve the shape context descriptor vector for the 0th point.
        // pcl::VFHSignature308 descriptor = vfhFeatures->points[0];
        // VFHEstimationType::PointCloudOut::PointType descriptor2 = vfhFeatures->points[0];
        // std::cout << descriptor << std::endl;

        //push to vector 
        // modelsVFH.push_back (vfhFeatures);

        // VFHValues.push_back(vfhFeatures->points[0].histogram);
        // std::vector<float> VFHValue;
        // float vfh[308] = vfhFeatures->points[0].histogram;

        std::vector<float> VFHValue(
            vfhFeatures->points[0].histogram,
            vfhFeatures->points[0].histogram + 308 
        );
            // std::end(vfhFeatures->points[0].histogram) - std::end(vfhFeatures->points[0].histogram)
        // for(auto it : vfhFeatures->points[0].histogram)
        // {
        //     // std::cout << it << std::endl;
        //     VFHValue.push_back(it);
        // }
        VFHValues.push_back(VFHValue);
    }
    
    std::cout << "jumlah vfh : " << VFHValues.size() << std::endl;


    // ARTIFICIAL NEURAL NETOWRK


    for(auto it : VFHValues)
    {
        struct fann *ann = fann_create_from_file("listDataSet.net"); // generated from training
        fann_type *calc_out;
        fann_type input[308]; //length of VFH Descriptor
        std::copy(it.begin(), it.end(), input);
        // for(int  i = 0; i < 308; i++)
        // {
        //     // input[i] = it[i];
        //     std::cout << input[i] << " ";
        // }
        // std::cout << std::endl;
        calc_out = fann_run(ann, input);
        // fann_type test[5] = {1, 0, 0, 0, 0};
        // calc_out = fann_test(ann, input, test);
        // printf("output test (%f) (%f) (%f) (%f) (%f) \n", 
        //     calc_out[0], calc_out[1], calc_out[2], calc_out[3], calc_out[4]);
        for(int i = 0 ; i < 5 ; i++)
        {
            std::cout << calc_out[i]  << " " ;//<< std::endl;
        }
        std::cout << std::endl;
        fann_destroy(ann);
    }
    // std::cout << calc_out[9] << std::endl;

    return (0);
}