#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// argument 1 => file ex : scene_mug_table.pcd 

int 
main (int argc, char** argv)
{
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PCDWriter writer;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read (argv[1], *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl; //*

 /* cropping alternative
  // pcl::PointCloud<pcl::PointXYZ>::Ptr bodyFiltered(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::CropBox<pcl::PointXYZ> boxFilter;
  // boxFilter.setMin(Eigen::Vector4f(-30.0f, -30.0f, -10.0f, 1.0f)); //x,y,z,w
  // boxFilter.setMax(Eigen::Vector4f(30.0f, 30.0f, 2.0f, 1.0f)); // 1.0f is 1 meter
  // boxFilter.setInputCloud(cloud);
  // boxFilter.filter(*bodyFiltered);
*/

  // Filter object crop by z coordinate
  pcl::PointCloud<pcl::PointXYZ>::Ptr bodyFiltered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 2.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*bodyFiltered);
  //save passthrough filter
  pcl::io::savePCDFile("src/object_recognition_pkg/output/filter/cropping.pcd", *bodyFiltered);
  std::cerr << "Pointcloud after cropped : " << bodyFiltered->size() << " data points." << std::endl;

  // Filter Removing NaN data Pointcloud.
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*bodyFiltered, *bodyFiltered, mapping);

/*Filter Outlier Removal
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (bodyFiltered);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  //save outlier removal filter
  // sor.setNegative(true); // get inlier
  sor.filter (*bodyFiltered);
  pcl::io::savePCDFile("src/object_recognition_pkg/output/filter/outlier.pcd", *bodyFiltered);
  std::cerr << "Pointcloud inlier : " << bodyFiltered->size() << " data points." << std::endl;
  sor.setNegative(true); // get outlier
  sor.filter (*bodyFiltered);
  pcl::io::savePCDFile("src/object_recognition_pkg/output/filter/inlier.pcd", *bodyFiltered);
  std::cerr << "Pointcloud outlier : " << bodyFiltered->size() << " data points." << std::endl;
*/

    

  // Filter downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (bodyFiltered);
  vg.setLeafSize (0.01f, 0.01f, 0.01f); //1cm
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*
  //save the result of downsample voxel grid
  std::stringstream ss;
  ss << "src/object_recognition_pkg/output/filter/voxel_grid.pcd";
  writer.write<pcl::PointXYZ> (ss.str (), *cloud_filtered, false); //*


  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

/* once
  // int i=0, nr_points = (int) cloud_plane->size();
  // seg.setInputCloud(cloud_filtered);
  // seg.segment(*inliers, *coefficients);
  // if(inliers->indices.size() == 0){
  //     std::cout << "Couldn't estimate a plannar model for given dataset!!! \n" << std::endl;
  //     return(-1);
  // }

  // pcl::ExtractIndices<pcl::PointXYZ> extract;
  // extract.setInputCloud(cloud_filtered);
  // extract.setIndices(inliers);
  // extract.setNegative(false);

  // extract.filter(*cloud_plane);
  // std::cout << "Point Cloud Representing the Planar: " << cloud_plane->size() << " data points." << std::endl;

  // pcl::io::savePCDFileASCII ("src/object_recognition_pkg/output/segmentation/setNegative_false.pcd", *cloud_plane);

  // extract.setNegative(true);
  // extract.filter(*cloud_f);

  // pcl::io::savePCDFileASCII ("src/object_recognition_pkg/output/segmentation/setNegative_true.pcd", *cloud_f);
*/

/* loop */
  int i=0, nr_points = (int) cloud_filtered->size ();
  while (cloud_filtered->size () > 0.4 * nr_points)
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

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->push_back ((*cloud_filtered)[*pit]); //*
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;

    std::stringstream ssClustered;
    ssClustered << "src/object_recognition_pkg/output/cluster/cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ssClustered.str (), *cloud_cluster, false); //*
    j++;
  }

  return (0);
}