#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include<pcl/visualization/histogram_visualizer.h>

int main (int argc, char** argv)
{
  std::string fileName = argv[1];
  std::cout << "Reading " << fileName << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file");
    return (-1);
  }

  std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

  // Compute the normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud (cloud);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  normalEstimation.setSearchMethod (tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);

  normalEstimation.setRadiusSearch (0.03);

  normalEstimation.compute (*cloudWithNormals);

  std::cout << "Computed " << cloudWithNormals->points.size() << " normals." << std::endl;
  
  // Setup the feature computation
  typedef pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> VFHEstimationType;
  VFHEstimationType vfhEstimation;

  // Provide the original point cloud (without normals)
  vfhEstimation.setInputCloud (cloud);

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
//   pcl::VFHSignature308 descriptor = vfhFeatures->points[0];
//   VFHEstimationType::PointCloudOut::PointType descriptor2 = vfhFeatures->points[0];
//   std::cout << vfhFeatures->points[0] << std::endl;
//   std::cout << vfhFeatures->points[0].histogram[30] << std::endl;

    // std::cout << pcl::VFHSignature308.escriptorSize() << std::endl;

    // std::vector<float> VFHValue;
    // for(auto it : vfhFeatures->points[0].histogram)
    // {
    //     // std::cout << it << std::endl;
    //     VFHValue.push_back(it);
    // }

    // for(auto it : VFHValue)
    // {
    //     std::cout << it << std::endl;
    //     // VFHValue.push_back(it);
    // }

    


//   // //ascii
//   std::string extension (".pcd");

// //   std::stringstream ss;
// //   ss << "src/object_recognition_pkg/output_vfh/" << fileName << ".pcd";
//   transform (fileName.begin (), fileName.end (), fileName.begin (), (int(*)(int))tolower);
//   std::cout << fileName.str() << std::endl;
// //   pcl::io::savePCDFile(ss.str(), *vfhFeatures);

// //   std::string ss = "src/object_recognition_pkg/ouput_vfh/";
// //   ss += fileName;
// //   std::cout << ss << std::endl;
// //   pcl::io::savePCDFile(ss, *vfhFeatures);
// //   pcl::PCDWriter writer;
// //     writer.write(ss.str (), *vfhFeatures, false); //*
//   std::cerr << "Saved in ASCII Fromat ." << std::endl;


	// Plotter object.
	pcl::visualization::PCLHistogramVisualizer viewer;
	// We need to set the size of the descriptor beforehand.
  viewer.addFeatureHistogram(*vfhFeatures, 308);
  // viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(vfhFeatures, normals);

	viewer.spin();

  return 0;
}