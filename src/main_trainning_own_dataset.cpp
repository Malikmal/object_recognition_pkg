#include <iostream>
#include <vector>
#include <array>
#include <algorithm>
#include <fstream>
#include <thread>

#include <boost/filesystem.hpp>

#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>


#include "fann.h"
#include "floatfann.h"
#include "fann_data.h"
// #include "parallel_fann.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
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

// argument 1 => folder dataset ex : dataset_washington 
// contain list folder of file segmented model .pcd  


struct info{
  int no;
  std::string category;
  std::string filename;
} ;

typedef std::pair<info, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> modelRaw;
typedef pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> VFHEstimationType;

   

//  fann_callback_type callbackTrainning(fann_train_data ann)
int FANN_API callbackTrainning(
    struct fann *ann,
    struct fann_train_data *train, 
    unsigned int max_epochs, 
    unsigned int epochs_between_reports, 
    float desired_error, 
    unsigned int epochs
)
{
    ofstream MyFileData("listOwnDataSetv-0.1_error.txt", ios::app);
    MyFileData << epochs << ", " 
               << fann_get_MSE(ann) << ", " 
               << desired_error << ", "
               << std::endl;
    std::cout  << epochs << ", " 
               << fann_get_MSE(ann) << ", " 
               << desired_error << ", "
               << std::endl;
    MyFileData.close();
}

// /****************** Filter & Segmentation **************************/
void filtering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_filtered){


  // Filter Removing NaN data Pointcloud.
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*cloud, *cloud_filtered, mapping);
  std::cerr << "Pointcloud remove NaN : " << cloud_filtered->size() << " data points." << std::endl;

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);


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
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component" << i << "th : " << cloud_plane->size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_filtered);

    i++;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clustered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  std::cout << "cloudClustered : " << cloud_clustered->size() << std::endl;

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->push_back ((*cloud_filtered)[*pit]); //*
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    // take the bigger size point that can be conclusion is main object
    if(cloud_clustered->size() < cloud_cluster->size())
      *cloud_clustered = *cloud_cluster;

    std::cout << "PointCloud representing the Cluster " << j << " th : " << cloud_cluster->size () << " data points." << std::endl;
    

    j++;
  }

  *cloud_filtered = *cloud_clustered;

}

// /****************** Extrack feature (descriptor) **************************/
void descriptoring(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_filtered, pcl::PointCloud<pcl::VFHSignature308>::Ptr &vfhFeatures)
{
    // // VFH DESCRIPTOR
    // // std::vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> modelsVFH;
    // std::vector<std::vector<float> > VFHValues;

    // //write file for data trainninng FANN librray format
    // ofstream MyFile("listOwnDataSetv0.1.data");
    // MyFile << models.size() << " 308 " << category.size() << std::endl; // coutn of row, count of input node ann, count of output node ann
    
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
    // typedef pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> VFHEstimationType;
    VFHEstimationType vfhEstimation;

    // std::cout << it.size() << std::endl;

    //  Compute the normals
    normalEstimation.setInputCloud (cloud_filtered);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    normalEstimation.setSearchMethod (tree);

    normalEstimation.setRadiusSearch (0.01); //0.03

    pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);
    normalEstimation.compute (*cloudWithNormals);

    std::cout << "Computed " << cloudWithNormals->points.size() << " normals." << std::endl;
    
    // Setup the feature computation

    // Provide the original point cloud (without normals)
    vfhEstimation.setInputCloud (cloud_filtered);

    // Provide the point cloud with normals
    vfhEstimation.setInputNormals(cloudWithNormals);

    // Use the same KdTree from the normal estimation
    vfhEstimation.setSearchMethod (tree);

    // vfhEstimation.setRadiusSearch (0.2); //nothing effect // With this, error: "Both radius (.2) and K (1) defined! Set one of them to zero first and then re-run compute()"

    // Actually compute the VFH features
    pcl::PointCloud<pcl::VFHSignature308>::Ptr tmpVfhFeatures(new pcl::PointCloud<pcl::VFHSignature308>);
    vfhEstimation.compute (*tmpVfhFeatures);

    *vfhFeatures = *tmpVfhFeatures;

    // std::cout << "output points.size (): " << vfhFeatures->points.size () << std::endl; // This outputs 1 - should be 397!

    // Display and retrieve the shape context descriptor vector for the 0th point.
    // pcl::VFHSignature308 descriptor = vfhFeatures->points[0];
    // VFHEstimationType::PointCloudOut::PointType descriptor2 = vfhFeatures->points[0];
    // std::cout << descriptor << std::endl;

    //push to vector 
    // modelsVFH.push_back (vfhFeatures);

    // VFHValues.push_back(vfhFeatures->points[0].histogram);
    // std::vector<float> VFHValues;
    // float vfh[308] = vfhFeatures->points[0].histogram;

    std::cout  << " vfh calculated : "  << std::endl;// << VFHValue.size();
     for(auto it2 : vfhFeatures->points[0].histogram)
      {
          std::cout << it2 << " " ;
          // VFHValue.push_back(it2);
          // MyFile << it2 << " " ;
      }
    std::cout  << " end of vfh calculated : "  << std::endl;
  // std::cout << it.first.no << ". VFH calculated " << it.first.category << " of file : "<< it.first.filename << std::endl;
}


void LoadModels (const boost::filesystem::path &base_dir, const std::string &extension, std::vector<modelRaw> &models)
{
  static int i = 1;
  if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
    return;

  for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
  {
    if (boost::filesystem::is_directory (it->status ()))
    {
      i++;
      std::stringstream ss;
      ss << it->path ();
      pcl::console::print_highlight ("Loading %s (%lu models loaded so far).\n", ss.str ().c_str (), (unsigned long)models.size ());
      LoadModels (it->path (), extension, models);
    }
    if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
    {
      pcl::PCDReader reader;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
      reader.read (it->path ().string(), *cloud);


      info dataInfo;
      dataInfo.no  = i;
      dataInfo.category = base_dir.string();
      dataInfo.filename = it->path().filename().string();
      modelRaw m(dataInfo, cloud);

      models.push_back(m);
      std::cout << "base_dir << '/' "<< it->path ().filename ().string() << std::endl;
      std::cout << dataInfo.no << ". reading " << dataInfo.category << " file : "<< dataInfo.filename << std::endl;

      /** visualize each model
      pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
      // viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "sample cloud");

      int v1(0);
      viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
      viewer->setBackgroundColor (0, 0, 0, v1);
      viewer->addText("original", 10, 10, "v1 text", v1);
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_cloud(cloud);
      viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb_cloud, "original", v1);
      **/

      filtering(cloud, cloud_filtered);
      // viewer->addPointCloud<pcl::PointXYZRGB> (cloud_filtered, "sample cloud");

      /** visualize each model after segmentation
      int v2(0);
      viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
      viewer->setBackgroundColor (0, 0, 0, v2);
      viewer->addText("filtered and segmented", 10, 10, "v2 text", v2);
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_cloud_filtered(cloud_filtered);
      viewer->addPointCloud<pcl::PointXYZRGB> (cloud_filtered, rgb_cloud_filtered, "filtered and segmented", v2);

      pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhFeatures1(new pcl::PointCloud<pcl::VFHSignature308>);

      viewer->addCoordinateSystem (1.0);


	    // pcl::visualization::PCLHistogramVisualizer viewerHistogram;
      // viewerHistogram.addFeatureHistogram(*vfhFeatures1, 308);
      // // viewerHistogram.updateFeatureHistogram(*vfhFeatures, 308);

      // viewerHistogram.spinOnce();

      while(!viewer->wasStopped())
      {
        viewer->spinOnce (100);
        // std::this_thread::sleep_for(100ms);
      }
      **/

      pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhFeatures(new pcl::PointCloud<pcl::VFHSignature308>);
      descriptoring(cloud_filtered, vfhFeatures);
std::cout  << " vfh calculated ted : " << vfhFeatures->size() << std::endl;// << VFHValue.size();

      

    } 
  }
}

int main (int argc, char** argv)
{
    std::vector<modelRaw> models;
    std::string extension (".pcd");
    transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);

    LoadModels(argv[1], extension, models); // argv[1] = "dataset_washington" 

    std::cout << "file readed : " << models.size() << std::endl;

    ofstream MyFileData("listOwnDataSetv0.1.txt");
    int no  = 0;
    std::vector<int> category;
    for(auto it : models)
    {
       if(no != it.first.no)
       {
          MyFileData << it.first.category << std::endl;
          category.push_back(it.first.no);
       }
        no = it.first.no;
    }
    MyFileData.close();

    
    //  // ARTIFICIAL NEURAL NETOWRK
    // fann_type *calc_out;
    // const unsigned int num_input = 308;
    // const unsigned int num_output = 8;
    // const unsigned int num_layers = 3;
    // const unsigned int num_neurons_hidden = 308;
    // const float desired_error = (const float) 0;
    // const unsigned int max_epochs = 2000;
    // const unsigned int epochs_between_reports = 1;
    // struct fann *ann;
    // struct fann_train_data *data;

    // unsigned int i = 0;
    // unsigned int decimal_point;

    // printf("Creating network.\n");
    // // ann = fann_create_standard(num_layers, num_input, num_neurons_hidden, num_output);
    // ann = fann_create_standard(5, 308, 616, 308, 64, 8); // bbbbbbb
    // // ann = fann_create_standard(5, 308, 616, 308, 64, 8); // ccccccc


    // data = fann_read_train_from_file("listOwnDataSetv0.1.data");

    // fann_set_activation_steepness_hidden(ann, 0.01); //deafault 0.5 //bbbbbb = 0.01
    // fann_set_activation_steepness_output(ann, 0.01); //deafault 0.5

    // // fann_scale_input_train_data(data, (fann_type) (0), (fann_type) 100);
    // // fann_scale_output_train_data(data, (fann_type) (0), (fann_type) 100);

    // fann_set_activation_function_hidden(ann, FANN_SIGMOID_STEPWISE);
    // fann_set_activation_function_output(ann, FANN_SIGMOID_STEPWISE);


    // fann_set_train_stop_function(ann, FANN_STOPFUNC_BIT);
    // // fann_set_bit_fail_limit(ann, 0.01f); //default 0.35

    // fann_set_training_algorithm(ann, FANN_TRAIN_RPROP); //defailt FANN_TRAIN_RPROP

    // fann_init_weights(ann, data);


    // fann_set_callback(ann, (fann_callback_type)&callbackTrainning); //callback to save error

    // printf("Training network.\n");
    // fann_train_on_data(ann, data, max_epochs, epochs_between_reports, desired_error);

    // // float error;
    // // ofstream MyFileData("ccccccc_error.txt", ios::app);
    // // for(i = 1; i <= max_epochs; i++)
    // // {
    // // error = fann_train_epoch_irpropm_parallel(ann, data, 4);
    // // 	printf("Epochs     %8d. Current error: %.10f. Bit Fail: %5d\n", i, error, fann_get_bit_fail(ann));
    // //     MyFileData << i << "#" 
    // //             << error << "#"
    // //             << fann_get_bit_fail(ann) << "#"
    // //             << std::endl;
    // // }
    // // MyFileData.close();

    // printf("Testing network. %f\n", fann_test_data(ann, data));


    // for(i = 0; i < fann_length_train_data(data); i++)
    // {
    // calc_out = fann_run(ann, data->input[i]);
    // printf("test (%f,%f) -> %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f \n, should be %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f , difference=%f\n",
    //       data->input[i][0], data->input[i][1], 
    //             calc_out[0], calc_out[1], calc_out[2], calc_out[3], calc_out[4], calc_out[5], calc_out[6], calc_out[7], 
    //             data->output[i][0], data->output[i][1], data->output[i][2], data->output[i][3], data->output[i][4], data->output[i][5], data->output[i][6], data->output[i][7], 
    //       fann_abs(calc_out[0] - data->output[i][0]));
        

    // }

    // printf("Saving network.\n");

    // fann_save(ann, "listOwnDataSetv0.1.net");

    // decimal_point = fann_save_to_fixed(ann, "listOwnDataSetv0.1_fixed.net");
    // fann_save_train_to_fixed(data, "listOwnDataSetv0.1fixed.data", decimal_point);

    // printf("Cleaning up.\n");
    // fann_destroy_train(data);
    // fann_destroy(ann);


    return (0);
}


