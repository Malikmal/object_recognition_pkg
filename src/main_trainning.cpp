#include <iostream>
#include <vector>
#include <array>
#include <algorithm>
#include <fstream>

#include <boost/filesystem.hpp>

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

// argument 1 => folder dataset ex : dataset_washington 
// contain list folder of file segmented model .pcd  


struct info{
  int no;
  std::string category;
  std::string filename;
} ;

typedef std::pair<info, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> modelRaw;


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
      reader.read (it->path ().string(), *cloud);

      info dataInfo;
      dataInfo.no  = i;
      dataInfo.category = base_dir.string();
      dataInfo.filename = it->path().filename().string();
      modelRaw m(dataInfo, cloud);

      models.push_back(m);
      // std::cout << "base_dir << '/' "<< it->path ().filename ().string() << std::endl;
      std::cout << dataInfo.no << ". reading " << dataInfo.category << " file : "<< dataInfo.filename << std::endl;
      
    } 
  }
}

int main (int argc, char** argv)
{
    // std::vector<modelRaw> models;
    // std::string extension (".pcd");
    // transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);

    // LoadModels(argv[1], extension, models); // argv[1] = "dataset_washington" 

    // std::cout << "file readed : " << models.size() << std::endl;

    // ofstream MyFileData("listDataSet.txt");
    // int no  = 0;
    // for(auto it : models)
    // {
    //    if(no != it.first.no)
    //       MyFileData << it.first.no << "#" << it.first.category << std::endl;
    //     no = it.first.no;
    // }
    // MyFileData.close();


    // //// VFH DESCRIPTOR
    // // std::vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> modelsVFH;
    // std::vector<std::vector<float> > VFHValues;

    // //write file for data trainninng FANN librray format
    // ofstream MyFile("listDataSet.data");
    // MyFile << models.size() << " 308 1" << std::endl; // coutn of row, count of input node ann, count of output node ann
    
    // pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
    // typedef pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> VFHEstimationType;
    // VFHEstimationType vfhEstimation;

    // // for(std::size_t i = 0; i < models.size(); ++i )
    // for(auto it : models)
    // {

    //     // std::cout << it.size() << std::endl;

    //     //  Compute the normals
    //     normalEstimation.setInputCloud (it.second);

    //     pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    //     normalEstimation.setSearchMethod (tree);


    //     normalEstimation.setRadiusSearch (0.03);

    //     pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);
    //     normalEstimation.compute (*cloudWithNormals);

    //     // std::cout << "Computed " << cloudWithNormals->points.size() << " normals." << std::endl;
        
    //     // Setup the feature computation

    //     // Provide the original point cloud (without normals)
    //     vfhEstimation.setInputCloud (it.second);

    //     // Provide the point cloud with normals
    //     vfhEstimation.setInputNormals(cloudWithNormals);

    //     // Use the same KdTree from the normal estimation
    //     vfhEstimation.setSearchMethod (tree);

    //     //vfhEstimation.setRadiusSearch (0.2); // With this, error: "Both radius (.2) and K (1) defined! Set one of them to zero first and then re-run compute()"

    //     // Actually compute the VFH features
    //     pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhFeatures(new pcl::PointCloud<pcl::VFHSignature308>);
    //     vfhEstimation.compute (*vfhFeatures);

    //     // std::cout << "output points.size (): " << vfhFeatures->points.size () << std::endl; // This outputs 1 - should be 397!

    //     // Display and retrieve the shape context descriptor vector for the 0th point.
    //     // pcl::VFHSignature308 descriptor = vfhFeatures->points[0];
    //     // VFHEstimationType::PointCloudOut::PointType descriptor2 = vfhFeatures->points[0];
    //     // std::cout << descriptor << std::endl;

    //     //push to vector 
    //     // modelsVFH.push_back (vfhFeatures);

    //     // VFHValues.push_back(vfhFeatures->points[0].histogram);
    //     // std::vector<float> VFHValues;
    //     // float vfh[308] = vfhFeatures->points[0].histogram;

    //     std::vector<float> VFHValue;
    //     for(auto it2 : vfhFeatures->points[0].histogram)
    //     {
    //         // std::cout << it2 << std::endl;
    //         VFHValue.push_back(it2);
    //         MyFile << it2 << " " ;
    //     }
    //     MyFile << std::endl;
    //     MyFile << it.first.no << std::endl;
    //     VFHValues.push_back(VFHValue);


    //     // std::cout << it.first.filename << " vfh calculated : " << std::endl;// << VFHValue.size();
    //   std::cout << it.first.no << ". VFH calculated " << it.first.category << " of file : "<< it.first.filename << std::endl;
    // }
    
    // // Close the file
    // MyFile.close();
    // std::cout << "jumlah data vfh : " << VFHValues.size();






    // ARTIFICIAL NEURAL NETOWRK
    const unsigned int num_input = 308;
    const unsigned int num_output = 5;
    const unsigned int num_layers = 4;
    const unsigned int num_neurons_hidden = 308; 
    const float desired_error = (const float) 0.0001; // break ketika error sudah lebih kecil dari ini
    const unsigned int max_epochs = 100;//50000; //epoch iterasi
    const unsigned int epochs_between_reports = 1; //jeda printing 

    struct fann *ann = fann_create_standard(
        num_layers, 
        num_input,
        num_neurons_hidden, 
        (int) num_neurons_hidden/2,
        num_output
    );

    // fann_set_activation_function_hidden(ann, FANN_LINEAR); // set act func all hidden layer
    fann_set_activation_function_layer(ann, FANN_LINEAR_PIECE, 1); //hidden layer ke 1 
    fann_set_activation_function_layer(ann, FANN_LINEAR_PIECE, 2); //hidden layer ke 2 
    // fann_set_activation_function_layer(ann, FANN_LINEAR_PIECE, 3); //hidden layer ke 3
    // fann_set_activation_function_layer(ann, FANN_SIGMOID, 4); //hidden layer ke 4
    // fann_set_activation_function_layer(ann, FANN_SIGMOID, 5); //hidden layer ke 5
    fann_set_activation_function_output(ann, FANN_SIGMOID);

    // fann_set_train_error_function(ann, FANN_ERRORFUNC_LINEAR);
    // fann_set_train_stop_function(ann, FANN_STOPFUNC_MSE);

    // fann_set_training_algorithm(ann, FANN_TRAIN_BATCH);

    // fann_set_learning_rate(ann, 15);
    // fann_reset_MSE(ann);
    // // fann_type a = 0.01;
    // // fann_scale_input(ann, &a);

    // fann_randomize_weights(ann, 0.0, 1.0);
    

    fann_train_on_file(ann, "listDataSet.data", max_epochs,
        epochs_between_reports, desired_error);
    /***still cant work**/
    // fann_type input[1], *desired_output;
    // input[0] = 5;
    // // desired_output[0] = 7.1414;
    // desired_output[0] = (const float) 7.1414;
    // fann_test(ann, input, desired_output);
    // printf("hasil test : %f\n", desired_output);
    /****/

    fann_save(ann, "listDataSet.net");

    // fann_print_connections(ann);

    fann_destroy(ann); 

    return (0);
}