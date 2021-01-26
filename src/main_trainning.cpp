#include <iostream>
#include <vector>
#include <array>
#include <algorithm>
#include <fstream>

#include <boost/filesystem.hpp>

#include "fann.h"
#include "floatfann.h"
#include "fann_data.h"
#include "parallel_fann.h"

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
    ofstream MyFileData("errorv3.1.txt", ios::app);
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

int main (int argc, char** argv)
{
    // std::vector<modelRaw> models;
    // std::string extension (".pcd");
    // transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);

    // LoadModels(argv[1], extension, models); // argv[1] = "dataset_washington" 

    // std::cout << "file readed : " << models.size() << std::endl;

    // ofstream MyFileData("listDataSetv2.1.txt");
    // int no  = 0;
    // std::vector<int> category;
    // for(auto it : models)
    // {
    //    if(no != it.first.no)
    //    {
    //       MyFileData << it.first.category << std::endl;
    //       category.push_back(it.first.no);
    //    }
    //     no = it.first.no;
    // }
    // MyFileData.close();


    // //// VFH DESCRIPTOR
    // // std::vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> modelsVFH;
    // std::vector<std::vector<float> > VFHValues;

    // //write file for data trainninng FANN librray format
    // ofstream MyFile("listDataSetv2.1.data");
    // MyFile << models.size() << " 308 " << category.size() << std::endl; // coutn of row, count of input node ann, count of output node ann
    
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
    //     // int i = 0;
    //     for(auto it2 : category)
    //     {
    //       if(it2 == it.first.no)
    //           MyFile << 1 ;      
    //       else
    //           MyFile << 0 ;
    //       MyFile << " "; 
    //     }
    //     MyFile << std::endl;
    //     // MyFile << it.first.no << " " << std::endl;
    //     VFHValues.push_back(VFHValue);


    //     // std::cout << it.first.filename << " vfh calculated : " << std::endl;// << VFHValue.size();
    //   std::cout << it.first.no << ". VFH calculated " << it.first.category << " of file : "<< it.first.filename << std::endl;
    // }
    
    // // Close the file
    // MyFile.close();
    // std::cout << "jumlah data vfh : " << VFHValues.size();






     // ARTIFICIAL NEURAL NETOWRK
    const unsigned int num_input = 308;
    const unsigned int num_output = 8; //according to count category
    const unsigned int num_layers = 4;
    const unsigned int num_neurons_hidden = 308; 
    const float desired_error = (const float) 0.01; // break ketika error sudah lebih kecil dari ini
    const unsigned int max_epochs = 200;//50000; //epoch iterasi
    const unsigned int epochs_between_reports = 1; //jeda printing 

    struct fann *ann = fann_create_standard(
        9,    //num_layers, 
        308,  //num_input,
        308*2,  // num_neurons_hidden, 
        308*2,  // num_neurons_hidden, 
        308,  // num_neurons_hidden, 
        154,  // (int) num_neurons_hidden/2,
        77,   // (int) num_neurons_hidden/4,
        38,   // (int) num_neurons_hidden/8,
        19,   // (int) num_neurons_hidden/32,
        num_output //8
    );

    // fann_set_activation_function_hidden(ann, FANN_LINEAR); // set act func all hidden layer
    // fann_set_activation_function_layer(ann, FANN_LINEAR_PIECE, 1); //hidden layer ke 1 
    // fann_set_activation_function_layer(ann, FANN_LINEAR_PIECE, 2); //hidden layer ke 2 
    // fann_set_activation_function_layer(ann, FANN_LINEAR_PIECE, 3); //hidden layer ke 3
    // fann_set_activation_function_layer(ann, FANN_SIGMOID, 4); //hidden layer ke 4
    // fann_set_activation_function_layer(ann, FANN_SIGMOID, 5); //hidden layer ke 5
    // fann_set_activation_function_layer(ann, FANN_SIGMOID, 6); //hidden layer ke 6
    // fann_set_activation_function_layer(ann, FANN_SIGMOID, 7); //hidden layer ke 7
    // fann_set_activation_function_output(ann, FANN_SIGMOID);


    // fann_set_train_error_function(ann, FANN_ERRORFUNC_LINEAR); //The default error function is FANN_ERRORFUNC_TANH
    // fann_set_train_stop_function(ann, FANN_STOPFUNC_MSE); //The default stop function is FANN_STOPFUNC_MSE
    // fann_set_training_algorithm(ann, FANN_TRAIN_BATCH); // deafult FANN_TRAIN_RPROP
    // fann_set_learning_rate(ann, 10.7); //deafult fann 0.7
    // fann_randomize_weights(ann, 0.0, 5.0);
    // fann_reset_MSE(ann);
    
    fann_set_callback(ann, (fann_callback_type)&callbackTrainning); //callback to save error

    // fann_type* weights;
    // fann_get_weights(ann, weights );
    // // for(int i =0; i < sizeof(weights)/sizeof(weights[0]) ; i++)
    // // {
    // //   std::cout << " ";// weights[i] << " ";
    // // }
    // std::cout << sizeof(&weights[0]) << std::endl;

    // print_fann_configuration(ann); //handmade   


    fann_train_on_file(ann, "listDataSetv2.1.data", max_epochs,
        epochs_between_reports, desired_error);
    /***still cant work**/
    // fann_type input[1], *desired_output;
    // input[0] = 5;
    // // desired_output[0] = 7.1414;
    // desired_output[0] = (const float) 7.1414;
    // fann_test(ann, input, desired_output);
    // printf("hasil test : %f\n", desired_output);
    /****/


    fann_save(ann, "listDataSetv3.1.net");
    // DONT FORGET NAMING FOR CALLBACK ERROR

    // fann_print_connections(ann);

    // fann_destroy_train(ann);
    fann_destroy(ann); 

    return (0);
}





// void print_fann_configuration()
// {
//   /** printting information configutarion **/

//   std::cout << "================================================" << std::endl;
//   std::cout << "information configutarion ann" << std::endl;
//   std::cout << "trainning algorithm : " << FANN_TRAIN_NAMES[fann_get_training_algorithm(ann)] << std::endl;
//   std::cout << "Learning Rate : " << fann_get_learning_rate(ann) << std::endl;
//   std::cout << "Learning Momentum : " << fann_get_learning_momentum(ann) << std::endl; //The learning momentum can be used to speed up FANN_TRAIN_INCREMENTAL training.  A too high momentum will however not benefit training.  Setting momentum to 0 will be the same as not using the momentum parameter.  The recommended value of this parameter is between 0.0 and 1.0. default 0
//   // fann_get_activation_function(... ,... ,...)    
//   // fann_get_activation_steepness(..., ..., ...)
//   std::cout << "Error Function : " << FANN_ERRORFUNC_NAMES[fann_get_train_error_function(ann)] << std::endl;
//   std::cout << "Stop Function : " << FANN_STOPFUNC_NAMES[fann_get_train_stop_function(ann)] << std::endl;
//   std::cout << "Bit Fail Limit : " << fann_get_bit_fail_limit(ann) << std::endl;
//   std::cout << "Get Quickprop decay : " << fann_get_quickprop_decay(ann) << std::endl;
//   std::cout << "Get Quickprop mu : " << fann_get_quickprop_mu(ann) << std::endl;
//   std::cout << "Get RPROP Increase factor : " << fann_get_rprop_increase_factor(ann) << std::endl;
//   std::cout << "Get RPROP Decrease factor : " << fann_get_rprop_decrease_factor(ann) << std::endl;
//   std::cout << "Get RPROP delta min : " << fann_get_rprop_delta_min(ann) << std::endl;
//   std::cout << "Get RPROP delta max : " << fann_get_rprop_delta_max(ann) << std::endl;
//   std::cout << "Get RPROP delta zero : " << fann_get_rprop_delta_zero(ann) << std::endl;
//   std::cout << "Get SarProp weight decay shift : " << fann_get_sarprop_weight_decay_shift(ann) << std::endl;
//   std::cout << "Get SarProp step error tresh factor : " << fann_get_sarprop_step_error_threshold_factor(ann) << std::endl;
//   std::cout << "Get SarProp step error shift : " << fann_get_sarprop_step_error_shift(ann) << std::endl;
//   std::cout << "Get SarProp temperatur : " << fann_get_sarprop_temperature(ann) << std::endl;
  
//   std::cout << "================================================" << std::endl;
//   /** end of printting information configutarion **/
// }
