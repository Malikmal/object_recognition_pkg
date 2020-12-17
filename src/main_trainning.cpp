#include <iostream>
#include <vector>
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

// argument 1 => file ex : scene_mug_table.pcd 

typedef std::pair<std::string, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> modelRaw;

// void extract_features()
// {

// }

// bool loadModelPCL(const boost::filesystem::path &path, modelRaw &model)
// {
//   try
//   {
//       pcl::PCDReader reader;
//       pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//       reader.read (path.string(), *cloud);  


//       return (1)
//   }
//   catch(const pcl::InvalidConversionException&)
//   {
//       return (false);
//   }
  
// }

void LoadModels (const boost::filesystem::path &base_dir, const std::string &extension, std::vector<modelRaw> &models)
{
  if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
    return;

  for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
  {
    if (boost::filesystem::is_directory (it->status ()))
    {
      std::stringstream ss;
      ss << it->path ();
      pcl::console::print_highlight ("Loading %s (%lu models loaded so far).\n", ss.str ().c_str (), (unsigned long)models.size ());
      LoadModels (it->path (), extension, models);
    }
    if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
    {
      // modelRaw m; 
      // if (loadModelPCL (base_dir / it->path ().filename (), m))
      //     models.push_back (m);
      // }
      pcl::PCDReader reader;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      reader.read (it->path ().string(), *cloud);
      // std::cout << cloud->width << std::endl;
      // m.first = 'asd';
      // m.second = cloud;
      std::string name("Asd"); //(it->path ().filename ().string());
      modelRaw m(name, cloud);

      models.push_back(m);
      // std::cout << "base_dir << '/' "<< it->path ().filename ().string() << std::endl;
      std::cout << "reading file : "<< it->path ().filename ().string() << std::endl;
      
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

    //// VFH DESCRIPTOR
    // std::vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> modelsVFH;
    std::vector<std::vector<float> > VFHValues;
    
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
    typedef pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> VFHEstimationType;
    VFHEstimationType vfhEstimation;

    // for(std::size_t i = 0; i < models.size(); ++i )
    for(auto it : models)
    {

        // std::cout << it.size() << std::endl;

        //  Compute the normals
        normalEstimation.setInputCloud (it.second);

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
        normalEstimation.setSearchMethod (tree);


        normalEstimation.setRadiusSearch (0.03);

        pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);
        normalEstimation.compute (*cloudWithNormals);

        std::cout << "Computed " << cloudWithNormals->points.size() << " normals." << std::endl;
        
        // Setup the feature computation

        // Provide the original point cloud (without normals)
        vfhEstimation.setInputCloud (it.second);

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
        // std::vector<float> VFHValues;
        // float vfh[308] = vfhFeatures->points[0].histogram;

        std::vector<float> VFHValue;
        for(auto it : vfhFeatures->points[0].histogram)
        {
            // std::cout << it << std::endl;
            VFHValue.push_back(it);
        }
        VFHValues.push_back(VFHValue);


        std::cout << it.first << "vfh calculated : " << std::endl;// << VFHValue.size();
    }
    
    std::cout << "jumlah vfh : " << VFHValues.size();


    // ARTIFICIAL NEURAL NETOWRK
    // fann_type *calc_out;
    // fann_type input[308]; //length of VFH Descriptor

    // sconst unsigned int num_input = 1;
    // const unsigned int num_output = 1;
    // const unsigned int num_layers = 4;
    // const unsigned int num_neurons_hidden = 8; 
    // const float desired_error = (const float) 0.0001; // break ketika error sudah lebih kecil dari ini
    // const unsigned int max_epochs = 500000; //epoch iterasi
    // const unsigned int epochs_between_reports = 1000; //jeda printing 

    // for(auto it : VFHValues)
    // {
    //     struct fann *ann = fann_create_standard(num_layers, num_input,
    //         8,8, num_output);

    //     // fann_set_activation_function_hidden(ann, FANN_LINEAR); // set act func all hidden layer
    //     fann_set_activation_function_layer(ann, FANN_SIGMOID_SYMMETRIC, 1); //hidden layer ke 1
    //     fann_set_activation_function_layer(ann, FANN_LINEAR, 2); //hidden layer ke 2 
    //     fann_set_activation_function_output(ann, FANN_LINEAR);

    //     fann_train_on_file(ann, "testPersamaan.data", max_epochs,
    //         epochs_between_reports, desired_error);
        
    //     /***still cant work**/
    //     // fann_type input[1], *desired_output;
    //     // input[0] = 5;
    //     // // desired_output[0] = 7.1414;
    //     // desired_output[0] = (const float) 7.1414;
    //     // fann_test(ann, input, desired_output);
    //     // printf("hasil test : %f\n", desired_output);
    //     /****/

    //     fann_save(ann, "testPersamaan.net");

    //     // fann_print_connections(ann);

    //     fann_destroy(ann); 
    // }

    return (0);
}