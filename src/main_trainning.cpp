#include <iostream>
#include <vector>
#include <array>
#include <algorithm>
#include <fstream>

#include <boost/filesystem.hpp>

#include "fann.h"
#include "floatfann.h"
#include "fann_data.h"
// #include "parallel_fann.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/vfh.h>
#include <pcl/features/our_cvfh.h>
#include <pcl/features/gasd.h>
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

      //parsing here
      std::vector<std::string> stringLabelParsed;
      boost::algorithm::split(stringLabelParsed, base_dir.string(), boost::is_any_of("/"));
      /**explanation
       * e.g : /media/robo/Windows/FFOutput/capture_dataset/class/bowl
       * take last "bowl"
       * change this if you have different path dataset 
       * in washington rgbd dataset  use this 
       * e.g : dataset/bowl/bowl_1/bowl_1_100.pcd
       * so take second name path not last
       * */



      info dataInfo;
      dataInfo.no  = i;
      // dataInfo.category = base_dir.string();
      dataInfo.category = stringLabelParsed[(stringLabelParsed.size() - 1)];
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
    fann_type *calc_out_tmp;
    /*** calculate the accuracy ***/
    int countTrue = 0;
    for(int i = 0; i < fann_length_train_data(train); i++)
    {
        calc_out_tmp = fann_run(ann, train->input[i]);
        // printf("test (%f,%f) -> %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f \n, should be %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f , difference=%f\n",
        //     data->input[i][0], data->input[i][1], 
        //             calc_out_tmp[0], calc_out_tmp[1], calc_out_tmp[2], calc_out_tmp[3], calc_out_tmp[4], calc_out_tmp[5], calc_out_tmp[6], calc_out_tmp[7], 
        //             data->output[i][0], data->output[i][1], data->output[i][2], data->output[i][3], data->output[i][4], data->output[i][5], data->output[i][6], data->output[i][7], 
        //     fann_abs(calc_out_tmp[0] - data->output[i][0]));
        int maxIndexOutput = 0, maxIndexInput = 0;
        float maxValueOutput = 0.0f, maxValueInput = 0.0f;
        for(int j = 0; j < ann->num_output; j++ )
        {
            if(calc_out_tmp[j] > maxValueOutput)
            {
              maxIndexOutput = j;
              maxValueOutput = calc_out_tmp[j];
            }
            if(train->output[i][j] > maxValueInput)
            {
              maxIndexInput = j;
              maxValueInput = train->output[i][j];
            }
        }
        if(maxIndexInput == maxIndexOutput) 
            countTrue++;
        // std::cout << i << ", " << maxIndexInput << " - " 
        //           << maxIndexOutput << " " << maxValueOutput*100 << " - " 
        //           << (
        //             (maxIndexInput == maxIndexOutput) ? "true" : "false"
        //           )
        //           << std::endl;

    }
    float accuracy = (float)countTrue/(float)fann_length_train_data(train);
    // std::cout << countTrue << "/" << fann_length_train_data(train) << " - Accuracy : " << accuracy << std::endl;
    /*** calculate the accuracy ***/

    ofstream MyFileData("newDatasetv6.9_error.txt", ios::app);
    MyFileData << epochs << ", " 
               << fann_get_MSE(ann) << ", " 
               << accuracy << ","
               << desired_error << ", "
               << std::endl;
    std::cout  << epochs << ", " 
               << fann_get_MSE(ann) << ", " 
               << accuracy << ","
               << desired_error << ", "
               << std::endl;
    MyFileData.close();
}

int main (int argc, char** argv)
{
//a
    clock_t start, end;
    double time_taken, total_time = 0.0f;

    start = clock();
    std::vector<modelRaw> models;
    std::string extension (".pcd");
    transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);

    LoadModels(argv[1], extension, models); // argv[1] = "dataset_washington" 

    ofstream MyFileData("newDatasetv6.9.txt");
    int no  = 0;
    std::vector<int> category;
    std::vector<std::string> categoryName;
    for(auto it : models)
    {
       if(no != it.first.no)
       {
          MyFileData << it.first.category << std::endl;
          category.push_back(it.first.no);
          categoryName.push_back(it.first.category);
       }
        no = it.first.no;
    }
    MyFileData.close();

    end = clock();
    time_taken = double(end - start) * 1000 / double(CLOCKS_PER_SEC); //ms
    total_time += time_taken;
    std::cerr <<  time_taken << setprecision(2) << std::fixed << " ms | " << "file readed : " << models.size() << std::endl;



    // // ///// FILTER AND SEGMENTATION
    start = clock();
    std::vector<modelRaw> modelsSegmented;
    pcl::PCDWriter writer;
    for(auto it : models)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>());

        // Filter Removing NaN data Pointcloud.
        std::vector<int> mapping;
        pcl::removeNaNFromPointCloud(*it.second, *cloud_filtered, mapping);
        // std::cerr << "Pointcloud remove NaN : " << cloud_filtered->size() << " data points." << std::endl;

        // // Filter object crop by z coordinate
        // pcl::PassThrough<pcl::PointXYZRGB> pass;
        // pass.setInputCloud (cloud_filtered);
        // pass.setFilterFieldName ("z");
        // pass.setFilterLimits (0.0, 2.0);
        // //pass.setFilterLimitsNegative (true);
        // pass.filter (*cloud_filtered);
        // // std::cerr << "Pointcloud after cropped : " << cloud->size() << " data points." << std::endl;


        // Filter downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud (cloud_filtered);
        vg.setLeafSize (0.005f, 0.005f, 0.005f); // 0.5cm
        vg.filter (*cloud_filtered);
        // std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*


        // Create the segmentation object for the planar model and set all the parameters
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.01);


      /* loop */
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
          pcl::ExtractIndices<pcl::PointXYZRGB> extract;
          extract.setInputCloud (cloud_filtered);
          extract.setIndices (inliers);
          extract.setNegative (false);

          // Get the points associated with the planar surface
          extract.filter (*cloud_plane);
          // std::cout << "PointCloud representing the planar component" << i << "th : " << cloud_plane->size () << " data points." << std::endl;

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
        ec.setClusterTolerance (0.02); // cm
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_filtered);
        ec.extract (cluster_indices);

        // std::cout << "cloudClustered : " << cloud_clustered->size() << std::endl;

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

          // std::cout << "PointCloud representing the Cluster " << j << " th : " << cloud_cluster->size () << " data points." << std::endl;
          

          j++;
        }

        *cloud_filtered = *cloud_clustered;


        

        // std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*

        //save segmented dataset
        // std::stringstream ssCloud;
        // ssCloud << "capture_dataset/segmented/" << it.first.category  << "_" << it.first.no << ".pcd";
        // writer.write<pcl::PointXYZRGB> (ssCloud.str (), *cloud_filtered, false); //*

        // std::string dir = "capture_dataset/segmented/" + it.first.category;
        // boost::filesystem::create_directory(dir);
        // std::string ssCloud = dir + "/" + std::to_string(it.first.no) + "_" + it.first.filename + ".pcd";
        // pcl::io::savePCDFile(ssCloud, *cloud_filtered);
        // std::cout << "model segmented saved" << ssCloud  << std::endl;


        // //PUSH TO MAIN DATA ARRAY 
        modelRaw tmpModelSegmented(it.first, cloud_filtered);
        modelsSegmented.push_back(tmpModelSegmented);
    }

    end = clock();
    time_taken = double(end - start) * 1000 / double(CLOCKS_PER_SEC); //ms
    total_time += time_taken;
    std::cerr <<  time_taken << setprecision(2) << std::fixed << " ms | " << "All objects has ben segmented" << std::endl;
  



    //// VFH DESCRIPTOR
    start = clock();
    // std::vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> modelsVFH;
    std::vector<std::vector<float> > VFHValues;

    //write file for data trainninng FANN librray format
    ofstream MyFile("newDatasetv6.9.data");
    MyFile << models.size() << " 308 " << category.size() << std::endl; // coutn of row, count of input node ann, count of output node ann

    //write file for data trainninng csv format 
    // Warning i'm using ; seperated symbol standard in my country
    ofstream MyFileCsv("newDatasetv6.9.csv");

    // saving csv header format
    MyFileCsv << "label" << ";" << "filename"; 
    for(int i =1 ; i <= 308; i++)
    {
        // std::cout << it2 << std::endl;
        MyFileCsv  << ";" << "H" << i ;
    }
    MyFileCsv << std::endl;

    
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
    typedef pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> VFHEstimationType;
    typedef pcl::OURCVFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> OurVFHEstimationType;
    // pcl::GASDColorEstimation<pcl::PointXYZRGB, pcl::GASDSignature984> gasdEstimation;
    VFHEstimationType vfhEstimation;
    OurVFHEstimationType ourVfhEstimation;
    

    // for(std::size_t i = 0; i < models.size(); ++i )
    for(auto it : modelsSegmented)// models)//modelsSegmented)
    {

        // std::cout << it.size() << std::endl;

        //  Compute the normals
        normalEstimation.setInputCloud (it.second);

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
        normalEstimation.setSearchMethod (tree);


        normalEstimation.setRadiusSearch (0.01);

        pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);
        normalEstimation.compute (*cloudWithNormals);

        // std::cout << "Computed " << cloudWithNormals->points.size() << " normals." << std::endl;
        
        // SetupValue the feature computation

        // Provide the original point cloud (without normals)
        vfhEstimation.setInputCloud (it.second);

        // Provide the point cloud with normals
        vfhEstimation.setInputNormals(cloudWithNormals);

        // Use the same KdTree from the normal estimation
        vfhEstimation.setSearchMethod (tree);

        //vfhEstimation.setRadiusSearch (0.2); // With this, error: "Both radius (.2) and K (1) defined! Set one of them to zero first and then re-run compute()"


        // ourVfhEstimation.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.
        // ourVfhEstimation.setCurvatureThreshold(1.0);
        // ourVfhEstimation.setNormalizeBins(true);
        // // Set the minimum axis ratio between the SGURF axes. At the disambiguation phase,
        // // this will decide if additional Reference Frames need to be created, if ambiguous.
        // ourVfhEstimation.setAxisRatio(0.8);

        // Actually compute the VFH features
        pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhFeatures(new pcl::PointCloud<pcl::VFHSignature308>);
        vfhEstimation.compute (*vfhFeatures);

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

        // saving csv format
        MyFileCsv << it.first.category << ";" << it.first.filename ;
        for(auto it2 : vfhFeatures->points[0].histogram)
        {
            // std::cout << it2 << std::endl;
            MyFileCsv << ";" << it2 ;
        }
        MyFileCsv << std::endl;

        //saving fann format
        std::vector<float> VFHValue;
        for(auto it2 : vfhFeatures->points[0].histogram)
        {
            // std::cout << it2 << std::endl;
            VFHValue.push_back(it2);
            MyFile << it2 << " " ;
        }
        MyFile << std::endl;
        // int i = 0;
        for(auto it2 : category)
        {
          if(it2 == it.first.no)
              MyFile << 1 ;      
          else
              MyFile << 0 ;
          MyFile << " "; 
        }
        MyFile << std::endl;
        // MyFile << it.first.no << " " << std::endl;
        VFHValues.push_back(VFHValue);


        // std::cout << it.first.filename << " vfh calculated : " << std::endl;// << VFHValue.size();
      std::cout << it.first.no << ". VFH calculated " << it.first.category << " of file : "<< it.first.filename << std::endl;
    }
    
    // Close the file
    MyFile.close();
    end = clock();
    time_taken = double(end - start) * 1000 / double(CLOCKS_PER_SEC); //ms
    total_time += time_taken;
    std::cerr <<  time_taken << setprecision(2) << std::fixed << " ms | " << "All vfh of each object has calculated : " << VFHValues.size() << std::endl;






     // ARTIFICIAL NEURAL NETOWRK
    start = clock();
    fann_type *calc_out;
    // const unsigned int num_input = 308;
    // const unsigned int num_output = 8;
    // const unsigned int num_layers = 3;
    // const unsigned int num_neurons_hidden = 308;
    const float desired_error = (const float) 0.00000001;
    const unsigned int max_epochs = 2000;
    const unsigned int epochs_between_reports = 1;
    struct fann *ann;
    struct fann_train_data *data;

    unsigned int i = 0;
    unsigned int decimal_point;

    printf("Creating network.\n");
    // ann = fann_create_standard(num_layers, num_input, num_neurons_hidden, num_output);
    // ann = fann_create_standard(5, 308, 616, 308, 64, 8); // newDatasetv6.9
    ann = fann_create_standard(3, 308, 616, category.size()); // newDatasetv6.9
    
    // ann = fann_create_standard(5, 308, 616, 308, 64, 8); // ccccccc


    data = fann_read_train_from_file("newDatasetv6.9.data");

    // fann_set_activation_steepness_hidden(ann, 0.01); //deafault 0.5 //bbbbbb = 0.01
    // fann_set_activation_steepness_output(ann, 0.01); //deafault 0.5

    // fann_scale_input_train_data(data, (fann_type) (0), (fann_type) 100);
    // fann_scale_output_train_data(data, (fann_type) (0), (fann_type) 100);

    fann_set_activation_function_hidden(ann, FANN_SIGMOID);
    fann_set_activation_function_output(ann, FANN_SIGMOID);


    // fann_set_train_stop_function(ann, FANN_STOPFUNC_BIT); //defailt is MSE
    // fann_set_bit_fail_limit(ann, 0.01f); //default 0.35

    fann_set_training_algorithm(ann, FANN_TRAIN_BATCH); //defailt FANN_TRAIN_RPROP

    // fann_init_weights(ann, data);


    fann_set_callback(ann, (fann_callback_type)&callbackTrainning); //callback to save error

    printf("Training network.\n");
    fann_train_on_data(ann, data, max_epochs, epochs_between_reports, desired_error);

    // float error;
    // ofstream MyFileData("ccccccc_error.txt", ios::app);
    // for(i = 1; i <= max_epochs; i++)
    // {
    // error = fann_train_epoch_irpropm_parallel(ann, data, 4);
    // 	printf("Epochs     %8d. Current error: %.10f. Bit Fail: %5d\n", i, error, fann_get_bit_fail(ann));
    //     MyFileData << i << "#" 
    //             << error << "#"
    //             << fann_get_bit_fail(ann) << "#"
    //             << std::endl;
    // }
    // MyFileData.close();

    // printf("Testing network. %f\n", fann_test_data(ann, data));


    int countTrue = 0;
    for(i = 0; i < fann_length_train_data(data); i++)
    {
        calc_out = fann_run(ann, data->input[i]);
        // printf("test (%f,%f) -> %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f \n, should be %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f , difference=%f\n",
        //     data->input[i][0], data->input[i][1], 
        //             calc_out[0], calc_out[1], calc_out[2], calc_out[3], calc_out[4], calc_out[5], calc_out[6], calc_out[7], 
        //             data->output[i][0], data->output[i][1], data->output[i][2], data->output[i][3], data->output[i][4], data->output[i][5], data->output[i][6], data->output[i][7], 
        //     fann_abs(calc_out[0] - data->output[i][0]));
        int maxIndexOutput = 0, maxIndexInput = 0;
        float maxValueOutput = 0.0f, maxValueInput = 0.0f;
        for(int j = 0; j < ann->num_output; j++ )
        {
            if(calc_out[j] > maxValueOutput)
            {
              maxIndexOutput = j;
              maxValueOutput = calc_out[j];
            }
            if(data->output[i][j] > maxValueInput)
            {
              maxIndexInput = j;
              maxValueInput = data->output[i][j];
            }
        }
        if(maxIndexInput == maxIndexOutput) 
            countTrue++;
        std::cout << i << ", " << categoryName[maxIndexInput] << " - " 
                  << categoryName[maxIndexOutput] << " " << maxValueOutput*100 << " - " 
                  << (
                    (maxIndexInput == maxIndexOutput) ? "true" : "false"
                  )
                  << std::endl;

    }
    std::cout << countTrue << "/" << fann_length_train_data(data) << " - Accuracy : " << (float) countTrue/fann_length_train_data(data) << std::endl;

    printf("Saving network.\n");

    fann_save(ann, "newDatasetv6.9.net");

    decimal_point = fann_save_to_fixed(ann, "newDatasetv6.9_fixed.net");
    fann_save_train_to_fixed(data, "newDatasetv6.9fixed.data", decimal_point);

    printf("Cleaning up.\n");
    fann_destroy_train(data);
    fann_destroy(ann);


    
    end = clock();
    time_taken = double(end - start) * 1000 / double(CLOCKS_PER_SEC); //ms
    total_time += time_taken;
    std::cerr <<  time_taken << setprecision(2) << std::fixed << " ms | " << "Trainnig process has completed " << std::endl;


    std::cerr <<  time_taken << setprecision(2) << std::fixed << "ms | "<< "All process has done" << std::endl;

    
    /*
    // const unsigned int num_input = 308;
    // const unsigned int num_output = 8; //according to count category
    // const unsigned int num_layers = 4;
    // const unsigned int num_neurons_hidden = 308; 
    // const float desired_error = (const float) 0.01; // break ketika error sudah lebih kecil dari ini
    // const unsigned int max_epochs = 200;//50000; //epoch iterasi
    // const unsigned int epochs_between_reports = 1; //jeda printing 
    // struct fann *ann = fann_create_standard(
    //     9,    //num_layers, 
    //     308,  //num_input,
    //     308*2,  // num_neurons_hidden, 
    //     308*2,  // num_neurons_hidden, 
    //     308,  // num_neurons_hidden, 
    //     154,  // (int) num_neurons_hidden/2,
    //     77,   // (int) num_neurons_hidden/4,
    //     38,   // (int) num_neurons_hidden/8,
    //     19,   // (int) num_neurons_hidden/32,
    //     num_output //8
    // );
    // // fann_set_activation_function_hidden(ann, FANN_LINEAR); // set act func all hidden layer
    // // fann_set_activation_function_layer(ann, FANN_LINEAR_PIECE, 1); //hidden layer ke 1 
    // // fann_set_activation_function_layer(ann, FANN_LINEAR_PIECE, 2); //hidden layer ke 2 
    // // fann_set_activation_function_layer(ann, FANN_LINEAR_PIECE, 3); //hidden layer ke 3
    // // fann_set_activation_function_layer(ann, FANN_SIGMOID, 4); //hidden layer ke 4
    // // fann_set_activation_function_layer(ann, FANN_SIGMOID, 5); //hidden layer ke 5
    // // fann_set_activation_function_layer(ann, FANN_SIGMOID, 6); //hidden layer ke 6
    // // fann_set_activation_function_layer(ann, FANN_SIGMOID, 7); //hidden layer ke 7
    // // fann_set_activation_function_output(ann, FANN_SIGMOID);
    // // fann_set_train_error_function(ann, FANN_ERRORFUNC_LINEAR); //The default error function is FANN_ERRORFUNC_TANH
    // // fann_set_train_stop_function(ann, FANN_STOPFUNC_MSE); //The default stop function is FANN_STOPFUNC_MSE
    // // fann_set_training_algorithm(ann, FANN_TRAIN_BATCH); // deafult FANN_TRAIN_RPROP
    // // fann_set_learning_rate(ann, 10.7); //deafult fann 0.7
    // // fann_randomize_weights(ann, 0.0, 5.0);
    // // fann_reset_MSE(ann);
    
    // fann_set_callback(ann, (fann_callback_type)&callbackTrainning); //callback to save error
    // // fann_type* weights;
    // // fann_get_weights(ann, weights );
    // // // for(int i =0; i < sizeof(weights)/sizeof(weights[0]) ; i++)
    // // // {
    // // //   std::cout << " ";// weights[i] << " ";
    // // // }
    // // std::cout << sizeof(&weights[0]) << std::endl;
    // // print_fann_configuration(ann); //handmade   
    // fann_train_on_file(ann, "newDatasetv6.9.data", max_epochs,
    //     epochs_between_reports, desired_error);
    // fann_save(ann, "listDataSetv3.1.net");
    // // DONT FORGET NAMING FOR CALLBACK ERROR
    // // fann_print_connections(ann);
    // // fann_destroy_train(ann);
    // fann_destroy(ann); 
*/
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