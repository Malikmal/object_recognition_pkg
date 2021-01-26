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

int FANN_API test_callback(struct fann *ann, struct fann_train_data *train,
	unsigned int max_epochs, unsigned int epochs_between_reports, 
	float desired_error, unsigned int epochs)
{
	printf("Epochs     %8d. MSE: %.5f. Desired-MSE: %.5f\n", epochs, fann_get_MSE(ann), desired_error);
	return 0;
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
    ofstream MyFileData("ccccccc_error.txt", ios::app);
    MyFileData << epochs << ", " 
               << fann_get_MSE(ann) << ", " 
               << fann_get_bit_fail(ann) << ", " 
               << desired_error << ", "
               << std::endl;
    std::cout  << epochs << ", " 
               << fann_get_MSE(ann) << ", " 
               << fann_get_bit_fail(ann) << ", " 
               << desired_error << ", "
               << std::endl;
    MyFileData.close();
}

int main()
{
	fann_type *calc_out;
	const unsigned int num_input = 308;
	const unsigned int num_output = 8;
	const unsigned int num_layers = 3;
	const unsigned int num_neurons_hidden = 308;
	const float desired_error = (const float) 0;
	const unsigned int max_epochs = 2000;
	const unsigned int epochs_between_reports = 1;
	struct fann *ann;
	struct fann_train_data *data;

	unsigned int i = 0;
	unsigned int decimal_point;

	printf("Creating network.\n");
	// ann = fann_create_standard(num_layers, num_input, num_neurons_hidden, num_output);
    // ann = fann_create_standard(5, 308, 616, 308, 64, 8); // bbbbbbb
	ann = fann_create_standard(5, 308, 616, 308, 64, 8); // ccccccc
	

	data = fann_read_train_from_file("listDataSetv2.1.data");

	fann_set_activation_steepness_hidden(ann, 0.01); //deafault 0.5 //bbbbbb = 0.01
	fann_set_activation_steepness_output(ann, 0.01); //deafault 0.5
    
    // fann_scale_input_train_data(data, (fann_type) (0), (fann_type) 100);
    // fann_scale_output_train_data(data, (fann_type) (0), (fann_type) 100);

	fann_set_activation_function_hidden(ann, FANN_SIGMOID_STEPWISE);
	fann_set_activation_function_output(ann, FANN_SIGMOID_STEPWISE);
	

	fann_set_train_stop_function(ann, FANN_STOPFUNC_BIT);
	// fann_set_bit_fail_limit(ann, 0.01f); //default 0.35

	fann_set_training_algorithm(ann, FANN_TRAIN_RPROP); //defailt FANN_TRAIN_RPROP

	fann_init_weights(ann, data);
	

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

	printf("Testing network. %f\n", fann_test_data(ann, data));


	for(i = 0; i < fann_length_train_data(data); i++)
	{
		calc_out = fann_run(ann, data->input[i]);
		printf("test (%f,%f) -> %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f \n, should be %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f , difference=%f\n",
			   data->input[i][0], data->input[i][1], 
               calc_out[0], calc_out[1], calc_out[2], calc_out[3], calc_out[4], calc_out[5], calc_out[6], calc_out[7], 
               data->output[i][0], data->output[i][1], data->output[i][2], data->output[i][3], data->output[i][4], data->output[i][5], data->output[i][6], data->output[i][7], 
			   fann_abs(calc_out[0] - data->output[i][0]));
        

	}

	printf("Saving network.\n");

	fann_save(ann, "ccccccc.net");

	decimal_point = fann_save_to_fixed(ann, "ccccccc_fixed.net");
	fann_save_train_to_fixed(data, "cccccccfixed.data", decimal_point);

	printf("Cleaning up.\n");
	fann_destroy_train(data);
	fann_destroy(ann);

	return 0;
}


