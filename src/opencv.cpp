#include "iostream"
#include <cmath>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
// #include "opencv2/opencv.hpp"
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv


#define FACE_DOWNSAMPLE_RATIO 4

//----------------------------------------------------------------------
struct histeq {
	int p[200];
};

struct histeq hist, subblock;

void HistEq(int size) {
	int i, j;
	float d, temp, *lut, *h;
	
	lut = new float[260];
	h = new float[260];
	
	for (i = 0; i < 256; i++) {
		h[i] = 0;
	}
	d = 1. / (size * size);
	
	for (i = 0; i < size; i++){
		for (j = 0; j < size; j++){
			h[subblock.p[i * size + j]] += d;
		}
	}
	temp = 0;
	
	for (i = 0; i<256; i++) {
		temp += h[i];
		lut[i] = temp * 255 + 0.5;
	}
	
	for (i = 0; i<size; i++){
		for (j = 0; j<size; j++) {
			hist.p[i*size + j] = lut[subblock.p[i*size + j]];
		}
	}
    delete[] lut;
    delete[] h;
}

//----------------------------------------------------------------------
int no;
float triangle(float a, float b, float c, float x) {
	float d;
	if(x >= a && x < b){
		d = (x - a) / (b - a);
	}
	else if(x >= b && x < c) {
		d = (c - x) / (c - b);
	}
	else{
		d = 0;
	}
	return(d);
}

float gaussian(float a, float b, float x) {
	float c;
	c = 1.0 / (exp(pow(a - x, 2) / pow(b, 2)));
	return(c);
}

float minimum(float a, float b) {
	float c;

	if(a < b)
		c = a;
	else
		c = b;

	return(c);
}

float fuzzy_a(float a, float b) {
	float s_dmf[100], t_dmf[100], rb[300];
	float fuzz, temp, temp1;
	int i, j;
	float rule1, rule2;
	int no_rule;
	
	//float r[49]={12,8,7,104,115,132,107,9,1,1,51,100,98,87,0,1,0,17,115,144,102,2,0,0,15,108,98,112,2,0,0,3,117,119,103,3,0,0,2,0,0,5,109,147,114,132,12,8,2};
	//float r[9]={0,3,123,0,0,132,0,0,34};
	float r[25] = {10, 2, 57, 142, 145, 1, 0, 7, 137, 118, 0, 0, 2, 97, 148, 0, 0, 0, 57, 145, 20, 2, 14, 0, 31};
	//float r[25]={10,1,31,142,127,1,0,5,137,122,0,0,1,97,148,0,0,0,57,145,20,2,14,0,31}; **
	//float r[25]={1,1,31,142,111,2,0,3,109,122,0,1,0,4,2,4,0,3,0,2,4,0,4,1,1}; **
	//float r[25]={1,1,34,68,67,0,2,4,3,4,0,1,0,4,2,4,0,3,0,2,4,0,4,1,1}; **
	//float r[25]={1,0,0,141,127,98,64,118,130,139,9,106,29,27,12,111,71,39,6,69,121,124,68,62,125};
	//float r[25]={1,0,2,45,67,25,55,4,60,12,69,10,69,58,54,17,3,68,55,14,7,32,18,44,49};
	//float r[25]={0,1,56,68,85,7,45,33,49,0,7,44,56,53,64,68,66,12,50,6,51,66,73,58,12};
	//float r[25]={0,1,109,141,123,1,0,10,14,140,2,0,4,3,3,0,4,0,0,3,1,4,1,1,0}; **
	//float r[25]={6,1,69,59,33,1,0,11,85,32,2,0,4,3,3,0,4,0,0,3,1,4,1,1,0};
	//float r[25]={15,44,32,8,41,26,51,4,9,11,7,2,7,7,12,9,12,5,11,2,5,12,0,7,14};
	//float r[25]={0,35,33,8,2,23,113,4,10,17,3,28,2,6,13,68,1,5,9,0,11,59,66,20,0};
	//float r[25]={56,74,51,63,60,51,62,28,32,61,38,68,52,32,25,66,10,0,73,32,18,59,20,64,30};
	//float r[25]={3,57,38,1,23,2,128,18,10,21,4,9,1,8,6,2,4,7,7,4,2,5,4,5,2};
	//float r[25]={3,62,38,1,4,8,128,18,10,15,86,80,77,8,6,3,131,16,6,2,14,14,10,2,0};

	no_rule = 5;
	rule1 = 1. / (no_rule - 1);
	rule2 = 255. / (no_rule - 1);

	// Fuzzyfikasi : Degree of MF Segmentation
	j = 0;
	s_dmf[0] = triangle(0, 0, rule1, a);
	
	for(i = 1; i < no_rule - 1; i++) {
		s_dmf[i] = triangle(j * rule1, (j + 1) * rule1, (j + 2) * rule1, a);
		j++;
	}
	s_dmf[no_rule-1] = triangle((no_rule - 2) * rule1, 1, 1, a);

	// Fuzzyfikasi : Degree of MF Sigmoid
	j = 0;
	t_dmf[0] = triangle(0, 0, rule2, b);
	for(i = 1; i < no_rule - 1; i++) {
		t_dmf[i] = triangle(j * rule2, (j + 1) * rule2, (j + 2) * rule2, b);
		j++;
	}
	t_dmf[no_rule-1] = triangle((no_rule - 2) * rule2, 255, 255, b);

	// Rule Base
	temp = 0;
	for(i = 0; i < no_rule; i++) {
		for(j = 0; j < no_rule; j++) {
			//rb[i*no_rule+j]=minimum(s_dmf[i],t_dmf[j]);
			rb[i * no_rule + j] = s_dmf[i] * t_dmf[j];
			temp += rb[i * no_rule + j];
		}
	}

	// Output (Defuzzyfikasi)
	temp1 = 0;
	for(i = 0; i < no_rule; i++)
		for(j = 0; j < no_rule; j++)
			temp1 += rb[i * no_rule + j] * r[i * no_rule + j];
	
	fuzz = temp1 / temp;

	return(fuzz);
}

float fuzzy(float a, float b) {
	float s_dmf[100], t_dmf[100], rb[300];
	float fuzz, temp, temp1;
	int i, j;
	float rule1, rule2;
	int no_rule;
	
	float r[25] = { 10, 2, 57, 142, 145, 1, 0, 7, 137, 118, 0, 0, 2, 97, 148, 0, 0, 0, 57, 145, 20, 2, 14, 0, 31 };

	no_rule = 5;
	rule1 = 1. / (no_rule - 1);
	rule2 = 255. / (no_rule - 1);

	// Fuzzyfikasi : Degree of MF Segmentation
	j = 0;
	s_dmf[no] = triangle(0, 0, rule1, a);
	//#pragma omp parallel for
	for (i = 1; i < no_rule - 1; i++) {
		s_dmf[i] = triangle(j * rule1, (j + 1) * rule1, (j + 2) * rule1, a);
		j++;
	}
	s_dmf[no_rule - 1] = triangle((no_rule - 2) * rule1, 1, 1, a);

	// Fuzzyfikasi : Degree of MF Sigmoid
	j = 0;
	t_dmf[no] = triangle(0, 0, rule2, b);
	//#pragma omp parallel for
	for (i = 1; i < no_rule - 1; i++) {
		t_dmf[i] = triangle(j * rule2, (j + 1) * rule2, (j + 2) * rule2, b);
		j++;
	}
	t_dmf[no_rule - 1] = triangle((no_rule - 2) * rule2, 255, 255, b);

	// Rule Base
	temp = 0;
	//#pragma omp parallel for
	for (i = 0; i < no_rule; i++) {
		for (j = 0; j<no_rule; j++) {
			//rb[i*no_rule+j]=minimum(s_dmf[i],t_dmf[j]);
			rb[i * no_rule + j] = s_dmf[i] * t_dmf[j];
			temp += rb[i * no_rule + j];
		}
	}

	// Output (Defuzzyfikasi)
	temp1 = 0;
	//#pragma omp parallel for
	for (i = 0; i < no_rule; i++){
		for (j = 0; j < no_rule; j++){
			temp1 += rb[i * no_rule + j] * r[i * no_rule + j];
		}
	}
	fuzz = temp1 / temp;

	return(fuzz);
}
//double getSimilarity(const Mat A, const Mat B);

//----------------------------------------------------------------------

void FG(cv::Mat src) {
	int x, y, I, J;
	int size, size2;
	int **filter, **datalhe;
	float intrinsic, current, a, b;
	
	cv::Mat MatInput(src.rows / 1.0, src.cols / 1.0, CV_8UC1);
	
	int maxsize = 480;
	resize(src, MatInput, MatInput.size(), 0, 0, 1);
	
	int width = MatInput.cols;
	int height = MatInput.rows;

	// Appearance Estimation Model
	size = 6;
	size2 = size / 2;
	
	height = ((int)((float)height / size)) * size;
	width = ((int)((float)width / size)) * size;
	
	filter = new int *[height + 1];
	for (size_t i = 0; i<height + 1; i++) {
		filter[i] = new int[width + 1];
	}
	datalhe = new int *[height + 1];
	
	for (size_t i = 0; i<height + 1; i++) { 
		datalhe[i] = new int[width + 1];
	}

    for (size_t i = 0; i < height; i++) {
		for (size_t j = 0; j < width; j++) {
			filter[i][j] = 0;
			datalhe[i][j] = 0;
        }
    }
	
	cv::Mat bhe(height, width, CV_8UC1);
	cv::Mat result(height, width, CV_8UC1);
	
	// Spatio-temporal filter
	I = 0;
	for (size_t i = 1; i <= height; i++) {
		J = 0;
		for (size_t j = 1; j <= width; j++) {
			if (I == 0 && J == 0 || I == 0 && J == ((int)((float)(width - size2) / size2)) || I == ((int)((float)(height - size2) / size2)) && J == 0 || I == ((int)((float)(height - size2) / size2)) && J == ((int)((float)(width - size2) / size2))){ 
				filter[i - 1][j - 1] = 1;
			}
            else if (I == 0 && J >= 1 && J <= ((int)((float)(width - size2) / size2)) || J == 0 && I >= 1 && I <= ((int)((float)(height - size2) / size2)) || I == ((int)((float)(height - size2) / size2)) && J >= 1 && J <= ((int)((float)(width - size2) / size2)) || J == ((int)((float)(width - size2) / size2)) && I >= 1 && I <= ((int)((float)(height - size2) / size2))) {
				filter[i - 1][j - 1] = 2;
			}
            else {
				filter[i - 1][j - 1] = 4;
			}

            if (j%size2 == 0) {
				J++;
			}
        }
        if (i%size2 == 0) {
			I++;
		}
	}
	
	// Subblock Histogram Equalization
	for (size_t i = 0; i<height; i++){
		for (size_t j = 0; j<width; j++) {
			datalhe[i][j] = 0;
		}
	}
	
	for (size_t i = 0; i<height - size2; i += size2) { 
		for (size_t j = 0; j<width - size2; j += size2) {
			x = 0;
			for (size_t k = i; k<i + size; k++) {
				y = 0;
				for (size_t l = j; l<j + size; l++) {
					subblock.p[x*size + y] = (int)MatInput.at<uchar>(k, l);
					y++;
				}
				x++;
            }
			HistEq(size);
			x = 0;
			for (size_t k = i; k < i + size; k++){
				y = 0;
				for (size_t l = j; l<j + size; l++){
					datalhe[k][l] += hist.p[x*size + y];
					y++;
				}
				x++;
			}
		}
	}
	
	for (size_t i = 0; i<height; i++) {
		for (size_t j = 0; j<width; j++){
			bhe.at<uchar>(i, j) = (int)((float)datalhe[i][j] / (float)filter[i][j]);
		}
	}
	
	cv::blur(bhe,bhe,cv::Size(1,1),cv::Point(-1,-1),4);
	cv::GaussianBlur(bhe, bhe, cv::Size(0, 0), 1, 0, 4);

    // OptiFuzz based illumination invariant process
    for (size_t i = 0; i<height; i++) {
		for (size_t j = 0; j<width; j++) {
			// Intrinsic appearance model
			intrinsic = (float)bhe.at<uchar>(i, j);
			
			if (intrinsic >= 255) {
				intrinsic = 254;
			}
            // Current appearance model
            current = (float)MatInput.at<uchar>(i, j) / 255;

            if (current >= 1){
				current = 0.999;
			}
			
			// Final calculation of OptiFuzz
			a = fuzzy(current, intrinsic);
			b = 1.2*current;
			
			result.at<uchar>(i, j) = (int)(((a + 1) / (b*a + 1))*(float)MatInput.at<uchar>(i, j));
			MatInput.at<uchar>(i, j) = result.at<uchar>(i, j);
        }
    }
	
	resize(MatInput, src, src.size(), 0, 0, 1);
	bhe.release();
    result.release();
	
	for (size_t i = 0; i<height + 1; i++){
		delete[] filter[i];
	}
	delete[] filter;
	
	for (size_t i = 0; i<height + 1; i++) {
		delete[] datalhe[i];
	}
	delete[] datalhe;
}

//----------------------------------------------------------------------


int main(int, char**) {
	// open the first webcam plugged in the computer
	cv::VideoCapture LaptopCamera(0);
	if (!LaptopCamera.isOpened()) {
		std::cerr << "ERROR: Could not open LaptopCamera" << std::endl;
		return 1;
	}
	
	cv::namedWindow("Window_OriginalImage", cv::WINDOW_AUTOSIZE);	// create a window to display the images from the webcam
	cv::Mat MatCamera;												// this will contain the image from the webcam
	cv::Mat MatBefIllumination;
	cv::Mat MatIllumination;
	
	// display the frame until you press a key
	while (1) {
		// capture the next frame from the webcam
		LaptopCamera >> MatCamera;
		//MatBefIllumination = MatCamera.clone(); 
		//Preprocessing before Illumination
		cv::resize(MatCamera, MatIllumination, cv::Size(320, 240), 1.0/FACE_DOWNSAMPLE_RATIO, 1.0/FACE_DOWNSAMPLE_RATIO);
		cv::cvtColor(MatIllumination, MatIllumination, cv::COLOR_BGR2GRAY);
		FG(MatIllumination);

		// show the image on the window
		cv::imshow("Window_OriginalImage", MatIllumination);
		// wait (10ms) for a key to be pressed
		if (cv::waitKey(10) >= 0){
			break;
		}
	}
	return 0;
}