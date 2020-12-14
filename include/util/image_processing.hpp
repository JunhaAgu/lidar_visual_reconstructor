#ifndef _IMAGE_PROC_H_
#define _IMAGE_PROC_H_
#include <iostream>
#include <exception>
#include <vector>
#include <cmath>
#include "opencv2/opencv.hpp"  
#include "Eigen/Dense"

#include "immintrin.h"
#include "xmmintrin.h"

#include "custom_struct.hpp"
#include "timer.hpp"

using namespace std;

namespace improc{
    void pyrDownNormal(const cv::Mat& source, cv::Mat& dest);
    void pyrDownSSE(const cv::Mat& source, cv::Mat& dest);
    void imagePyramid(const cv::Mat& img, vector<cv::Mat>& img_pyr);
    void interpImage(const cv::Mat& img, const vector<chk::Point2f>& pts, vector<float>& brightness, vector<int>& valid_vec);
    void interpImage(const cv::Mat& img, const vector<chk::Point2f>& pts, chk::Point2f& pt_offset, float* brightness, int* valid_vec);
    float interpImageSingle(const cv::Mat& img, const float& u, const float& v);
    void  interpImageSingleRegularPatch(const cv::Mat& img, const float& uref, const float& vref, 
								const vector<cv::Point2f>& patch, float* res);
    void interpImageSingle3(const cv::Mat& img, const cv::Mat& du, const cv::Mat& dv, const float& u, const float& v, Eigen::Vector3f& interp_);
    void interpImageSingle3RegularPatch(const cv::Mat& img, const cv::Mat& du, const cv::Mat& dv,
	 	 const float& ur, const float& vr, const vector<cv::Point2f>& patch, float* res_img, float* res_du, float* res_dv);
    void sampleImage(const cv::Mat& img, const vector<chk::Point2f>& pts, chk::Point2f& pt_offset, float* brightness,int* valid_vec);
    void diffImage(const cv::Mat& img, cv::Mat& dimg, bool flag_dx, bool flag_dy);
    float calcZNCC(float* a, float* b, int len);
};


#endif