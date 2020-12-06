#ifndef _FRAME_H_
#define _FRAME_H_
#include <iostream>
#include <vector>

#include <Eigen/Dense>

#define _ROS_AVAILABLE_
#ifdef _ROS_AVAILABLE_
    #include <cv_bridge/cv_bridge.h>
#endif

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

class Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Frame(int max_lvl_pyr);
    ~Frame();

    void calcGradient(const cv::Mat& img, const cv::Mat& img_f);


private:
    cv::Mat du_s; // CV_16S, sobel result, maximum resolution.
    cv::Mat dv_s; // CV_16S, sobel result, maximum resolution.

    vector<cv::Mat> img_pyr; // CV_32F
    vector<cv::Mat> du_pyr;  // CV_32F
    vector<cv::Mat> dv_pyr;  // CV_32F

    int MAX_PYR_LVL;


};

#endif