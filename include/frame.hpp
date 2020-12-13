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

#include "camera.hpp"
#include "util/image_processing.hpp"

using namespace std;

class Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Frame(Camera* cam, int max_lvl_pyr);
    ~Frame();

    void constructFrame(const cv::Mat& img_input);

    cv::Mat imgu_raw() const {return img_raw_;};
    cv::Mat imgu() const {return img_undist_;};
    cv::Mat imgf() const {return img_undist_f_;};
    vector<cv::Mat> img_pyr() const {return img_pyr_;};
    vector<cv::Mat> du() const {return du_pyr_;};
    vector<cv::Mat> dv() const {return dv_pyr_;};


private:
    void calcGradient();


private:
    cv::Mat img_raw_; // CV_8UC1
    cv::Mat img_undist_; // CV_8UC1
    cv::Mat img_raw_f_; // CV_32FC1
    cv::Mat img_undist_f_; // CV_32FC1
    cv::Mat du_s_; // CV_16S, sobel result, maximum resolution.
    cv::Mat dv_s_; // CV_16S, sobel result, maximum resolution.
    cv::Mat d_tmp_;

    vector<cv::Mat> img_pyr_; // CV_32F
    vector<cv::Mat> du_pyr_;  // CV_32F
    vector<cv::Mat> dv_pyr_;  // CV_32F

    int MAX_PYR_LVL_;

    Camera* cam_;
};

#endif