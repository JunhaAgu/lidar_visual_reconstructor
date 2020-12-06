#ifndef _CAMERA_H_
#define _CAMERA_H_

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

class Camera{
public:
    Camera();
    ~Camera();

    void initParams(int n_cols, int n_rows, const cv::Mat& cvK, const cv::Mat& cvD);
    void generateUndistortMaps();
    void undistortImage(const cv::Mat& raw, cv::Mat& rectified);

public:

    const int cols() const{return n_cols_;};
    const int rows() const{return n_rows_;};
    const float fx() const {return fx_;};
    const float fy() const {return fy_;};
    const float cx() const {return cx_;};
    const float cy() const {return cy_;};
    const float fxinv() const {return fxinv_;};
    const float fyinv() const {return fyinv_;};
    const Eigen::Matrix3f K() const {return K_;};
    const Eigen::Matrix3f Kinv() const {return Kinv_;};

private:
    
    int n_cols_, n_rows_;

    Eigen::Matrix3f K_;
    Eigen::Matrix3f Kinv_;
    float fx_, fy_, cx_, cy_;
    float fxinv_, fyinv_;
    float distortion_[5];
    float k1_, k2_, k3_, p1_, p2_;

    // undistortion maps
    cv::Mat undist_map_x_; // CV_32FC1
    cv::Mat undist_map_y_; // CV_32FC1
};

#endif