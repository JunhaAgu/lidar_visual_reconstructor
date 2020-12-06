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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int n_cols_, n_rows_;

    // Unrectified K and Kinv (distorted)
    cv::Mat cvK_;
    Eigen::Matrix3f K_;
    Eigen::Matrix3f Kinv_;
    float fx_, fy_, cx_, cy_;
    float fxinv_, fyinv_;

    float distortion_[5];
    float k1_, k2_, k3_, p1_, p2_;

    // undistortion maps
    cv::Mat undist_map_x_; // CV_32FC1
    cv::Mat undist_map_y_; // CV_32FC1

public:
    Camera(int n_cols, int n_rows, float fx, float fy, float cx, float cy, float k1, float k2, float k3, float p1, float p2);
    ~Camera();

    void generateUndistortMaps();
    void undistortImage(const cv::Mat& raw, cv::Mat& rectified);

private:

};

#endif