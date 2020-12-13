#ifndef _EPIPOLARKLT_H_
#define _EPIPOLARKLT_H_

#include <iostream>
#include <vector>
#include "Eigen/Dense"

// ROS cv_bridge
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "custom_memory.hpp"
#include "point_database.hpp"
#include "util/image_processing.hpp"

using namespace std;
typedef Eigen::Matrix<float, 2, 1> Vec2;
typedef Eigen::Matrix<float, 2, 2> Mat2;
typedef Eigen::Matrix<float, 6, 1> Vec6;
typedef Eigen::Matrix<float, 6, 6> Mat66;

class EpipolarKLT {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EpipolarKLT();
    ~EpipolarKLT();

    void runEpipolarKLT(
        const vector<cv::Mat>& Ik,   const vector<cv::Mat>& Ic, 
        const vector<cv::Mat>& du_k, const vector<cv::Mat>& dv_k,
        const vector<cv::Mat>& du_c, const vector<cv::Mat>& dv_c,
        const int& win_sz, const int& MAX_ITER,
        const float& logalpha, const float& beta, 
        vector<PointDB>& db);

    void runAffineBrightnessCompensation(
        const vector<cv::Mat>& Ik, const vector<cv::Mat>& Ic, 
        float& logalpha_res, float& beta_res);

    void runEpipolarAffineKLT(
        const vector<cv::Mat>& Ik,   const vector<cv::Mat>& Ic, 
        const vector<cv::Mat>& du_k, const vector<cv::Mat>& dv_k,
        const vector<cv::Mat>& du_c, const vector<cv::Mat>& dv_c,
        const int& win_sz, const int& MAX_ITER,
        const float& logalpha, const float& beta, 
        vector<PointDB>& db);
        
private:
    int n_pts_; // # of current pixel points.

    // Hessian and Jacobian
    // We just solve Linear system JtWJ*delta_xi = mJtWr; where mJtWr = -J^t*W*r;
    // JtWJ matrix is guaranteed to be P.S.D and symmetric matrix.
    // Thus, we can efficiently solve this equation by Cholesky decomposition.
    // --> JtWJ.ldlt().solve(mJtWr);
    Mat66 JtWJ, JtJ;
    Vec6 mJtWr, JtJinvJt;

private:
    inline void update(const Vec6& Jt, const float& r, const float& weight, float& err_ssd);
    void solveGaussNewtonStep(Vec6& delta);
    void trackForwardAdditiveSingle(
        const cv::Mat& img_k, const cv::Mat& img_c, const cv::Mat& du_c, const cv::Mat& dv_c, const Vec2& pt_k,
        Vec2& pt_c_tracked, Vec6& point_params,
        float& err_ssd_, float& err_ncc_, int& mask_);

};

#endif