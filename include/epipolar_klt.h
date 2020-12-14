#ifndef _EPIPOLARKLT_H_
#define _EPIPOLARKLT_H_

#include <iostream>
#include <cmath>
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

#include "immintrin.h"

using namespace std;
typedef Eigen::Matrix<float, 2, 1> Vec2;
typedef Eigen::Matrix<float, 2, 2> Mat22;
typedef Eigen::Matrix<float, 3, 1> Vec3;
typedef Eigen::Matrix<float, 3, 3> Mat33;
typedef Eigen::Matrix<float, 5, 1> Vec5;
typedef Eigen::Matrix<float, 5, 5> Mat55;

class EpipolarKLT {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EpipolarKLT(int win_sz, bool flag_fastpattern = false);
    ~EpipolarKLT();

    void runEpipolarKLT(
        const vector<cv::Mat>& Ik,   const vector<cv::Mat>& Ic, 
        const vector<cv::Mat>& du_k, const vector<cv::Mat>& dv_k,
        const vector<cv::Mat>& du_c, const vector<cv::Mat>& dv_c,
        const int& win_sz, const int& MAX_ITER,
        const float& alpha, const float& beta, 
        vector<PointDB>& db);

    void runAffineBrightnessCompensation(
        const vector<cv::Mat>& Ik, const vector<cv::Mat>& Ic, 
        const vector<PointDB>& db,
        float& alpha_res, float& beta_res);

    void runEpipolarAffineKLT(
        const vector<cv::Mat>& Ik,   const vector<cv::Mat>& Ic, 
        const vector<cv::Mat>& du_k, const vector<cv::Mat>& dv_k,
        const vector<cv::Mat>& du_c, const vector<cv::Mat>& dv_c,
        const int& win_sz, const int& MAX_ITER,
        const float& alpha, const float& beta, 
        vector<PointDB>& db);

// points and pattern
private:
    int n_pts_; // # of current pixel points.
    int win_sz_; // length of window.
    int win_half_; // half length of window.
    int M_; // # of patchpoints
    vector<cv::Point2f> patch_; // patch points


// For SIMD (SSE).
private:
    // Hessian and Jacobian
    // We just solve Linear system JtWJ*delta_xi = mJtWr; where mJtWr = -J^t*W*r;
    // JtWJ matrix is guaranteed to be P.S.D and symmetric matrix.
    // Thus, we can efficiently solve this equation by Cholesky decomposition.
    // --> JtWJ.ldlt().solve(mJtWr);
    // JtWJ_sse_ = [
    // 0, *, *, *, *;
    // 1, 5, *, *, *;
    // 2, 6, 9, *, *;
    // 3, 7,10,12, *;
    // 4, 8,11,13,14;
    // mJtWr_sse_ = [15,16,17,18,19]^t
    // err = [20];

    Mat55 JtWJ_sse_, JtJ_sse_;
    Vec5 mJtWr_sse_, JtJinvJt_sse_;

    float* err_sse_;
    int* mask_;
    float* upattern_;
    float* vpattern_;

    float* buf_up_ref_;
    float* buf_vp_ref_;
    float* buf_up_warp_;
    float* buf_vp_warp_;

    float* buf_Ik_;
    float* buf_Ic_warp_;
    float* buf_du_c_warp_;
    float* buf_dv_c_warp_;

    float* buf_residual_;
    float* buf_weight_;

    float* SSEData_; // [4 * 21] (for make "JtWJ_sse_" and "mJtWR_sse_")



private:
    void calcEpipolarAffineKLTSingle(
        const cv::Mat& img_k, const cv::Mat& img_c, 
        const cv::Mat& du_c, const cv::Mat& dv_c, const Vec2& pt_k,
        Vec2& pt_c_tracked, Vec5& point_params,
        float& err_ssd_, float& err_ncc_, int& mask_);

private: 
    void generatePattern(
        int win_half, vector<cv::Point2f>& patch, bool flag_fastpattern);
    

};

#endif