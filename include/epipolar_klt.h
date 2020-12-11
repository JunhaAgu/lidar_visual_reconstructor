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
#include "custom_struct.hpp"

using namespace std;
typedef Eigen::Matrix<float, 2, 1> Vec2;
typedef Eigen::Matrix<float, 2, 2> Mat2;
typedef Eigen::Matrix<float, 6, 1> Vec6;
typedef Eigen::Matrix<float, 6, 6> Mat66;

class EpipolarKLT {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EpipolarKLT(const int& n_cols, const int& n_rows, const int& MAX_PYR_LVL);
    ~EpipolarKLT();

    void registerImages(const cv::Mat& Ik, const cv::Mat& Ic, 
            const cv::Mat& du_k, const cv::Mat& dv_k,
            const cv::Mat& du_c, const cv::Mat& dv_c);
    void registerQueryPoints();
    void registerPriorPoints();

    void calcOpticalFlowEpipolarKLTPyr(int i);
    void calcOpticalFlowEpipolarKLTPyr();
    
    void calcOpticalFlowAffineKLTPyr(int i);
    void calcOpticalFlowAffineKLTPyr(); // with prior points

private:
    int n_pts_; // # of current pixel points.
    int MAX_PYR_LVL_; // maximum pyramid level.

    vector<int> n_cols_pyr_;
    vector<int> n_rows_pyr_;

    vector<chk::Point2f> patch_;
    int win_sz_; // half length of KLT window
    int M_; // # of elements in a patch.
    
    // Hessian and Jacobian
    // We just solve Linear system JtWJ*delta_xi = mJtWr; where mJtWr = -J^t*W*r;
    // JtWJ matrix is guaranteed to be P.S.D and symmetric matrix.
    // Thus, we can efficiently solve this equation by Cholesky decomposition.
    // --> JtWJ.ldlt().solve(mJtWr);
    Mat66 JtWJ;
    Vec6 mJtWr;

    Mat66 JtJ;
    Vec6 JtJinvJt;

    float* errs_ssd; // tracking err (SSD)
    float* errs_ncc; // tracking err (NCC with considering affine transform)
    int* mask;      // tracking pass or fail.

private:
    // For SSE (AVX2)
    Mat66 JtWJ_sse;
    Vec6 mJtWr_sse;

    float* upattern;
    float* vpattern;

    float* buf_up_ref;
    float* buf_vp_ref;
    float* buf_up_warp;
    float* buf_vp_warp;

    float* buf_Ik;
    float* buf_du_k;
    float* buf_dv_k;

    float* buf_Ic_warp;
    float* buf_du_c_warp;
    float* buf_dv_c_warp;

    float* buf_residual;
    float* buf_weight;

    float* SSEData; // [4 * 28] (for make JtWJ and JtWr)
    // JtWJ = [
    // 0, *, *, *, *, *;
    // 1, 6, *, *, *, *;
    // 2, 7,11, *, *, *;
    // 3, 8,12,15, *, *;
    // 4, 9,13,16,18, *;
    // 5,10,14,17,19,20];
    // JtWr = [21,22,23,24,25,26]^t
    // err = [27];
    float* errs_ssd_sse;

private:
    inline void update(const Vec6& Jt, const float& r, const float& weight, float& err_ssd);
    void solveGaussNewtonStep(Vec6& delta);
    void trackForwardAdditiveSingle(
        const cv::Mat& img_k, const cv::Mat& img_c, const cv::Mat& du_c, const cv::Mat& dv_c, const Vec2& pt_k,
        Vec2& pt_c_tracked, Vec6& point_params,
        float& err_ssd_, float& err_ncc_, int& mask_);

};

#endif