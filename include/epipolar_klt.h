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
#include "stdlib.h"

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

    void runEpipolarKLT_LM(
        const vector<cv::Mat>& Ik,   const vector<cv::Mat>& Ic, 
        const vector<cv::Mat>& du_k, const vector<cv::Mat>& dv_k,
        const vector<cv::Mat>& du_c, const vector<cv::Mat>& dv_c,
        const int& win_sz, const int& MAX_ITER,
        const float& alpha, const float& beta, 
        vector<PointDB>& db);

    void runEpipolarKLT_SSE(
            const vector<cv::Mat>& Ik, const vector<cv::Mat>& Ic, 
            const vector<cv::Mat>& du_k, const vector<cv::Mat>& dv_k,
            const vector<cv::Mat>& du_c, const vector<cv::Mat>& dv_c,
            const int& win_sz, const int& MAX_ITER,
            const float& logalpha, const float& beta, 
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

    void runEpipolarAffineKLT_SSE(
        const vector<cv::Mat>& Ik,   const vector<cv::Mat>& Ic, 
        const vector<cv::Mat>& du_k, const vector<cv::Mat>& dv_k,
        const vector<cv::Mat>& du_c, const vector<cv::Mat>& dv_c,
        const int& win_sz, const int& MAX_ITER,
        const float& alpha, const float& beta, 
        vector<PointDB>& db);

    void runEpipolarAffineKLT_AVX(
        const vector<cv::Mat>& Ik,   const vector<cv::Mat>& Ic, 
        const vector<cv::Mat>& du_k, const vector<cv::Mat>& dv_k,
        const vector<cv::Mat>& du_c, const vector<cv::Mat>& dv_c,
        const int& win_sz, const int& MAX_ITER,
        const float& alpha, const float& beta, 
        vector<PointDB>& db);

    void runEpipolarAffineKLT_AVX_LM(
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
    // JtWJ_simd_ = [
    // 0, *, *, *, *;
    // 1, 5, *, *, *;
    // 2, 6, 9, *, *;
    // 3, 7,10,12, *;
    // 4, 8,11,13,14;
    // JtWJ_simd_ = [15,16,17,18,19]^t
    // err = [20];

    int n_patch_;
    int n_ssesteps_; // == floor(n_patch_ / 4)
    int n_patch_trunc_ ; // == 4* n_ssesteps_ <= n_patch_

    Mat55 JtWJ_simd_;
    Vec5 mJtWr_simd_;
    
    float JtWJ_normal_;
    float mJtWr_normal_;

    float* upattern_;
    float* vpattern_;

    float* buf_u_ref_;
    float* buf_v_ref_;
    float* buf_u_warp_;
    float* buf_v_warp_;

    float* buf_Ik_;
    float* buf_Ic_warp_;
    float* buf_du_c_warp_;
    float* buf_dv_c_warp_;

    float* buf_residual_;
    float* buf_weight_;

    float* SSEData_; // [4 * 21] (for make "JtWJ_simd_" and "JtWJ_simd_")
    float* AVXData_; // [8 * 21] (for make "JtWJ_simd_" and "JtWJ_simd_")

    // for interpolation SIMD version
    float* I00_;
    float* I01_;
    float* I10_;
    float* I11_;
    
    float* mask_Ik_; // test validity of pixel.

    float* mask_Ic_;
    float* mask_duc_;
    float* mask_dvc_;

    bool* mask_valid_;

    void simd_setPatchPattern(const vector<cv::Point2f>& patch, int stepsize);
    
    void simd_generateReferencePoints_SSE(const float& u, const float& v); // commonly used.
    void simd_generateReferencePoints_AVX(const float& u, const float& v); // commonly used.

    void simd_warpNormal_SSE(const float& s,     const float& lx, const float& ly, const float& u, const float& v);
    void simd_warpAffine_SSE(const Vec5& params, const float& lx, const float& ly, const float& u, const float& v);
    void simd_warpAffine_AVX(const Vec5& params, const float& lx, const float& ly, const float& u, const float& v);
    void simd_warpAffine_AVX_LM(const Vec5& params, const float& s_far, const float& lx, const float& ly, const float& u, const float& v);
    
    void simd_interpImage(const cv::Mat& Ik); // non simd version
    void simd_interpImage_SSE(const cv::Mat& img, float* buf_u, float* buf_v,  float* res_img, float* mask);
    void simd_interpImage_AVX(const cv::Mat& img, float* buf_u, float* buf_v,  float* res_img, float* mask);

    void simd_calcResidualAndWeight    (const cv::Mat& Ic, const cv::Mat& du_c, const cv::Mat& dv_c, 
        const float& alpha, const float& beta,  const float& thres_huber);
    void simd_calcResidualAndWeight_SSE(const cv::Mat& Ic, const cv::Mat& du_c, const cv::Mat& dv_c,
        const float& alpha, const float& beta, const float& thres_huber);
    void simd_calcResidualAndWeight_AVX(const cv::Mat& Ic, const cv::Mat& du_c, const cv::Mat& dv_c,
        const float& alpha, const float& beta, const float& thres_huber);
    void simd_calcResidual_AVX(const cv::Mat& Ic,
        const float& alpha, const float& beta, const float& thres_huber, float& res);

    void simd_calcHessianAndJacobianNormal_SSE(const float& s, const float& lx, const float& ly, float& err_sse);
    void simd_calcHessianAndJacobianAffine_SSE(const Vec5& params, const float& lx, const float& ly, float& err_sse);
    void simd_calcHessianAndJacobianAffine_AVX(const Vec5& params, const float& lx, const float& ly, float& err_sse);
    void simd_calcHessianAndJacobianAffine_AVX_LM(const Vec5& params, const float& s_far, const float& lx, const float& ly, float& err_sse);

    void simd_updateNormal_SSE(const __m128& J1,
        const __m128& res, const __m128& weight, float& err_sse);
    void simd_updateAffine_SSE(const __m128& J1, const __m128& J2, const __m128& J3, const __m128& J4, const __m128& J5,
        const __m128& res, const __m128& weight, float& err_sse);
    void simd_updateAffine_AVX(const __m256& J1, const __m256& J2, const __m256& J3, const __m256& J4, const __m256& J5,
        const __m256& res, const __m256& weight, float& err_sse);
    
    void simd_solveGaussNewtonStepNormal(float& delta_s);
    void simd_solveGaussNewtonStepAffine(Vec5& delta_params);
    void simd_solveGaussNewtonStepAffine_LM(const float& lam, Vec5& delta_params);


private: 
    void generatePattern(
        int win_half, vector<cv::Point2f>& patch, bool flag_fastpattern);
    

};

#endif