#include "frame.hpp"

Frame::Frame(Camera* cam, int max_lvl_pyr)
: MAX_PYR_LVL_(max_lvl_pyr){
    cam_ = cam; // cam is well initialized already. (in the recon node)

    img_raw_ = cv::Mat::zeros(cam_->rows(), cam_->cols(), CV_8UC1);
    img_undist_ = cv::Mat::zeros(cam_->rows(), cam_->cols(), CV_8UC1);
    img_raw_f_ = cv::Mat::zeros(cam_->rows(), cam_->cols(), CV_32FC1);
    img_undist_f_ = cv::Mat::zeros(cam_->rows(), cam_->cols(), CV_32FC1);
    du_s_    = cv::Mat::zeros(cam_->rows(), cam_->cols(), CV_16SC1);
    dv_s_    = cv::Mat::zeros(cam_->rows(), cam_->cols(), CV_16SC1);
    d_tmp_   = cv::Mat::zeros(cam_->rows(), cam_->cols(), CV_32FC1);

    // preallocation
    img_pyr_.reserve(MAX_PYR_LVL_);
    du_pyr_.reserve(MAX_PYR_LVL_);
    dv_pyr_.reserve(MAX_PYR_LVL_);

    for(int lvl = 0; lvl < MAX_PYR_LVL_; ++lvl){
        if(lvl == 0){
            img_pyr_.emplace_back(cam_->rows(), cam_->cols(), CV_32FC1);
            du_pyr_.emplace_back(cam_->rows(), cam_->cols(), CV_32FC1);
            dv_pyr_.emplace_back(cam_->rows(), cam_->cols(), CV_32FC1);
        }
        else {
            int n_cols_tmp = (int)ceil(cam_->cols()/ pow(2, lvl));
            int n_rows_tmp = (int)ceil(cam_->rows()/ pow(2, lvl));
            img_pyr_.emplace_back(n_rows_tmp, n_cols_tmp, CV_32FC1);
            du_pyr_.emplace_back( n_rows_tmp, n_cols_tmp, CV_32FC1);
            dv_pyr_.emplace_back( n_rows_tmp, n_cols_tmp, CV_32FC1);
            // cout <<"lvl: " << lvl << ", cols and rows: " << n_cols_tmp << "," << n_rows_tmp <<"\n";
        }
    }
};

Frame::~Frame(){
    cam_ = nullptr; // detach!
};

void Frame::constructFrame(const cv::Mat& img_input){
    // Calculate gradients
    float sobel_num = 1;

    // Copy image.
    img_input.copyTo(img_raw_);
    img_raw_.convertTo(img_raw_f_, CV_32FC1);

    // undistort image
    cam_->undistortImage(img_raw_, img_undist_);
    
    img_undist_.convertTo(img_undist_f_, CV_32FC1);

    improc::imagePyramid( img_undist_f_, img_pyr_);
    calcGradient();
    cout << "frame is successfully constructed.\n";

};

void Frame::calcGradient(){
    // After img_raw_ and img_raw_f_ are initialized.
    cv::Sobel(img_undist_, this->du_s_, CV_16SC1, 1, 0);
    cv::Sobel(img_undist_, this->dv_s_, CV_16SC1, 0, 1);

    improc::diffImage(img_undist_f_, d_tmp_, 1, 0);
    improc::imagePyramid(d_tmp_, du_pyr_);
    improc::diffImage(img_undist_f_, d_tmp_, 0, 1);
    improc::imagePyramid(d_tmp_, dv_pyr_);

};
