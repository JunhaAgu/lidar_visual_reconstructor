#include "camera.hpp"
using namespace std;
Camera::Camera(int n_cols, int n_rows, float fx, float fy, float cx, float cy, float k1, float k2, float k3, float p1, float p2)
{   
    n_cols_ = n_cols;
    n_rows_ = n_rows;
    fx_ = fx;    fy_ = fy;    cx_ = cx;    cy_ = cy;
    fxinv_ = 1.0f/fx_; fyinv_ = 1.0f/fy_;
    k1_ = k1;    k2_ = k2;    k3_ = k3;
    p1_ = p1;    p2_ = p2;
    distortion_[0] = k1;
    distortion_[1] = k2;
    distortion_[2] = p1;
    distortion_[3] = p2;
    distortion_[4] = k3;

    // initialize all things
    K_        = Eigen::Matrix3f::Identity();
    Kinv_     = Eigen::Matrix3f::Identity();

    K_ << fx_, 0.0f, cx_, 0.0f, fy_, cy_, 0.0f, 0.0f, 1.0f;
    Kinv_ = K_.inverse();

    cout << " K:\n";
    cout << K_;
    cout << " distortion: " << k1<<"," << k2 <<"," << k3<<"," <<p1 <<"," <<p2 <<"\n";
};

Camera::~Camera(){
    cout << "Camera is deleted.\n";
};

void Camera::generateUndistortMaps(){
    float* map_x_ptr = nullptr;
    float* map_y_ptr = nullptr;
    float x, y, r, r2, r4, r6, r_radial, x_dist, y_dist;
    
    for(int v = 0; v < n_rows_; ++v){
        map_x_ptr = undist_map_x_.ptr<float>(v);
        map_y_ptr = undist_map_y_.ptr<float>(v);
        y = (v - cy_) * fyinv_;

        for(int u = 0; u < n_cols_; ++u){
            x = (u - cx_) * fxinv_;
            r = sqrtf(x*x + y*y);
            r2 = r*r;
            r4 = r2*r2;
            r6 = r4*r2;
            r_radial = 1.0f + k1_*r2 + k2_*r4 + k3_*r6;
            x_dist = x*r_radial * 2 * p1_ * x * y + p2_*(r2 + 2*x*x);
            y_dist = y*r_radial + p1_*(r2 + 2* y*y) + 2*p2_*x*y;

            *(map_x_ptr + u) = cx_ + x_dist*fx_;
            *(map_y_ptr + u) = cy_ + y_dist*fy_;
        }
    }
};

void Camera::undistortImage(const cv::Mat& raw, cv::Mat& rectified){
    if (raw.empty() || raw.type() != CV_8UC1 || raw.cols != n_cols_ || raw.rows != n_rows_)
        throw std::runtime_error("undistort image: provided image has not the same size as the camera model or image is not grayscale!\n");

    cv::remap(raw, rectified, this->undist_map_x_, this->undist_map_y_, CV_INTER_LINEAR);
};