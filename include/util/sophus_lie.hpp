#ifndef _SOPHUS_LIE_H_
#define _SOPHUS_LIE_H_

#include <iostream>
#include <vector>
#include <cmath>

#include "opencv2/opencv.hpp"  
#include "opencv2/core/ocl.hpp"
#include "Eigen/Dense"
#include "Eigen/Geometry"

namespace sophuslie {
	void se3Exp(const Eigen::Matrix<float,6,1>& xi, Eigen::Matrix4d& T);
	void se3Exp(const Eigen::MatrixXd& xi_, Eigen::MatrixXd& T_);
	void SE3Log(const Eigen::MatrixXf& T, Eigen::MatrixXf& xi);
	void SE3Log(const Eigen::MatrixXd& T_, Eigen::MatrixXd& xi_);
	void a2r(const float& r, const float& p, const float& y, Eigen::Matrix3f& R);
	void a2r(const double& r, const double& p, const double& y, Eigen::Matrix3d& R);
    void r2a(const Eigen::Matrix3d& R, double& r, double& p, double& y);
    void r2a(const Eigen::Matrix3f& R, float& r, float& p, float& y);

    Eigen::Matrix3d hat(const Eigen::Vector3d& vec);
};
#endif