#include "util/sophus_lie.hpp"
/* =====================================================================
// =====================================================================
// ========================== IMPLEMENTATIONS ==========================
// =====================================================================
// =====================================================================*/
// * Verified functions compared to Matlab version *

void sophuslie::r2a(const Eigen::Matrix3d& R, double& r, double& p, double& y) 
{
    r = atan2(R(2, 1), R(2, 2));
    p = asin(-R(2, 0));
    y = atan2(R(1, 0), R(0, 0));
};

void sophuslie::r2a(const Eigen::Matrix3f& R, float& r, float& p, float& y)
{
    r = atan2(R(2, 1), R(2, 2));
    p = asin(-R(2, 0));
    y = atan2(R(1, 0), R(0, 0));
};


void sophuslie::se3Exp(const Eigen::Matrix<float, 6, 1>& xi_, Eigen::Matrix4d& T_)
{
	// initialize variables
	double theta = 0.0f;
	Eigen::Vector3d v, w;
	Eigen::Matrix3d wx, R, V;
	Eigen::Vector3d t;

	v(0) = xi_(0);
	v(1) = xi_(1);
	v(2) = xi_(2);

	w(0) = xi_(3);
	w(1) = xi_(4);
	w(2) = xi_(5);

	theta = std::sqrt(w.transpose() * w);
	wx << 0, -w(2), w(1),
		w(2), 0, -w(0),
		-w(1), w(0), 0;

	if (theta < 1e-7) {
		R = Eigen::MatrixXd::Identity(3, 3) + wx + 0.5 * wx * wx;
		V = Eigen::MatrixXd::Identity(3, 3) + 0.5 * wx + wx * wx / 3.0;
	}
	else {
		R = Eigen::MatrixXd::Identity(3, 3) + (sin(theta) / theta) * wx + ((1 - cos(theta)) / (theta*theta)) * (wx*wx);
		V = Eigen::MatrixXd::Identity(3, 3) + ((1 - cos(theta)) / (theta*theta)) * wx + ((theta - sin(theta)) / (theta*theta*theta)) * (wx*wx);
	}
	t = V * v;

	// assign rigid body transformation matrix (in SE(3))
	T_ = Eigen::MatrixXd::Zero(4, 4);
	T_(0, 0) = R(0, 0);
	T_(0, 1) = R(0, 1);
	T_(0, 2) = R(0, 2);

	T_(1, 0) = R(1, 0);
	T_(1, 1) = R(1, 1);
	T_(1, 2) = R(1, 2);

	T_(2, 0) = R(2, 0);
	T_(2, 1) = R(2, 1);
	T_(2, 2) = R(2, 2);

	T_(0, 3) = t(0);
	T_(1, 3) = t(1);
	T_(2, 3) = t(2);

	T_(3, 3) = 1.0;

	// for debug
	// std::cout << R << std::endl;
	// std::cout << t << std::endl;
	//usleep(10000000);
};


void sophuslie::se3Exp(const Eigen::MatrixXd& xi_, Eigen::MatrixXd& T_)
{
	// initialize variables
	double theta = 0.0;
	Eigen::Vector3d v, w;
	Eigen::Matrix3d wx, R, V;
	Eigen::Vector3d t;

	v(0) = xi_(0);
	v(1) = xi_(1);
	v(2) = xi_(2);

	w(0) = xi_(3);
	w(1) = xi_(4);
	w(2) = xi_(5);

	theta = std::sqrt(w.transpose() * w);
	wx << 0, -w(2), w(1),
		w(2), 0, -w(0),
		-w(1), w(0), 0;

	if (theta < 1e-7) {
		R = Eigen::MatrixXd::Identity(3, 3) + wx + 0.5 * wx * wx;
		V = Eigen::MatrixXd::Identity(3, 3) + 0.5 * wx + wx * wx / 3.0;
	}
	else {
		R = Eigen::MatrixXd::Identity(3, 3) + (sin(theta) / theta) * wx + ((1 - cos(theta)) / (theta*theta)) * (wx*wx);
		V = Eigen::MatrixXd::Identity(3, 3) + ((1 - cos(theta)) / (theta*theta)) * wx + ((theta - sin(theta)) / (theta*theta*theta)) * (wx*wx);
	}
	t = V * v;

	// assign rigid body transformation matrix (in SE(3))
	T_ = Eigen::MatrixXd::Zero(4, 4);
	T_(0, 0) = R(0, 0);
	T_(0, 1) = R(0, 1);
	T_(0, 2) = R(0, 2);

	T_(1, 0) = R(1, 0);
	T_(1, 1) = R(1, 1);
	T_(1, 2) = R(1, 2);

	T_(2, 0) = R(2, 0);
	T_(2, 1) = R(2, 1);
	T_(2, 2) = R(2, 2);

	T_(0, 3) = t(0);
	T_(1, 3) = t(1);
	T_(2, 3) = t(2);

	T_(3, 3) = 1.0;

	// for debug
	// std::cout << R << std::endl;
	// std::cout << t << std::endl;
	//usleep(10000000);
};

void sophuslie::se3Exp(const Eigen::Matrix<float,6,1>& xi_, Eigen::Matrix4f& T_){
	
	// initialize variables
	float theta = 0.0;
	Eigen::Vector3f v, w;
	Eigen::Matrix3f wx, R, V;
	Eigen::Vector3f t;

	v(0) = xi_(0);
	v(1) = xi_(1);
	v(2) = xi_(2);

	w(0) = xi_(3);
	w(1) = xi_(4);
	w(2) = xi_(5);

	theta = std::sqrt(w.transpose() * w);
	wx << 0, -w(2), w(1),
		w(2), 0, -w(0),
		-w(1), w(0), 0;

	if (theta < 1e-7) {
		R = Eigen::Matrix3f::Identity() + wx + 0.5 * wx * wx;
		V = Eigen::Matrix3f::Identity() + 0.5 * wx + wx * wx / 3.0f;
	}
	else {
		R = Eigen::Matrix3f::Identity() + (sin(theta) / theta) * wx + ((1 - cos(theta)) / (theta*theta)) * (wx*wx);
		V = Eigen::Matrix3f::Identity() + ((1 - cos(theta)) / (theta*theta)) * wx + ((theta - sin(theta)) / (theta*theta*theta)) * (wx*wx);
	}
	t = V * v;

	// assign rigid body transformation matrix (in SE(3))
	T_ = Eigen::Matrix4f::Zero();
	T_(0, 0) = R(0, 0);
	T_(0, 1) = R(0, 1);
	T_(0, 2) = R(0, 2);

	T_(1, 0) = R(1, 0);
	T_(1, 1) = R(1, 1);
	T_(1, 2) = R(1, 2);

	T_(2, 0) = R(2, 0);
	T_(2, 1) = R(2, 1);
	T_(2, 2) = R(2, 2);

	T_(0, 3) = t(0);
	T_(1, 3) = t(1);
	T_(2, 3) = t(2);

	T_(3, 3) = 1.0f;

	// for debug
	// std::cout << R << std::endl;
	// std::cout << t << std::endl;
	//usleep(10000000);
};

void sophuslie::SE3Log(const Eigen::MatrixXf& T_, Eigen::MatrixXf& xi_)
{
	float theta = 0.0f;
	float A, B;
	Eigen::Matrix3f wx, R;
	Eigen::MatrixXf lnR, Vin;
	Eigen::Vector3f t, w, v;

	R = T_.block<3, 3>(0, 0);
	t = T_.block<3, 1>(0, 3);


	theta = acosf((R.trace() - 1.0f)*0.5f);

	if (theta < 1e-9) {
		Vin = Eigen::MatrixXf::Identity(3, 3);
		w << 0, 0, 0;
	}
	else {
		lnR = (theta / (2 * sinf(theta)))*(R - R.transpose());
		w << -lnR(1, 2), lnR(0, 2), -lnR(0, 1);
		wx << 0, -w(2), w(1),
			w(2), 0, -w(0),
			-w(1), w(0), 0;
		A = sinf(theta) / theta;
		B = (1 - cosf(theta)) / (theta*theta);
		Vin = Eigen::MatrixXf::Identity(3, 3) - 0.5f*wx + (1 / (theta*theta))*(1 - A / (2 * B))*(wx*wx);
	}

	v = Vin*t;
	xi_ = Eigen::MatrixXf(6, 1);
	xi_(0) = v(0);
	xi_(1) = v(1);
	xi_(2) = v(2);
	xi_(3) = w(0);
	xi_(4) = w(1);
	xi_(5) = w(2);
	// for debug
	// std::cout << xi << std::endl;
};

void sophuslie::SE3Log(const Eigen::MatrixXd& T_, Eigen::MatrixXd& xi_)
{
	double theta = 0.0;
	double A, B;
	Eigen::Matrix3d wx, R;
	Eigen::MatrixXd lnR, Vin;
	Eigen::Vector3d t, w, v;

	R = T_.block<3, 3>(0, 0);
	t = T_.block<3, 1>(0, 3);


	theta = acos((R.trace() - 1.0)*0.5);

	if (theta < 1e-7) {
		Vin = Eigen::MatrixXd::Identity(3, 3);
		w << 0, 0, 0;
	}
	else {
		lnR = (theta / (2 * sin(theta)))*(R - R.transpose());
		w << -lnR(1, 2), lnR(0, 2), -lnR(0, 1);
		wx << 0, -w(2), w(1),
			w(2), 0, -w(0),
			-w(1), w(0), 0;

		A = sin(theta) / theta;
		B = (1 - cosf(theta)) / (theta*theta);
		Vin = Eigen::MatrixXd::Identity(3, 3) - 0.5*wx + (1 / (theta*theta))*(1 - A / (2 * B))*(wx*wx);
	}

	v = Vin*t;
	xi_ = Eigen::MatrixXd(6, 1);
	xi_(0) = v(0);
	xi_(1) = v(1);
	xi_(2) = v(2);
	xi_(3) = w(0);
	xi_(4) = w(1);
	xi_(5) = w(2);
	// for debug
	// std::cout << xi_ << std::endl;
};

void sophuslie::SE3Log(const Eigen::Matrix4f& T, Eigen::Matrix<float,6,1>& xi)
{
	float theta = 0.0;
	float A, B;
	Eigen::Matrix3f wx, R;
	Eigen::Matrix3f lnR, Vin;
	Eigen::Vector3f t, w, v;

	R = T.block<3, 3>(0, 0);
	t = T.block<3, 1>(0, 3);


	theta = acos((R.trace() - 1.0)*0.5);

	if (theta < 1e-7) {
		Vin = Eigen::Matrix3f::Identity();
		w << 0, 0, 0;
	}
	else {
		lnR = (theta / (2 * sin(theta)))*(R - R.transpose());
		w << -lnR(1, 2), lnR(0, 2), -lnR(0, 1);
		wx << 0, -w(2), w(1),
			w(2), 0, -w(0),
			-w(1), w(0), 0;

		A = sin(theta) / theta;
		B = (1 - cosf(theta)) / (theta*theta);
		Vin = Eigen::Matrix3f::Identity() - 0.5*wx + (1 / (theta*theta))*(1 - A / (2 * B))*(wx*wx);
	}

	v = Vin*t;
	xi(0) = v(0);
	xi(1) = v(1);
	xi(2) = v(2);
	xi(3) = w(0);
	xi(4) = w(1);
	xi(5) = w(2);
	// for debug
	// std::cout << xi_ << std::endl;
}


void sophuslie::a2r(const float& r, const float& p, const float& y, Eigen::Matrix3f& R) {// r,p,y are defined on the radian domain.
	Eigen::Matrix3f Rx, Ry, Rz;

	Rx << 1, 0, 0, 0, cos(r), -sin(r), 0, sin(r), cos(r);
	Ry << cos(p), 0, sin(p), 0, 1, 0, -sin(p), 0, cos(p);
	Rz << cos(y), -sin(y), 0, sin(y), cos(y), 0, 0, 0, 1;
	R = Rz*Ry*Rx;
};


void sophuslie::a2r(const double& r, const double& p, const double& y, Eigen::Matrix3d& R) {// r,p,y are defined on the radian domain.
	Eigen::Matrix3d Rx, Ry, Rz;

	Rx << 1, 0, 0, 0, cos(r), -sin(r), 0, sin(r), cos(r);
	Ry << cos(p), 0, sin(p), 0, 1, 0, -sin(p), 0, cos(p);
	Rz << cos(y), -sin(y), 0, sin(y), cos(y), 0, 0, 0, 1;
	R = Rz*Ry*Rx;
};

Eigen::Matrix3d sophuslie::hat(const Eigen::Vector3d& v)
{
    Eigen::Matrix3d res;
    res << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    return res;
};
