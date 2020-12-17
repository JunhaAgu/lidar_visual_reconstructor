#ifndef _POINT_DATABASE_HPP_
#define _POINT_DATABASE_HPP_

#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/SVD>

using namespace std;

struct PointDB {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int state_;
    float depth_lidar_;           // LiDAR-based depths for 'points_constraints_'
    float depth_recon_;           // estimated depths for 'points_constraints_'
    float std_depth_;
    float err_klt_normal_;
    float err_klt_affine_;
    float s_normal_;
    float s_affine_;
    bool klt_valid_;
    Eigen::Vector3f X_;           // augmented by densification step.
    Eigen::Vector3f X_recon_;     // reconstructed 3D point.
    Eigen::Vector2f pts_;         // 2D projected pixel points of 'points_constraints_'.
    Eigen::Vector4i info_;        // used for densification.

    Eigen::Vector2f pts_prior_;
    Eigen::Vector2f pts_guess_near_;
    Eigen::Vector2f pts_guess_far_;
    Eigen::Vector2f pts_tracked_;
    Eigen::Vector4f abcd_;        // abcd interest (p)

    int iter_normal_;
    int iter_affine_;

    PointDB(){
        state_ = -1; // 0: augmented, 1: intersection, -1: not initialized
        depth_lidar_ = -1;
        depth_recon_ = -1;
        std_depth_ = -1;
        err_klt_normal_ = -1;
        err_klt_affine_ = -1;
        s_normal_ = -1;
        s_affine_ = -1;
        klt_valid_ = true;
        X_ << -1,-1,-1;
        X_recon_ << -1,-1,-1;
        pts_ << -1,-1;
        info_ << -1,-1,-1,-1;
        pts_tracked_ << -1,-1;
        pts_prior_ << -1,-1;
        pts_guess_near_ << -1, -1;
        pts_guess_far_ << -1, -1;
        abcd_ << -1,-1,-1,-1;

        iter_normal_ = 0;
        iter_affine_ = 0;
    };

    void recon3D(const Eigen::Matrix3f& K1, const Eigen::Matrix3f& K2, 
        const Eigen::Matrix3f& R21, const Eigen::Vector3f& t21)
    {
        Eigen::Matrix4f A;
        Eigen::Matrix4f P1, P2;
        P1 << K1, Eigen::Vector3f::Zero();
        P2 << K2*R21, K2*t21;

        A << pts_(1)*P1.block<1,4>(2,0)-P1.block<1,4>(1,0),
             P1.block<1,4>(0,0)-pts_(0)*P1.block<1,4>(2,0),
             pts_tracked_(1)*P2.block<1,4>(2,0)-P2.block<1,4>(1,0),
             P2.block<1,4>(0,0)-pts_tracked_(0)*P2.block<1,4>(2,0);
        
        
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, 
                            Eigen::ComputeThinU | Eigen::ComputeThinV);
        //cout << "Its singular values are:" << endl << svd.singularValues() << endl;
        //cout << "Its left singular vectors are the columns of the thin U matrix:" << endl << svd.matrixU() << endl;
        //cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;
        X_recon_ << svd.matrixV().block<3,1>(0,3)/svd.matrixV()(3,3);
        depth_recon_ = X_recon_(2);
    };
};
#endif