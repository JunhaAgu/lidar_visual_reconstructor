#ifndef _POINT_DATABASE_HPP_
#define _POINT_DATABASE_HPP_

#include <iostream>
#include <vector>
#include <Eigen/Dense>


struct PointDB {
    int state_;
    float depth_lidar_;           // LiDAR-based depths for 'points_constraints_'
    float depth_recon_;           // estimated depths for 'points_constraints_'
    float err_klt_normal_;
    float err_klt_affine_;
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

    PointDB(){
        state_ = -1; // 0: augmented, 1: intersection, -1: not initialized
        depth_lidar_ = -1;
        depth_recon_ = -1;
        err_klt_normal_ = -1;
        err_klt_affine_ = -1;
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
    };
};
#endif