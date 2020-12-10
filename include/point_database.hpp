#ifndef _POINT_DATABASE_HPP_
#define _POINT_DATABASE_HPP_

#include <iostream>
#include <vector>
#include <Eigen/Dense>


struct PointDB {
    int state_;
    float depth_;                 // estimated depths for 'points_constraints_'
    Eigen::Vector3f X_;           // augmented by densification step.
    Eigen::Vector3f X_recon_;     // reconstructed 3D point.
    Eigen::Vector2f pts_;         // 2D projected pixel points of 'points_constraints_'.
    Eigen::Vector4i info_;        // used for densification.
    Eigen::Vector2f pts_tracked_;
    Eigen::Vector4f abcd_;        // abcd interest (p)

    PointDB(){
        state_ = -1; // 0: augmented, 1: intersection, -1: not initialized
        depth_ = -1;
        X_ << -1,-1,-1;
        X_recon_ << -1,-1,-1;
        pts_ << -1,-1;
        info_ << -1,-1,-1,-1;
        pts_tracked_ << -1,-1;
        abcd_ << -1,-1,-1,-1;
    };
};
#endif