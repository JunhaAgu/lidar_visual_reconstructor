#ifndef _LIDAR_VISUAL_RECONSTRUCTOR_H_
#define _LIDAR_VISUAL_RECONSTRUCTOR_H_


#include <iostream>
#include <exception>
#include <vector>

// ROS
#include <ros/ros.h>

// ROS eigen
#include <Eigen/Dense>

// ROS cv_bridge
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>


// For subscribe
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h> 
#include <image_transport/image_transport.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Custom messages and services
#include "hce_autoexcavator/LidarImageDataStamped.h" // client
#include "hce_autoexcavator/ProfilePointsStamped.h" // server for 'GCS node'

#include "lidar_pcl.hpp"
#include "frame.hpp"
#include "camera.hpp"
#include "feature_tracker.h"


using namespace std;
class LidarVisualReconstructor {
public:
    LidarVisualReconstructor(ros::NodeHandle& nh);
    ~LidarVisualReconstructor();

    void loadCameraIntrinsics(string& dir);
    void loadSensorExtrinsics(string& dir);

    bool run(); // success or fail.
    
private:
    ros::NodeHandle nh_;
    ros::ServiceClient client_lidarimagedata_;

    hce_autoexcavator::LidarImageDataStamped srv_lidarimagedata_;

// Callback function
private:
    bool serverCallbackProfilePoints(hce_autoexcavator::ProfilePointsStamped::Request &req,
        hce_autoexcavator::ProfilePointsStamped::Response &res);

// Tracker
private:
    FeatureTracker* tracker_;
    // ConstraindDelaunayTriangulation* cdt_;

// pcl
private:
    int n_lidars_;  // 2
    vector<int> n_channels_;
    vector<LidarPcl*> pcls_;

// cams & frames
private:
    int n_cameras_; // 4
    int MAX_LVL_PYR_;;
    vector<Camera*> cams_;
    vector<Frame*> frames_;

// extrinsics
private:
    vector<Eigen::Matrix4f> T_cl0_;
    vector<Eigen::Matrix4f> T_c0c1_;

};
#endif