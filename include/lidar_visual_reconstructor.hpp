#ifndef _LIDAR_VISUAL_RECONSTRUCTOR_H_
#define _LIDAR_VISUAL_RECONSTRUCTOR_H_


#include <iostream>
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
#include "feature_tracker.h"


using namespace std;
class LidarVisualReconstructor {
public:
    LidarVisualReconstructor(ros::NodeHandle& nh);
    ~LidarVisualReconstructor();

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
    LidarPcl* pcls0;
    LidarPcl* pcls1;

};
#endif