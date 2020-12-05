#ifndef _HCEGCS_H_
#define _HCEGCS_H_

#include <iostream>
#include <vector>
#include <stdlib.h>

// File IO. for lidar pcd files
#include <string>
#include <sstream>
#include <fstream> 

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

#include <std_msgs/Int32.h> // command message to MCU
#include <sensor_msgs/TimeReference.h> // arduino time
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Custom messages and services
#include "hce_autoexcavator/ControlInputsStamped.h" // msg

#include "hce_autoexcavator/LidarImageDataStamped.h" // service server
#include "hce_autoexcavator/ProfilePolynomialStamped.h" // service (server to 'planner')
#include "hce_autoexcavator/ProfilePointsStamped.h" // client of 'vis_recon'

using namespace std;

class HCEGCS {
// Public methods
public:
    HCEGCS(ros::NodeHandle& nh, int n_cams, int n_lidars, const string& save_dir);
    ~HCEGCS();

    void streamingMode();
    void snapshotMode();
    void runAlgorithms();

    const int getNumCameras() const {return n_cameras_; };
    const int getNumLidars()  const {return n_lidars_; };



// ROS topic subs., pub., services. related variables
private:
    // node handler
    ros::NodeHandle nh_;

    // publishers
    ros::Publisher pub_msg_command_;
    std_msgs::Int32 msg_command_; // user command msg.
    
    // subscribers
    image_transport::ImageTransport it_;
    vector<image_transport::Subscriber> subs_imgs_; // from mvBlueCOUGAR-X cameras
    vector<ros::Subscriber> subs_lidars_; // from Velodyne lidars
    ros::Subscriber sub_timestamp_; // from arduino.

    // services for ground profile
    ros::ServiceServer server_ground_;
    ros::ServiceServer server_target_;

    // topic names
    vector<string> topicnames_imgs_;
    vector<string> topicnames_lidars_;
    string topicname_timestamp_;



// LiDAR and camera-related variables
private: 
    int n_cameras_; // numbering rule(cams) 0,1) cabin left,right, 2,3) boom frontal,rear
    int n_lidars_;// numbering rule(lidars)- 0) cabin, 1) boom

    // Transmittion flags.
    bool* flag_imgs_;   // 'true' when image data is received.
    bool* flag_lidars_; // 'true' when lidar data is received.
    bool  flag_mcu_;    // 'true' when arduino data is received. 

    // data container (buffer)
    cv::Mat* buf_imgs_; // Images from mvBlueCOUGAR-X cameras.
    double   buf_time_; // triggered time stamp from Arduino. [sec.]
    pcl::PointCloud<pcl::PointXYZI>::Ptr* buf_lidars_; // point clouds (w/ intensity) from Velodyne VLP16 
    
    vector<float*> buf_lidars_x;
    vector<float*> buf_lidars_y;
    vector<float*> buf_lidars_z;
    vector<float*> buf_lidars_intensity;
    vector<unsigned short*> buf_lidars_ring;
    vector<float*> buf_lidars_time;
    vector<int> buf_lidars_npoints;



// Callback functions
private:
    // request Profile points -> after 'planner's request, calculate and serve Profile Polynomial. 
    bool serverCallbackProfilePolynomial(hce_autoexcavator::ProfilePolynomialStamped::Request &req,
        hce_autoexcavator::ProfilePolynomialStamped::Response &res);

    // After 'lidar_visual_reconstructor's request, acquire a snapshot and send data to the 'lidar_visual_reconstructor'
    bool serverCallbackLidarImageData(hce_autoexcavator::LidarImageDataStamped::Request &req,
        hce_autoexcavator::LidarImageDataStamped::Response &res); // to reconstructor.


    void callbackImage(const sensor_msgs::ImageConstPtr& msg, const int& id);
    void callbackLidar(const sensor_msgs::PointCloud2ConstPtr& msg_lidar, const int& id);
    void callbackTime(const sensor_msgs::TimeReference::ConstPtr& t_ref);



// Private methods    
private:
    bool sendSingleQueryToAllSensors();
    void saveAllData();

    void pointcloud2tobuffers(const sensor_msgs::PointCloud2ConstPtr& msg_lidar, const int& id);
    void saveLidarDataRingTime(const std::string& file_name, const int& id);
    void clearMsgCommand(){msg_command_.data = 0; };
    void initAllFlags();

    string save_dir_;
    int current_seq_;
};




#endif