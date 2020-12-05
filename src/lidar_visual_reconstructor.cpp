#include "lidar_visual_reconstructor.hpp"

LidarVisualReconstructor::LidarVisualReconstructor(ros::NodeHandle& nh)
: nh_(nh)
{
    nh_.serviceClient<hce_autoexcavator::LidarImageDataStamped>("srv_lidar_image_data");

};
LidarVisualReconstructor::~LidarVisualReconstructor(){

};

bool LidarVisualReconstructor::serverCallbackProfilePoints(hce_autoexcavator::ProfilePointsStamped::Request &req,
        hce_autoexcavator::ProfilePointsStamped::Response &res)
{   
    // run algorithm.
    this->run();

    // Fill out response for 'GCS' node.

};

// 'run()' function is executed when 'GCS' requests profile 3D points.
// Thus, this function should be in the 'callback' function for service.
bool LidarVisualReconstructor::run(){
    if(client_lidarimagedata_.call(srv_lidarimagedata_)){
        ROS_INFO("Service is requested by 'lidar_visual_reconstructor' node.\n");

        // 
        
        // Delaunay ... 

        // KLT ...

        // Densification ...

        // Extract profile 3D points ... 

        // Respond to 'GCS' node.
        

        return true;
    }
    else{
        ROS_ERROR("Failed to call service by 'lidar_visual_reconstructor' node.\n");
        return false;
    }
};