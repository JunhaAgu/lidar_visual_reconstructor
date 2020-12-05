#include "lidar_visual_reconstructor.hpp"

LidarVisualReconstructor::LidarVisualReconstructor(ros::NodeHandle& nh)
: nh_(nh)
{
    nh_.serviceClient<hce_autoexcavator::LidarImageDataStamped>("srv_lidar_image_data");

};
LidarVisualReconstructor::~LidarVisualReconstructor(){

};
bool LidarVisualReconstructor::run(){
    if(client_lidarimagedata_.call(srv_lidarimagedata_)){
        ROS_INFO("Service is requested by 'lidar_visual_reconstructor' node.\n");
        return true;
    }
    else{
        ROS_ERROR("Failed to call service by 'lidar_visual_reconstructor' node.\n");
        return false;
    }
};