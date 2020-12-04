#include <ros/ros.h>
#include <iostream>
#include <time.h>
#include <string>
#include <sstream>



using namespace std;
   
int main(int argc, char **argv) {
    ros::init(argc, argv, "vis_recon");
    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("START: \"vis_recon\".\n");
    ros::spin();

    ROS_INFO_STREAM("TERMINATE: \"vis_recon\".\n");
    return -1;
}
