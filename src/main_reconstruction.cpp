#include <ros/ros.h>
#include <iostream>
#include <time.h>
#include <string>
#include <sstream>



using namespace std;
   
int main(int argc, char **argv) {
    ros::init(argc, argv, "reconstruction_node");
    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("START: \"reconstruction_node\".\n");
    ros::spin();

    ROS_INFO_STREAM("TERMINATE: \"reconstruction_node\".\n");
    return -1;
}
