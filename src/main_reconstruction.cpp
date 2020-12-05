#include <ros/ros.h>
#include <iostream>
#include <exception>
#include <time.h>
#include <string>
#include <sstream>

#include "lidar_visual_reconstructor.hpp"

using namespace std;
int main(int argc, char **argv) {
    ros::init(argc, argv, "reconstruction_node");
    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("START: \"reconstruction_node\".\n");
    
    LidarVisualReconstructor* recon = nullptr;

    try {
        recon = new LidarVisualReconstructor(nh);

        while(ros::ok()){
            recon->run();
        }
    }
    catch(std::exception& e){
        cout << "\n[Exception]: " << e.what();
        if(recon != nullptr) delete recon;
    }


    ROS_INFO_STREAM("TERMINATE: \"reconstruction_node\".\n");
    return -1;
}
