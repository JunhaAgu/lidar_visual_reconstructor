#include <ros/ros.h>
#include <iostream>
#include <time.h>
#include <string>
#include <sstream>
#include <exception>

// keyboard input tool
#include "keyinput.h"
#include "hcegcs.h"


using namespace std;
// Get current data/time, format is yyyy-mm-dd.hh:mm:ss
const std::string currentDateTime(){
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about data/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d_%H_%M_%S", &tstruct);

    return buf;
}
   
int main(int argc, char **argv) {
    ros::init(argc, argv, "gcs_node");
    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("START: \"communicator\".\n");

    int n_cameras = 0;
    int n_lidars = 0;

    HCEGCS* gcs = nullptr;

    try {
        gcs = new HCEGCS(nh, 0, 0, std::string(""));
        // user input manual.
        string user_manual;
        stringstream ss;

        ss << "\n==============================================\n|" 
        << "  Select..." 
        << "\n|      [s]  Query and save a snapshot." 
        << "\n|      [r]  Run All HCE algorithms" 
        << "\n| Select an input: \n";
        user_manual = ss.str();
        cout << user_manual;
        ss.clear();
        ss.flush();

        ss << "\n |\n L\n";

        while(ros::ok()){
            ros::spinOnce();
            int c = getch(); // call my own non-blocking input function
            if(c == 's') {
                cout << "\n\n Snapshot & save the current scene.\n";
                gcs->snapshotMode();

                cout << user_manual;
            }
            else if(c == 'r'){
                cout << "\n\n RUN algorithms...\n";
                // Do algorithm parts... GCS (Ground Control System)
                // Each module should be a 'class' object.

                cout << user_manual;
            }
            else if(c != 0) {
                cout << ": Un-identified command...\n";
                cout << user_manual;
            }
        }
    }
    catch (std::exception& e){
        cout << "\n[Exception]: " << e.what();
        if(gcs != nullptr) delete gcs;
    }

    
    ROS_INFO_STREAM("TERMINATE: \"communicator\".\n");
    return -1;
}
