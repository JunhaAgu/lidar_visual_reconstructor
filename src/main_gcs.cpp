#include <ros/ros.h>
#include <iostream>
#include <time.h>
#include <string>
#include <sstream>
#include <exception>
#include <chrono>

// keyboard input tool
#include "util/keyinput.h"
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

    int n_cameras = 4;
    int n_lidars  = 2;

    HCEGCS* gcs = nullptr;

    try {
        gcs = new HCEGCS(nh, n_cameras, n_lidars, std::string(""));
        // user input manual.
        string user_manual;
        stringstream ss;

        ss << "\n==============================================\n|" 
        << "  Select..." 
        << "\n|      [s]  Query and save a snapshot." 
        << "\n|      [r]  Run All HCE algorithms" 
        << "\n|      [t]  Get test dataset" 
        << "\n|      [c]  [TEST] test CAN 10 Hz signals publishing"
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
                
                gcs->runAlgorithms(); // To be updated.

                cout << user_manual;
            }
            else if(c == 't') {
                cout << "\n\n Get test data...\n";
                string dir_testdata = "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/test_data/up_20";
                
                float theta  = -(20.0f-9.9f); // 20 degrees for up 20. (ref angle: 9.9 deg)
                float L_boom = 4.661836021f; // [m]
                Eigen::Vector3f w;
                w << -1.12196576,1.371003368,1.439545800;
                Eigen::Matrix<float,6,1> xi_bf;
                xi_bf << 0.855383628, 0.834228799, 1.503234857, -1.57548926, 0.318537783, -0.08536957;
                gcs->setTestLidarImages(dir_testdata, theta*3.141592f/180.0f, L_boom, w, xi_bf);
                cout << user_manual;
            }
            else if(c == 'c'){
                cout << "\n\n [TEST] test CAN 10 Hz signals publishing\n";
                cout << "10 Hz (forced) CAN publisher test mode\n";
                cout << " Press 'c' to stop... \n";


                auto t_past = chrono::high_resolution_clock::now();
                auto t_now = t_past;
                while(1) {
                    int cc = getch(); 
                    if(cc == 'c') break;

                    t_now = chrono::high_resolution_clock::now();
                    auto gap = t_now - t_past;
                    double dt = (double)(gap/chrono::microseconds(1))/1000.0;

                    // 10 Hz publishing.
                    if(dt > 100){
                        gcs->testCan10hzPublisher();
                        t_past = t_now;
                    }
                    ros::spinOnce();
                }

                cout << "\n\nCAN test mode stops.\n";
                cout << user_manual;
            }
            else if(c != 0) {
                cout << ": Un-identified command...\n";
                cout << user_manual;
            }
        }
    }
    catch (std::exception& e) {
        cout << "\n[Exception]: " << e.what();
        if(gcs != nullptr) delete gcs;
    }

    
    ROS_INFO_STREAM("TERMINATE: \"communicator\".\n");
    return -1;
}
