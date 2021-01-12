#ifndef _CAN_MODULE_H_
#define _CAN_MODULE_H_

#include <ros/ros.h>
#include <iostream>
#include <time.h>
#include <string>
#include <sstream>

#include <vector>
#include <queue>

#include "can/can_message.hpp" // structures for data transmissions.

// Customized messages (from MCU, to MCU)
#include "hce_autoexcavator/packetsToExcavator.h"   // packets (bytes) to excavator (exactly, Arduino MCU)
#include "hce_autoexcavator/packetsFromExcavator.h" // packets (bytes) from excavator (exactly, Arduino MCU)


// Datatype redefine.
#define BYTE unsigned char


using namespace std;

class CanCommunicator{
public:
    CanCommunicator(ros::NodeHandle& nh, string tpcname_from_ardu, string tpcname_to_ardu);
    ~CanCommunicator();

public:
    FromExcvt fromEx_;
    ToExcvt toEx_;
    int butterworth_cnt_;

private:
    ros::NodeHandle nh_;
    string topicname_from_arduino_;
    string topicname_to_arduino_;

// messages and callback functions from Excavator (arduino)
private: 
    ros::Publisher  pub_to_ex_;
    ros::Subscriber sub_from_ex_; 
    hce_autoexcavator::packetsToExcavator   msg_to_ex_;
    hce_autoexcavator::packetsFromExcavator msg_from_ex_;
    
    float planner_outputs0_[12]; // inputs ? 
    float planner_outputs1_[12];
    float planner_outputs2_[12];

    float planner_outputs0_processed_[12];
    float planner_outputs1_processed_[12];
    float planner_outputs2_processed_[12];

    BYTE  array_to_ex_[72]; // 12 states * 3 times * 2 bytes = 72.

    void callbackFromExcavator(const hce_autoexcavator::packetsFromExcavatorConstPtr &msg_from_ex);

// message sender to excavator (Arduino)
public:
    void fillPlannerOutputs_TEST(); 
    void fillPlannerOutputs(); // TODO: planner...
    void publishToExcavator();

// Functions for processing packets
private:
    void calWithFactor(float *input, float *array);
    void encodeToEx(float *array0, float *array1, float *array2, BYTE *array_to_ex);

};


#endif