#include "can/can_communicator.hpp"

using namespace constants;

CanCommunicator::CanCommunicator(ros::NodeHandle& nh, string tpcname_from_ardu, string tpcname_to_ardu)
:nh_(nh), topicname_from_arduino_(tpcname_from_ardu), topicname_to_arduino_(tpcname_to_ardu)
{
    cout << "CAN communicator starts.\n";
    pub_to_ex_   = nh_.advertise<hce_autoexcavator::packetsToExcavator>(
        topicname_from_arduino_, 1);
    sub_from_ex_ = nh_.subscribe<hce_autoexcavator::packetsFromExcavator>(
        topicname_to_arduino_, 1, &CanCommunicator::callbackFromExcavator, this);

    butterworth_cnt_ = 0;
};

CanCommunicator::~CanCommunicator()
{
    cout << "can communicator stops.\n";
};


#define BUTTERWORTH(x) ( 1.691 * x[1] - 0.7327 * x[2] + 1.04 * (x[0] - x[1]) + 2.09 * (x[1] - x[2]) + 1.04 * (x[2] - x[3]) )

void CanCommunicator::callbackFromExcavator(const hce_autoexcavator::packetsFromExcavatorConstPtr &msg_from_ex)
{
    int n_bytes = msg_from_ex->n_bytes;
    cout << "subscriber callback ... n_bytes: " << n_bytes << "\n";

    float pow16_2 = pow16_2;
    float pow16_4 = pow16_4;

    //Sensor1
    fromEx_.ACyl_LC = 0.1 * (float)msg_from_ex->bytes[SENSOR1_START + 1] * pow16_2 + (float)msg_from_ex->bytes[SENSOR1_START + 0];
    fromEx_.ACyl_SC = 0.1 * (float)msg_from_ex->bytes[SENSOR1_START + 3] * pow16_2 + (float)msg_from_ex->bytes[SENSOR1_START + 2];
    fromEx_.Swing_L = 0.1 * (float)msg_from_ex->bytes[SENSOR1_START + 5] * pow16_2 + (float)msg_from_ex->bytes[SENSOR1_START + 4];
    fromEx_.Swing_R = 0.1 * (float)msg_from_ex->bytes[SENSOR1_START + 7] * pow16_2 + (float)msg_from_ex->bytes[SENSOR1_START + 6];
    //Sensor2
    fromEx_.BCyl_LC = 0.1 * (float)msg_from_ex->bytes[SENSOR2_START + 1] * pow16_2 + (float)msg_from_ex->bytes[SENSOR2_START + 0];
    fromEx_.BCyl_SC = 0.1 * (float)msg_from_ex->bytes[SENSOR2_START + 3] * pow16_2 + (float)msg_from_ex->bytes[SENSOR2_START + 2];
    fromEx_.KCyl_LC = 0.1 * (float)msg_from_ex->bytes[SENSOR2_START + 5] * pow16_2 + (float)msg_from_ex->bytes[SENSOR2_START + 6];
    fromEx_.KCyl_SC = 0.1 * (float)msg_from_ex->bytes[SENSOR2_START + 7] * pow16_2 + (float)msg_from_ex->bytes[SENSOR2_START + 8];
    //Sensor3
    fromEx_.Body_Pitch_Angle = angle_scale * ((float)msg_from_ex->bytes[SENSOR3_START + 2] * pow16_4 + (float)msg_from_ex->bytes[SENSOR3_START + 1] * pow16_2 + (float)msg_from_ex->bytes[SENSOR3_START + 0]) - angle_offset;
    fromEx_.Body_Roll_Angle  = angle_scale * ((float)msg_from_ex->bytes[SENSOR3_START + 5] * pow16_4 + (float)msg_from_ex->bytes[SENSOR3_START + 4] * pow16_2 + (float)msg_from_ex->bytes[SENSOR3_START + 3]) - angle_offset;
    //Sensor4
    fromEx_.Boom_Joint_Angle = angle_scale * ((float)msg_from_ex->bytes[SENSOR4_START + 2] * pow16_4 + (float)msg_from_ex->bytes[SENSOR4_START + 1] * pow16_2 + (float)msg_from_ex->bytes[SENSOR4_START + 0]) - angle_offset;
    //Sensor5
    fromEx_.Arm_Joint_Angle  = angle_scale * ((float)msg_from_ex->bytes[SENSOR5_START + 2] * pow16_4 + (float)msg_from_ex->bytes[SENSOR5_START + 1] * pow16_2 + (float)msg_from_ex->bytes[SENSOR5_START + 0]) - angle_offset;
    //Sensor6
    fromEx_.Bkt_Joint_Angle  = angle_scale * ((float)msg_from_ex->bytes[SENSOR6_START + 2] * pow16_4 + (float)msg_from_ex->bytes[SENSOR6_START + 1] * pow16_2 + (float)msg_from_ex->bytes[SENSOR6_START + 0]) - angle_offset;
    //Sensor7
    fromEx_.Swing_Angle      = angle_scale * ((float)msg_from_ex->bytes[SENSOR7_START + 2] * pow16_4 + (float)msg_from_ex->bytes[SENSOR7_START + 1] * pow16_2 + (float)msg_from_ex->bytes[SENSOR7_START + 0]) - angle_offset;

    fromEx_.Boom_Angle_buf[0]  = fromEx_.Boom_Joint_Angle;
    fromEx_.Arm_Angle_buf[0]   = fromEx_.Arm_Joint_Angle;
    fromEx_.Bkt_Angle_buf[0]   = fromEx_.Bkt_Joint_Angle;
    fromEx_.Swing_Angle_buf[0] = fromEx_.Swing_Angle;

    if (butterworth_cnt_ > 3)
    {
        // Butterworth filter.
        fromEx_.Boom_Joint_Rate = BUTTERWORTH(fromEx_.Boom_Rate_buf);
        fromEx_.Boom_Rate_buf[2]  = fromEx_.Boom_Rate_buf[1];
        fromEx_.Boom_Rate_buf[1]  = fromEx_.Boom_Rate_buf[0];
        fromEx_.Boom_Rate_buf[0]  = fromEx_.Boom_Joint_Rate;

        fromEx_.Arm_Joint_Rate  = BUTTERWORTH(fromEx_.Arm_Rate_buf);
        fromEx_.Arm_Rate_buf[2]   = fromEx_.Arm_Rate_buf[1];
        fromEx_.Arm_Rate_buf[1]   = fromEx_.Arm_Rate_buf[0];       
        fromEx_.Arm_Rate_buf[0]   = fromEx_.Arm_Joint_Rate;

        fromEx_.Bkt_Joint_Rate  = BUTTERWORTH(fromEx_.Bkt_Rate_buf);
        fromEx_.Bkt_Rate_buf[2]   = fromEx_.Bkt_Rate_buf[1];
        fromEx_.Bkt_Rate_buf[1]   = fromEx_.Bkt_Rate_buf[0];
        fromEx_.Bkt_Rate_buf[0]   = fromEx_.Bkt_Joint_Rate;
        
        fromEx_.Swing_Rate      = BUTTERWORTH(fromEx_.Swing_Rate_buf);
        fromEx_.Swing_Rate_buf[2] = fromEx_.Swing_Rate_buf[1];
        fromEx_.Swing_Rate_buf[1] = fromEx_.Swing_Rate_buf[0];
        fromEx_.Swing_Rate_buf[0] = fromEx_.Swing_Rate;
    }

    fromEx_.Boom_Angle_buf[3]  = fromEx_.Boom_Angle_buf[2];
    fromEx_.Boom_Angle_buf[2]  = fromEx_.Boom_Angle_buf[1];
    fromEx_.Boom_Angle_buf[1]  = fromEx_.Boom_Angle_buf[0];

    fromEx_.Arm_Angle_buf[3]   = fromEx_.Arm_Angle_buf[2];
    fromEx_.Arm_Angle_buf[2]   = fromEx_.Arm_Angle_buf[1];
    fromEx_.Arm_Angle_buf[1]   = fromEx_.Arm_Angle_buf[0];

    fromEx_.Bkt_Angle_buf[3]   = fromEx_.Bkt_Angle_buf[2];
    fromEx_.Bkt_Angle_buf[2]   = fromEx_.Bkt_Angle_buf[1];
    fromEx_.Bkt_Angle_buf[1]   = fromEx_.Bkt_Angle_buf[0];

    fromEx_.Swing_Angle_buf[3] = fromEx_.Swing_Angle_buf[2];
    fromEx_.Swing_Angle_buf[2] = fromEx_.Swing_Angle_buf[1];
    fromEx_.Swing_Angle_buf[1] = fromEx_.Swing_Angle_buf[0];

    ++butterworth_cnt_;
};


void CanCommunicator::calWithFactor(float *input, float *array)
{
    array[0] = (input[0] - off_psi_U) * inv_scale_psi_U;
    array[1] = (input[1] - off_L_B) * inv_scale_L_B;
    array[2] = (input[2] - off_L_A) * inv_scale_L_A;
    array[3] = (input[3] - off_L_K) * inv_scale_L_K;
    array[4] = (input[4] - off_psi_Udot) * inv_scale_psi_Udot;
    array[5] = (input[5] - off_L_Bdot) * inv_scale_L_Bdot;
    array[6] = (input[6] - off_L_Adot) * inv_scale_L_Adot;
    array[7] = (input[7] - off_L_Kdot) * inv_scale_L_Kdot;
    array[8] = (input[8] - off_T_U) * inv_scale_T_U;
    array[9] = (input[9] - off_F_B) * inv_scale_F_B;
    array[10] = (input[10] - off_F_A) * inv_scale_F_A;
    array[11] = (input[11] - off_F_K) * inv_scale_F_K;
};


void CanCommunicator::encodeToEx(
    float *array0, float *array1, float *array2, BYTE *array_to_ex)
{
    float X0;
    int h3, h2, h1, h0;
    for (int i = 0; i < 36; ++i)
    {
        if (i < 12)
        {
            X0 = array0[i]; //real number
        }
        else if (i < 24)
        {
            X0 = array1[i - 12]; //real number
        }
        else
        {
            X0 = array2[i - 24]; //real number
        }
        // save considering Intel byte order
        h3 = (X0 / pow(16, 3)); //integer
        h2 = (X0 - h3 * 4096) / pow(16, 2); //integer
        h1 = (X0 - h3 * 4096 - h2 * 256) / pow(16, 1); //integer
        h0 = (X0 - h3 * 4096 - h2 * 256 - h1 * 16); //integer
        array_to_ex[2 * i] = 16 * h1 + h0;
        array_to_ex[2 * i + 1] = 16 * h3 + h2;
    }
};


void CanCommunicator::publishToExcavator(){
    msg_to_ex_.n_bytes = 0;
    msg_to_ex_.bytes.clear();
    
    this->calWithFactor(planner_outputs0_, planner_outputs0_processed_); // 0.2 s 
    this->calWithFactor(planner_outputs1_, planner_outputs1_processed_); // 0.5 s
    this->calWithFactor(planner_outputs2_, planner_outputs2_processed_); // 1.0 s 

    this->encodeToEx(
        planner_outputs0_processed_,
        planner_outputs1_processed_,
        planner_outputs2_processed_,
        array_to_ex);

    for(int i = 0; i < 72; ++i){
        msg_to_ex_.bytes.push_back(array_to_ex[i]);
    }

    msg_to_ex_.n_bytes = msg_to_ex_.bytes.size();
    pub_to_ex_.publish(msg_to_ex_);
};