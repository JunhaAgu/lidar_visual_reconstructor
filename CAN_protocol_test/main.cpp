#include <ros/ros.h>
#include <iostream>
#include <time.h>
#include <string>
#include <sstream>

#include "tutorial_rosardu_customtopic/packetsToExcavator.h"   // custom messages
#include "tutorial_rosardu_customtopic/packetsFromExcavator.h" // custom messages

const float SENSOR1_START = 0;  //CAN ID 18FF4DE7
const float SENSOR2_START = 8;  //CAN ID 18FF61E7
const float SENSOR3_START = 16; //CAN ID 18F02971
const float SENSOR4_START = 22; //CAN ID 18F02972
const float SENSOR5_START = 25; //CAN ID 18F02973
const float SENSOR6_START = 28; //CAN ID 18F02974
const float SENSOR7_START = 31; //CAN ID 18F02970

using namespace std;

// FROM EXCAVATOR
// 18 states (14 raw incomings, 4 derived)
// 14 states (raw)
//0x18FF4DE7
float ACyl_LC; // arm cylinder pressure (large chamber)
float ACyl_SC; // arm cylinder pressure (small chamber)
float Swing_L; // swing pressure (left)
float Swing_R; // swing pressure (right)
//0x18FF61E7
float BCyl_LC; // boom cylinder pressure   (large chamber)
float BCyl_SC; // boom cylinder pressure   (small chamber)
float KCyl_LC; // bucket cylinder pressure (large chamber)
float KCyl_SC; // bucket cylinder pressure (small chamber)
//0x18F02971
float Body_Pitch_Angle;
float Body_Roll_Angle;
//0x18F02972
float Boom_Joint_Angle;
//0x18F02973
float Arm_Joint_Angle;
//0x18F02974
float Bkt_Joint_Angle;
//0x18F02970
float Swing_Angle;

// 4 derived (rates) (numerical derivatives by us)
float Boom_Joint_Rate;
float Arm_Joint_Rate;
float Bkt_Joint_Rate;
float Swing_Rate;

// receive message
const float angle_scale = 3.0517578125e-5;
const float angle_offset = -250;

int buf_cnt = 0;
bool flag_rate = false;

// Numerical derivatives 
float Boom_Angle_buf[4]  = {0, 0, 0, 0};
float Arm_Angle_buf[4]   = {0, 0, 0, 0};
float Bkt_Angle_buf[4]   = {0, 0, 0, 0};
float Swing_Angle_buf[4] = {0, 0, 0, 0};

float Boom_Rate_buf[3]  = {0, 0, 0};
float Arm_Rate_buf[3]   = {0, 0, 0};
float Bkt_Rate_buf[3]   = {0, 0, 0};
float Swing_Rate_buf[3] = {0, 0, 0};


// TO EXCAVATOR (defined by SNU)
// 12 states 
//offset
const float off_psi_U = -270; // Swing angle, (Length: Boom, Arm, Bucket)
const float off_L_B = 2;
const float off_L_A = 2;
const float off_L_K = 1.5;

const float off_psi_Udot = -3; // Swing rate, (Velocity: Boom, Arm, Bucket)
const float off_L_Bdot = -3;
const float off_L_Adot = -3;
const float off_L_Kdot = -3;

const float off_T_U = -126309; // Swing Torque, (Force: Boom, Arm, Bucket)
const float off_F_B = -855617;
const float off_F_A = -463459;
const float off_F_K = -384181;

//scale - *inverse*
const float inv_scale_psi_U = 1 / 0.0083; // Swing angle, (Length: Boom, Arm, Bucket)
const float inv_scale_L_B = 1 / 3.06e-5;
const float inv_scale_L_A = 1 / 3.82e-5;
const float inv_scale_L_K = 1 / 3.06e-5;

const float inv_scale_psi_Udot = 1 / 9.16e-5; // Swing rate, (Velocity: Boom, Arm, Bucket)
const float inv_scale_L_Bdot = 1 / 9.16e-5;
const float inv_scale_L_Adot = 1 / 9.16e-5;
const float inv_scale_L_Kdot = 1 / 9.16e-5;

const float inv_scale_T_U = 1 / 3.86; // Swing Torque, (Force: Boom, Arm, Bucket)
const float inv_scale_F_B = 1 / 39.72;
const float inv_scale_F_A = 1 / 22.38;
const float inv_scale_F_K = 1 / 18.26;

//================== Example: output from mpc ==================
float psi_U = 150; //50602 --> C5AA --> AA C5 / 170 197
float L_B = 3;     //32680 --> 7FA8 --> A8 7F / 168(167) 127
float L_A = 3;     //26178 --> 6642 --> 42 66 / 66 102
float L_K = 2;     //16340 --> 3FD4 --> D4 3F / 212(211) 63

float psi_Udot = 1; // 43668 --> AA94 --> 94 AA / 148 170
float L_Bdot = -1;  // 21834 --> 554A --> 4A 55 / 74 85
float L_Adot = 2;   // 54585 --> D539 --> 39 D5 / 57 213
float L_Kdot = -2;  // 10917 --> 2AA5 --> A5 2A / 165 42

float T_U = 100000;  //58629 --> E505 --> 05 E5 / 5 229
float F_B = 1500000; //59306 --> E7AA --> AA E7 / 170(169) 231
float F_A = 1000000; //65391 --> FF6F --> 6F FF / 111 255
float F_K = 500000;  //48422 --> BD26 --> 26 BD / 38(37) 189

float input0[12] = {psi_U, L_B, L_A, L_K, psi_Udot, L_Bdot, L_Adot, L_Kdot, T_U, F_B, F_A, F_K}; // 0.2 s 
float input1[12] = {psi_U, L_B, L_A, L_K, psi_Udot, L_Bdot, L_Adot, L_Kdot, T_U, F_B, F_A, F_K}; // 0.5 s 
float input2[12] = {psi_U, L_B, L_A, L_K, psi_Udot, L_Bdot, L_Adot, L_Kdot, T_U, F_B, F_A, F_K}; // 1.0 s
//================== for mcp output example ==================

float array0[12] = {};
float array1[12] = {};
float array2[12] = {};
unsigned short array_to_ex[72]; // 12 states * 3 times * 2 bytes = 72.
unsigned int cnt_pub = 0;

void callbackFromExcavator(const tutorial_rosardu_customtopic::packetsFromExcavatorConstPtr &msg_from_ex)
{
    int n_bytes = msg_from_ex->n_bytes;
    cout << "subscriber callback ... n_bytes: " << n_bytes << "\n";

    int pow16_2 = pow(16, 2);

    //Sensor1
    ACyl_LC = 0.1 * msg_from_ex->bytes[SENSOR1_START + 1] * pow(16, 2) + msg_from_ex->bytes[SENSOR1_START + 0];
    ACyl_SC = 0.1 * msg_from_ex->bytes[SENSOR1_START + 3] * pow(16, 2) + msg_from_ex->bytes[SENSOR1_START + 2];
    Swing_L = 0.1 * msg_from_ex->bytes[SENSOR1_START + 5] * pow(16, 2) + msg_from_ex->bytes[SENSOR1_START + 4];
    Swing_R = 0.1 * msg_from_ex->bytes[SENSOR1_START + 7] * pow(16, 2) + msg_from_ex->bytes[SENSOR1_START + 6];
    //Sensor2
    BCyl_LC = 0.1 * msg_from_ex->bytes[SENSOR2_START + 1] * pow(16, 2) + msg_from_ex->bytes[SENSOR2_START + 0];
    BCyl_SC = 0.1 * msg_from_ex->bytes[SENSOR2_START + 3] * pow(16, 2) + msg_from_ex->bytes[SENSOR2_START + 2];
    KCyl_LC = 0.1 * msg_from_ex->bytes[SENSOR2_START + 5] * pow(16, 2) + msg_from_ex->bytes[SENSOR2_START + 6];
    KCyl_SC = 0.1 * msg_from_ex->bytes[SENSOR2_START + 7] * pow(16, 2) + msg_from_ex->bytes[SENSOR2_START + 8];
    //Sensor3
    Body_Pitch_Angle = angle_scale * (msg_from_ex->bytes[SENSOR3_START + 2] * pow(16, 4) + msg_from_ex->bytes[SENSOR3_START + 1] * pow(16, 2) + msg_from_ex->bytes[SENSOR3_START + 0]) - angle_offset;
    Body_Roll_Angle = angle_scale * (msg_from_ex->bytes[SENSOR3_START + 5] * pow(16, 4) + msg_from_ex->bytes[SENSOR3_START + 4] * pow(16, 2) + msg_from_ex->bytes[SENSOR3_START + 3]) - angle_offset;
    //Sensor4
    Boom_Joint_Angle = angle_scale * (msg_from_ex->bytes[SENSOR4_START + 2] * pow(16, 4) + msg_from_ex->bytes[SENSOR4_START + 1] * pow(16, 2) + msg_from_ex->bytes[SENSOR4_START + 0]) - angle_offset;
    //Sensor5
    Arm_Joint_Angle = angle_scale * (msg_from_ex->bytes[SENSOR5_START + 2] * pow(16, 4) + msg_from_ex->bytes[SENSOR5_START + 1] * pow(16, 2) + msg_from_ex->bytes[SENSOR5_START + 0]) - angle_offset;
    //Sensor6
    Bkt_Joint_Angle = angle_scale * (msg_from_ex->bytes[SENSOR6_START + 2] * pow(16, 4) + msg_from_ex->bytes[SENSOR6_START + 1] * pow(16, 2) + msg_from_ex->bytes[SENSOR6_START + 0]) - angle_offset;
    //Sensor7
    Swing_Angle = angle_scale * (msg_from_ex->bytes[SENSOR7_START + 2] * pow(16, 4) + msg_from_ex->bytes[SENSOR7_START + 1] * pow(16, 2) + msg_from_ex->bytes[SENSOR7_START + 0]) - angle_offset;

    /*cout << " " << ACyl_LC << " " << ACyl_SC << "\n";
    cout << " " << Swing_L << " " << Swing_R << "\n";
    cout << " " << BCyl_LC << " " << BCyl_SC << "\n";
    cout << " " << KCyl_LC << " " << KCyl_SC << "\n";

    cout << " " << Body_Pitch_Angle << " " << Body_Roll_Angle << "\n";
    cout << " " << Boom_Joint_Angle << " " << Arm_Joint_Angle << " " << Bkt_Joint_Angle << " " << Swing_Angle << "\n";*/

    Boom_Angle_buf[0] = Boom_Joint_Angle;
    Arm_Angle_buf[0] = Arm_Joint_Angle;
    Bkt_Angle_buf[0] = Bkt_Joint_Angle;
    Swing_Angle_buf[0] = Swing_Angle;

    if (flag_rate)
    {
        Boom_Joint_Rate = 1.691 * Boom_Rate_buf[1] - 0.7327 * Boom_Rate_buf[2] + 1.04 * (Boom_Angle_buf[0] - Boom_Angle_buf[1]) + 2.09 * (Boom_Angle_buf[1] - Boom_Angle_buf[2]) + 1.04 * (Boom_Angle_buf[2] - Boom_Angle_buf[3]);
        Arm_Joint_Rate = 1.691 * Arm_Rate_buf[1] - 0.7327 * Arm_Rate_buf[2] + 1.04 * (Arm_Angle_buf[0] - Arm_Angle_buf[1]) + 2.09 * (Arm_Angle_buf[1] - Arm_Angle_buf[2]) + 1.04 * (Arm_Angle_buf[2] - Arm_Angle_buf[3]);
        Bkt_Joint_Rate = 1.691 * Bkt_Rate_buf[1] - 0.7327 * Bkt_Rate_buf[2] + 1.04 * (Bkt_Angle_buf[0] - Bkt_Angle_buf[1]) + 2.09 * (Bkt_Angle_buf[1] - Bkt_Angle_buf[2]) + 1.04 * (Bkt_Angle_buf[2] - Bkt_Angle_buf[3]);
        Swing_Rate = 1.691 * Swing_Rate_buf[1] - 0.7327 * Swing_Rate_buf[2] + 1.04 * (Swing_Angle_buf[0] - Swing_Angle_buf[1]) + 2.09 * (Swing_Angle_buf[1] - Swing_Angle_buf[2]) + 1.04 * (Swing_Angle_buf[2] - Swing_Angle_buf[3]);

        Boom_Rate_buf[2] = Boom_Rate_buf[1];
        Boom_Rate_buf[1] = Boom_Rate_buf[0];
        Arm_Rate_buf[2] = Arm_Rate_buf[1];
        Arm_Rate_buf[1] = Arm_Rate_buf[0];
        Bkt_Rate_buf[2] = Bkt_Rate_buf[1];
        Bkt_Rate_buf[1] = Bkt_Rate_buf[0];
        Swing_Rate_buf[2] = Swing_Rate_buf[1];
        Swing_Rate_buf[1] = Swing_Rate_buf[0];

        Boom_Rate_buf[0] = Boom_Joint_Rate;
        Arm_Rate_buf[0] = Arm_Joint_Rate;
        Bkt_Rate_buf[0] = Bkt_Joint_Rate;
        Swing_Rate_buf[0] = Swing_Rate;
    }

    Boom_Angle_buf[3] = Boom_Angle_buf[2];
    Boom_Angle_buf[2] = Boom_Angle_buf[1];
    Boom_Angle_buf[1] = Boom_Angle_buf[0];
    Arm_Angle_buf[3] = Arm_Angle_buf[2];
    Arm_Angle_buf[2] = Arm_Angle_buf[1];
    Arm_Angle_buf[1] = Arm_Angle_buf[0];
    Bkt_Angle_buf[3] = Bkt_Angle_buf[2];
    Bkt_Angle_buf[2] = Bkt_Angle_buf[1];
    Bkt_Angle_buf[1] = Bkt_Angle_buf[0];
    Swing_Angle_buf[3] = Swing_Angle_buf[2];
    Swing_Angle_buf[2] = Swing_Angle_buf[1];
    Swing_Angle_buf[1] = Swing_Angle_buf[0];

    ++buf_cnt;
    //cout << " " << Boom_Angle_buf[0] << " " << Boom_Angle_buf[1] << " " << Boom_Angle_buf[2] << " " << Boom_Angle_buf[3] << "\n";
    if (buf_cnt > 3)
    {
        flag_rate = true;
    }

    //cout << " " << Boom_Joint_Rate << " " << Arm_Joint_Rate << " " << Bkt_Joint_Rate << " " << Swing_Rate << "\n";

    ++cnt_pub;
};

void calWithFactor(float *array, float *input)
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

float X0;
int h3, h2, h1, h0;

void encodeToEx(float *array0, float *array1, float *array2, unsigned short *array_to_ex)
{
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

unsigned int cnt = 0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosardu_communication");
    ros::NodeHandle nh("~");

    // publish
    tutorial_rosardu_customtopic::packetsToExcavator msg_to_ex;
    ros::Publisher pub_to_ex = nh.advertise<tutorial_rosardu_customtopic::packetsToExcavator>("/mpc/msg", 1);

    // subscribe
    tutorial_rosardu_customtopic::packetsFromExcavator msg_from_ex;
    ros::Subscriber sub_from_ex = nh.subscribe<tutorial_rosardu_customtopic::packetsFromExcavator>("/ardu/msg", 1, callbackFromExcavator);

    while (ros::ok())
    {
        ros::spinOnce();
        //ros::Duration(0.01).sleep();
        //subscribe - 100Hz
        //publish - 10Hz

        if (cnt_pub == 10)
        {
            msg_to_ex.n_bytes = 0;
            msg_to_ex.bytes.clear();

            calWithFactor(array0, input0); // 0.2sec
            calWithFactor(array1, input1); // 0.5sec
            calWithFactor(array2, input2); // 1.0sec

            encodeToEx(array0, array1, array2, array_to_ex);

            for (int i = 0; i < 72; ++i)
            {
                msg_to_ex.bytes.push_back(array_to_ex[i]);
                //cout << array_to_ex[i] << " ";
            }
            //cout << endl;

            msg_to_ex.n_bytes = msg_to_ex.bytes.size();
            pub_to_ex.publish(msg_to_ex);
            cnt_pub = 0;
        }

        ++cnt;
        //++cnt_pub;
        if (cnt == 255)
            cnt = 0;
    }

    ROS_INFO_STREAM("End of the program.\n");
    return -1;
}