#ifndef _CAN_MESSAGE_H_
#define _CAN_MESSAGE_H_

#include <iostream>
#include <queue>

namespace constants{
    // Constant values related to the CAN communication.
    
    // For HCE messages
    // CAN PGN ID
    const int SENSOR1_START = 0;  //CAN ID 18FF4DE7
    const int SENSOR2_START = 8;  //CAN ID 18FF61E7
    const int SENSOR3_START = 16; //CAN ID 18F02971
    const int SENSOR4_START = 22; //CAN ID 18F02972
    const int SENSOR5_START = 25; //CAN ID 18F02973
    const int SENSOR6_START = 28; //CAN ID 18F02974
    const int SENSOR7_START = 31; //CAN ID 18F02970

    // Scaler and offset values for all angles from EXCAVATOR
    const float angle_scale = 3.0517578125e-5;
    const float angle_offset = -250;

    // For SNU messages
    // Offset values (defined by SNU)
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
};


struct FromExcvt{
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
    float Body_Pitch_Angle; // body pitch angle
    float Body_Roll_Angle;  // body roll  angle 
    //0x18F02972
    float Boom_Joint_Angle; // boom joint angle (joint: boom-body)
    //0x18F02973
    float Arm_Joint_Angle;  // arm  joint angle (joint: arm-boom)
    //0x18F02974
    float Bkt_Joint_Angle;  // bucket joint angle (joint: bucket-arm)
    //0x18F02970
    float Swing_Angle;      // swing angle (joint: body mobile part-body top part)

    // 4 derived (rates) (numerical derivatives by us)
    float Boom_Joint_Rate;
    float Arm_Joint_Rate;
    float Bkt_Joint_Rate;
    float Swing_Rate;

    // Numerical derivatives 
    float Boom_Angle_buf[4]  = {0, 0, 0, 0};
    float Arm_Angle_buf[4]   = {0, 0, 0, 0};
    float Bkt_Angle_buf[4]   = {0, 0, 0, 0};
    float Swing_Angle_buf[4] = {0, 0, 0, 0};

    float Boom_Rate_buf[3]  = {0, 0, 0};
    float Arm_Rate_buf[3]   = {0, 0, 0};
    float Bkt_Rate_buf[3]   = {0, 0, 0};
    float Swing_Rate_buf[3] = {0, 0, 0};
};

struct ToExcvt{
    // TO EXCAVATOR (defined by SNU)
    // 12 states 
    float psi_U; //50602 --> C5AA --> AA C5 / 170 197
    float L_B;     //32680 --> 7FA8 --> A8 7F / 168(167) 127
    float L_A;     //26178 --> 6642 --> 42 66 / 66 102
    float L_K;     //16340 --> 3FD4 --> D4 3F / 212(211) 63

    float psi_Udot; // 43668 --> AA94 --> 94 AA / 148 170
    float L_Bdot;  // 21834 --> 554A --> 4A 55 / 74 85
    float L_Adot;   // 54585 --> D539 --> 39 D5 / 57 213
    float L_Kdot;  // 10917 --> 2AA5 --> A5 2A / 165 42

    float T_U;  //58629 --> E505 --> 05 E5 / 5 229
    float F_B; //59306 --> E7AA --> AA E7 / 170(169) 231
    float F_A; //65391 --> FF6F --> 6F FF / 111 255
    float F_K;  //48422 --> BD26 --> 26 BD / 38(37) 189

    float input0[12] = {psi_U, L_B, L_A, L_K, psi_Udot, L_Bdot, L_Adot, L_Kdot, T_U, F_B, F_A, F_K}; // 0.2 s 
    float input1[12] = {psi_U, L_B, L_A, L_K, psi_Udot, L_Bdot, L_Adot, L_Kdot, T_U, F_B, F_A, F_K}; // 0.5 s 
    float input2[12] = {psi_U, L_B, L_A, L_K, psi_Udot, L_Bdot, L_Adot, L_Kdot, T_U, F_B, F_A, F_K}; // 1.0 s
};


#endif