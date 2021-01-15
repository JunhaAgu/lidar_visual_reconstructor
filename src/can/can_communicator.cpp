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

    this->fillPlannerOutputs_TEST(); // for CAN test


    // Test?
    this->flag_test_ = true;
    if(this->flag_test_) {
        // generate save folder
        std::string folder_create_command;
        string save_dir_ = "/home/larrkchlaptop/hce_data/";
        folder_create_command = "mkdir " + save_dir_ + "can_test_data/";
        system(folder_create_command.c_str());

        // save association
        this->filedir_fromex_ = save_dir_ + "can_test_data/" + "from_excavator.txt";
        std::ofstream output_file(this->filedir_fromex_, std::ios::trunc);
        output_file.precision(6);
        output_file.setf(std::ios_base::fixed, std::ios_base::floatfield);
        if(output_file.is_open()){
            output_file << "time [ms], ";
            output_file << "ACyl_LC[0], ACyl_LC[1], "; //all bytes
            output_file << "ACyl_SC[0], ACyl_SC[1], ";
            output_file << "Swing_L[0], Swing_L[1], ";
            output_file << "Swing_R[0], Swing_R[1], ";
            output_file << "BCyl_LC[0], BCyl_LC[1], ";
            output_file << "BCyl_SC[0], BCyl_SC[1], ";
            output_file << "KCyl_LC[0], KCyl_LC[1], ";
            output_file << "KCyl_SC[0], KCyl_SC[1], ";
            output_file << "Body_Pitch_Angle[0], Body_Pitch_Angle[1], Body_Pitch_Angle[2] ";
            output_file << "Body_Roll_Angle[0], Body_Roll_Angle[1], Body_Roll_Angle[2] ";
            output_file << "Boom_Joint_Angle[0], Boom_Joint_Angle[1], Boom_Joint_Angle[2] ";
            output_file << "Arm_Joint_Angle[0], Arm_Joint_Angle[1], Arm_Joint_Angle[2] ";
            output_file << "Bkt_Joint_Angle[0], Bkt_Joint_Angle[1], Bkt_Joint_Angle[2] ";
            output_file << "Swing_Angle[0], Swing_Angle[1], Swing_Angle[2] ";
            output_file << "\n";           
        }

        this->filedir_toex_ = save_dir_ + "can_test_data/" + "to_excavator.txt";
        ofstream output_file2(this->filedir_toex_, std::ios::trunc);
        output_file2.precision(6);
        output_file2.setf(std::ios_base::fixed, std::ios_base::floatfield);
        if(output_file2.is_open()){
                        output_file2 <<  "time [ms], ";
            output_file2 <<  "in0_2[0], in0_2[1], "; //all bytes
            output_file2 <<  "in1_2[0], in1_2[1], ";
            output_file2 <<  "in2_2[0], in2_2[1], ";
            output_file2 <<  "in3_2[0], in3_2[1], ";
            output_file2 <<  "in4_2[0], in4_2[1], ";
            output_file2 <<  "in5_2[0], in5_2[1], ";
            output_file2 <<  "in6_2[0], in6_2[1], ";
            output_file2 <<  "in7_2[0], in7_2[1], ";
            output_file2 <<  "in8_2[0], in8_2[1], ";
            output_file2 <<  "in9_2[0], in9_2[1], ";
            output_file2 << "in10_2[0], in10_2[1], ";
            output_file2 << "in11_2[0], in11_2[1], ";
            output_file2 <<  "in0_5[0], in0_5[1], ";
            output_file2 <<  "in1_5[0], in1_5[1], ";
            output_file2 <<  "in2_5[0], in2_5[1], ";
            output_file2 <<  "in3_5[0], in3_5[1], ";
            output_file2 <<  "in4_5[0], in4_5[1], ";
            output_file2 <<  "in5_5[0], in5_5[1], ";
            output_file2 <<  "in6_5[0], in6_5[1], ";
            output_file2 <<  "in7_5[0], in7_5[1], ";
            output_file2 <<  "in8_5[0], in8_5[1], ";
            output_file2 <<  "in9_5[0], in9_5[1], ";
            output_file2 << "in10_5[0], in10_5[1], ";
            output_file2 << "in11_5[0], in11_5[1], ";
            output_file2 <<  "in0_1[0], in0_1[1], ";
            output_file2 <<  "in1_1[0], in1_1[1], ";
            output_file2 <<  "in2_1[0], in2_1[1], ";
            output_file2 <<  "in3_1[0], in3_1[1], ";
            output_file2 <<  "in4_1[0], in4_1[1], ";
            output_file2 <<  "in5_1[0], in5_1[1], ";
            output_file2 <<  "in6_1[0], in6_1[1], ";
            output_file2 <<  "in7_1[0], in7_1[1], ";
            output_file2 <<  "in8_1[0], in8_1[1], ";
            output_file2 <<  "in9_1[0], in9_1[1], ";
            output_file2 << "in10_1[0], in10_1[1], ";
            output_file2 << "in11_1[0], in11_1[1]";
            output_file2 << "\n";
        }
    }

    
};

CanCommunicator::~CanCommunicator()
{
    cout << "can communicator stops.\n";
};


#define BUTTERWORTH(abuf,rbuf) ( 1.691 * rbuf[1] - 0.7327 * rbuf[2] + 1.04 * (abuf[0] - abuf[1]) + 2.09 * (abuf[1] - abuf[2]) + 1.04 * (abuf[2] - abuf[3]) )
#define SLIDE_WINDOW_RATE_BUF(arr) {arr[2] = arr[1]; arr[1] = arr[0]; } 
#define SLIDE_WINDOW_ANGLE_BUF(arr) {arr[3] = arr[2]; arr[2] = arr[1]; arr[1] = arr[0]; }

void CanCommunicator::callbackFromExcavator(const hce_autoexcavator::packetsFromExcavatorConstPtr &msg_from_ex)
{
    int n_bytes = msg_from_ex->n_bytes;    
    float pow16_2 = (float)pow(16,2);
    float pow16_4 = (float)pow(16,4);

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

    // Update new sensor angles. (fill out the 0-th element of a angle buffer.)
    fromEx_.Boom_Angle_buf[0]  = fromEx_.Boom_Joint_Angle;
    fromEx_.Arm_Angle_buf[0]   = fromEx_.Arm_Joint_Angle;
    fromEx_.Bkt_Angle_buf[0]   = fromEx_.Bkt_Joint_Angle;
    fromEx_.Swing_Angle_buf[0] = fromEx_.Swing_Angle;

    if (butterworth_cnt_ > 3)
    {
        // Butterworth filter.
        fromEx_.Boom_Joint_Rate   = BUTTERWORTH(fromEx_.Boom_Angle_buf,  fromEx_.Boom_Rate_buf);
        SLIDE_WINDOW_RATE_BUF(fromEx_.Boom_Rate_buf);
        fromEx_.Boom_Rate_buf[0]  = fromEx_.Boom_Joint_Rate;

        fromEx_.Arm_Joint_Rate    = BUTTERWORTH(fromEx_.Arm_Angle_buf,   fromEx_.Arm_Rate_buf);
        SLIDE_WINDOW_RATE_BUF(fromEx_.Arm_Rate_buf);
        fromEx_.Arm_Rate_buf[0]   = fromEx_.Arm_Joint_Rate;

        fromEx_.Bkt_Joint_Rate    = BUTTERWORTH(fromEx_.Bkt_Angle_buf,   fromEx_.Bkt_Rate_buf);
        SLIDE_WINDOW_RATE_BUF(fromEx_.Bkt_Rate_buf);
        fromEx_.Bkt_Rate_buf[0]   = fromEx_.Bkt_Joint_Rate;
        
        fromEx_.Swing_Rate        = BUTTERWORTH(fromEx_.Swing_Angle_buf, fromEx_.Swing_Rate_buf);
        SLIDE_WINDOW_RATE_BUF(fromEx_.Swing_Rate_buf);
        fromEx_.Swing_Rate_buf[0] = fromEx_.Swing_Rate;
    }
    SLIDE_WINDOW_ANGLE_BUF(fromEx_.Boom_Angle_buf);
    SLIDE_WINDOW_ANGLE_BUF(fromEx_.Arm_Angle_buf);
    SLIDE_WINDOW_ANGLE_BUF(fromEx_.Bkt_Angle_buf);
    SLIDE_WINDOW_ANGLE_BUF(fromEx_.Swing_Angle_buf);

    ++butterworth_cnt_;


    // For test. Save raw data.
    if(flag_test_){
        std::ofstream output_file(this->filedir_fromex_, std::ios::app);
        output_file.precision(6);
        output_file.setf(std::ios_base::fixed, std::ios_base::floatfield);
        if(output_file.is_open()){
            output_file << ros::Time::now() <<", ";
            for(int i = 0; i < 33; ++i){
                // output_file << i <<", ";
                output_file << (int)msg_from_ex->bytes[i] << ", ";
            }
            output_file << (int)msg_from_ex->bytes[33] <<"\n";
        }  
    }
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

void CanCommunicator::fillPlannerOutputs_TEST() {
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

    planner_outputs0_[0] =  psi_U;
    planner_outputs0_[1] =  L_B;
    planner_outputs0_[2] =  L_A;
    planner_outputs0_[3] =  L_K;
    planner_outputs0_[4] =  psi_Udot;
    planner_outputs0_[5] =  L_Bdot;
    planner_outputs0_[6] =  L_Adot;
    planner_outputs0_[7] =  L_Kdot;
    planner_outputs0_[8] =  T_U;
    planner_outputs0_[9] =  F_B;
    planner_outputs0_[10] =  F_A; 
    planner_outputs0_[11] =  F_K; // 0.2 s 

    for(int i = 0; i < 12; ++i){
        planner_outputs1_[i] = planner_outputs0_[i]; // 0.5 s 
        planner_outputs2_[i] = planner_outputs0_[i]; // 1.0 s 
    }
};

void CanCommunicator::fillPlannerOutputs(){
    // TODO
};

void CanCommunicator::publishToExcavator(){
    // initialize msg.
    msg_to_ex_.n_bytes = 0;
    msg_to_ex_.bytes.clear();

    // fill out ros time.
    ++msg_to_ex_.header.seq;
    msg_to_ex_.header.stamp = ros::Time::now();
    
    this->calWithFactor(planner_outputs0_, planner_outputs0_processed_); // 0.2 s 
    this->calWithFactor(planner_outputs1_, planner_outputs1_processed_); // 0.5 s
    this->calWithFactor(planner_outputs2_, planner_outputs2_processed_); // 1.0 s 

    this->encodeToEx(
        planner_outputs0_processed_,
        planner_outputs1_processed_,
        planner_outputs2_processed_,
        array_to_ex_);

    for(int i = 0; i < 72; ++i){
        msg_to_ex_.bytes.push_back(array_to_ex_[i]);
    }
    msg_to_ex_.n_bytes = msg_to_ex_.bytes.size();

    // publish
    pub_to_ex_.publish(msg_to_ex_);

    // For test. Save raw data.
    if(flag_test_){
        std::ofstream output_file(this->filedir_toex_, std::ios::app);
        output_file.precision(6);
        output_file.setf(std::ios_base::fixed, std::ios_base::floatfield);
        if(output_file.is_open()){
            output_file << ros::Time::now() <<", ";
            for(int i = 0; i < 71; ++i){
                output_file << (int)array_to_ex_[i] <<", ";
            }
            output_file << (int)array_to_ex_[71] <<"\n";
        }  
    }
};
