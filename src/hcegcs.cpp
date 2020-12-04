#include "hcegcs.h"

inline string dtos(double x){
	stringstream s;
	s << setprecision(6) << fixed << x;
	return s.str();
};

inline string itos(double x){
	stringstream s;
	s << x;
	return s.str();
};

HCEGCS::HCEGCS(ros::NodeHandle& nh,
    int n_cams, int n_lidars, const string& save_dir)
: nh_(nh), it_(nh_), n_cameras_(n_cams), n_lidars_(n_lidars),save_dir_(save_dir)
{
    // default.
    flag_lidars_ = nullptr;
    flag_imgs_   = nullptr;
    buf_imgs_    = nullptr;

    // command message publisher.
    pub_msg_command_ = nh_.advertise<std_msgs::Int32>("/hhi/msg",1);

    // initialize image container & subscribers.
    buf_imgs_  = nullptr;
    flag_imgs_ = nullptr;
    if(n_cameras_ > 0){
        buf_imgs_  = new cv::Mat[n_cameras_];
        flag_imgs_ = new bool[n_cameras_];
        for(int i = 0; i < n_cameras_; i++) {
            flag_imgs_[i] = false;
            string name_temp = "/" + itos(i) + "/image_raw";
            topicnames_imgs_.push_back(name_temp);
            subs_imgs_.push_back(it_.subscribe(topicnames_imgs_[i], 1, boost::bind(&HCEGCS::callbackImage, this, _1, i)));
        }
    }

    // initialize lidar container & subscribers.
    buf_lidars_  = nullptr;
    flag_lidars_ = nullptr;
    if(n_lidars_ > 0){
        buf_lidars_ = new pcl::PointCloud<pcl::PointXYZI>::Ptr[n_lidars_];
        flag_lidars_ = new bool[n_lidars_];
        for(int i = 0; i < n_lidars_; i++){
            flag_lidars_[i] = false;
            *(buf_lidars_ + i) = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            buf_lidars_x.push_back(new float[100000]);
            buf_lidars_y.push_back(new float[100000]);
            buf_lidars_z.push_back(new float[100000]);
            buf_lidars_intensity.push_back(new float[100000]);
            buf_lidars_ring.push_back(new unsigned short[100000]);
            buf_lidars_time.push_back(new float[100000]);
            buf_lidars_npoints.push_back(0);

            string name_temp = "/lidar" + itos(i) + "/velodyne_points";
            topicnames_lidars_.push_back(name_temp);
            subs_lidars_.push_back(nh_.subscribe<sensor_msgs::PointCloud2>(topicnames_lidars_[i], 1, boost::bind(&HCEGCS::callbackLidar, this, _1, i)));
        }
    }
    
    // initialize arduino container & subscriber.
    flag_mcu_ = false;
    buf_time_ = -1.0;
    sub_timestamp_ = nh_.subscribe("/trigger_time", 1, &HCEGCS::callbackTime, this);


    // generate save folder
    std::string folder_create_command;
    folder_create_command = "sudo rm -rf " + save_dir_;
	system(folder_create_command.c_str());
    folder_create_command = "mkdir " + save_dir_;
	system(folder_create_command.c_str());

    // make image saving directories
    for(int i = 0; i< n_cameras_; i++){
        folder_create_command = "mkdir " + save_dir_ + "cam" + itos(i) + "/";
	    system(folder_create_command.c_str());
    }

    // make lidar data saving directories
    for(int i = 0; i < n_lidars_; i++){
        folder_create_command = "mkdir " + save_dir_ + "lidar" + itos(i) + "/";
	    system(folder_create_command.c_str());
    }

    // save association
    string file_name = save_dir_ + "/association.txt";
    std::ofstream output_file(file_name, std::ios::trunc);
    output_file.precision(6);
    output_file.setf(std::ios_base::fixed, std::ios_base::floatfield);
    if(output_file.is_open()){
        output_file << "time_us ";
        for(int i = 0; i < n_cameras_; i++) output_file << "cam" << i << " ";
        output_file << "exposure_us gain_dB ";
        for(int i = 0; i < n_lidars_; i++) output_file << "lidar" << i <<" ";
        output_file << "\n";
    }

};

HCEGCS::~HCEGCS() {
    // ! all allocation needs to be freed.
    if( buf_imgs_ != nullptr ) delete[] buf_imgs_;
    if( flag_imgs_ != nullptr ) delete[] flag_imgs_;

    if( buf_lidars_ != nullptr) delete[] buf_lidars_;
    if( flag_lidars_ != nullptr) delete[] flag_lidars_;
    for(int i = 0; i < n_lidars_; i++){
	delete[] buf_lidars_x[i];
	delete[] buf_lidars_y[i];
	delete[] buf_lidars_z[i];
	delete[] buf_lidars_intensity[i];
	delete[] buf_lidars_ring[i];
	delete[] buf_lidars_time[i];
    }
};

void HCEGCS::streamingMode(){
    cout << "20 Hz (forced) streaming mode\n";
    // initialize all flags
    initAllFlags();

    // fill out control msg
    msg_command_.data = 1;
    
    // query sensor data for all sensors
    pub_msg_command_.publish(msg_command_);
    clearMsgCommand();
    
    // (timeout) Wait for obtaining and transmitting all sensor data. 
    // Considering exposure time and lidar gathering time, set 50 ms
    ros::spinOnce();
    ros::Duration(0.05).sleep();
};

void HCEGCS::snapshotMode(){
    // send single query to all sensors.
    bool is_query_ok = sendSingleQueryToAllSensors();
    
    // Save all data
    if(is_query_ok) saveAllData(); // save and purge the current data!
    else cout << "   fail to save...\n";
};

void HCEGCS::runAlgorithms(){
    // use current boom, arm, bucket, body angles
    // use L (distance from boom rotating axis to boom lidar)
    // use w (boom lidar real pose)
    // 

    // 1) Calculate all poses of sensors at the time.

    // 2) request 
};

bool HCEGCS::sendSingleQueryToAllSensors()
{   
    // initialize all flags
    initAllFlags();

    // fill out control msgc
    msg_command_.data = 1;

    // query sensor data for all sensors
    pub_msg_command_.publish(msg_command_);
    clearMsgCommand();

    // (timeout) Wait for obtaining and transmitting all sensor data. 
    // Considering exposure time and lidar gathering time, set 50 ms
    cout << "wating 150 ms for data transmission...\n"; // because a rate of a lidar is about 10 Hz.
    ros::Duration(0.15).sleep();
    ros::spinOnce();
    
    // Check whether all data is received.
    bool transmit_success = true;
    if(flag_imgs_ != nullptr){
        for(int i = 0; i < n_cameras_; i++){
            transmit_success = transmit_success & flag_imgs_[i];
            if(flag_imgs_[i]) cout << "Rcvd image[" << i<<"]\n";
            else cout <<"Not rcvd image[" << i << "]\n";
	    }
    }
    if(flag_lidars_ != nullptr){
        for(int i = 0; i < n_lidars_; i++){
            transmit_success = transmit_success & flag_lidars_[i];
            if(flag_lidars_[i]) cout << "Rcvd lidar[" << i<<"]\n";
            else cout <<"Not rcvd lidar[" << i << "]\n";
	    }
    }

    transmit_success = transmit_success & flag_mcu_;
    if(flag_mcu_) cout << "Rcvd MCU signal & trigger time.\n";


    if(transmit_success) cout << "... Transmission successes\n";
    else cout << "... Fail to transmit!\n";

    return transmit_success;
};

void HCEGCS::initAllFlags(){
    for(int i = 0; i < n_cameras_; i++) flag_imgs_[i] = false;
    for(int i = 0; i < n_lidars_; i++) {
        flag_lidars_[i] = false;
        buf_lidars_npoints[i] = 0;
    }
    flag_mcu_ = false;
};

void HCEGCS::callbackImage(const sensor_msgs::ImageConstPtr& msg, const int& id){
    cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	*(buf_imgs_ + id) = cv_ptr->image;

    cout << " [CALLBACK] Image OK! camera id: [" << id << "]\n";
    flag_imgs_[id] = true;
};

void HCEGCS::pointcloud2tobuffers(const sensor_msgs::PointCloud2ConstPtr& msg_lidar, const int& id){
    // get width and height of 2D point cloud data
    buf_lidars_npoints[id] = msg_lidar->width;
    for(int i = 0; i < msg_lidar->width; i++) {
       int arrayPosX = i*msg_lidar->point_step + msg_lidar->fields[0].offset; // X has an offset of 0
       int arrayPosY = i*msg_lidar->point_step + msg_lidar->fields[1].offset; // Y has an offset of 4
       int arrayPosZ = i*msg_lidar->point_step + msg_lidar->fields[2].offset; // Z has an offset of 8

       int ind_intensity = i*msg_lidar->point_step + msg_lidar->fields[3].offset; // 12
       int ind_ring = i*msg_lidar->point_step + msg_lidar->fields[4].offset; // 16
       int ind_time = i*msg_lidar->point_step + msg_lidar->fields[5].offset; // 18

       memcpy(buf_lidars_x[id]+i, &msg_lidar->data[arrayPosX], sizeof(float));
       memcpy(buf_lidars_y[id]+i, &msg_lidar->data[arrayPosY], sizeof(float));
       memcpy(buf_lidars_z[id]+i, &msg_lidar->data[arrayPosZ], sizeof(float));
       memcpy(buf_lidars_intensity[id]+i, &msg_lidar->data[ind_intensity], sizeof(float));
       memcpy(buf_lidars_ring[id]+i, &msg_lidar->data[ind_ring], sizeof(unsigned short));
       memcpy(buf_lidars_time[id]+i, &msg_lidar->data[ind_time], sizeof(float));
    }
}

void HCEGCS::callbackLidar(const sensor_msgs::PointCloud2ConstPtr& msg_lidar, const int& id){
	pointcloud2tobuffers(msg_lidar,  id);
    msg_lidar->header.stamp; // timestamp

    // Create a container for the data.
    sensor_msgs::PointCloud2 output;

    // Do data processing here...
    output = *msg_lidar;

    pcl::PointCloud<pcl::PointXYZI>::Ptr temp = *(buf_lidars_ + id);
    temp->clear();
    pcl::fromROSMsg(output, *temp);

    int n_pts = temp->points.size();
    cout <<" [CALLBACK] LiDAR OK! lidar id: [" << id <<"], # of points: " <<n_pts<<endl;
    flag_lidars_[id] = true;
};

void HCEGCS::callbackTime(const sensor_msgs::TimeReference::ConstPtr& t_ref){
    buf_time_ = (double)t_ref->header.stamp.sec + (double)t_ref->header.stamp.nsec/(double)1000000.0;
    current_seq_ = t_ref->header.seq;
    cout << " [CALLBACK] MCU OK! trigger time: " << buf_time_ <<"], # seq: " << current_seq_ << "\n";
    flag_mcu_ = true;
};

void HCEGCS::saveLidarDataRingTime(const std::string& file_name, const int& id){
    int n_pts = buf_lidars_npoints[id];

    std::ofstream output_file(file_name, std::ios::trunc);
    output_file.precision(6);
    output_file.setf(std::ios_base::fixed, std::ios_base::floatfield);

    if(output_file.is_open()){
        output_file << "# .PCD v.7 - Point Cloud Data file format\n";
        output_file << "VERSION .7\n";
        output_file << "FIELDS x y z intensity ring time\n";
        output_file << "SIZE 4 4 4 4 2 4\n";
        output_file << "TYPE F F F F U F\n";
        output_file << "COUNT 1 1 1 1 1 1\n";
        output_file << "WIDTH " << n_pts << "\n";
        output_file << "HEIGHT 1\n";
        output_file << "VIEWPOINT 0 0 0 1 0 0 0\n";
        output_file << "POINTS " << n_pts<< "\n";
        output_file << "DATA ascii\n";
        for(int i = 0; i < n_pts; i++){
            output_file << *(buf_lidars_x[id] + i)<<" ";
            output_file << *(buf_lidars_y[id] + i)<<" ";
            output_file << *(buf_lidars_z[id] + i)<<" ";
            output_file << *(buf_lidars_intensity[id] + i)<<" ";
            output_file << *(buf_lidars_ring[id] + i)<<" ";
            output_file << *(buf_lidars_time[id] + i)<<"\n";
        }
    }  
};

void HCEGCS::saveAllData(){
    // save images
    bool static png_param_on = false;
	vector<int> static png_parameters;
	if (png_param_on == false)
	{
		png_parameters.push_back(CV_IMWRITE_PNG_COMPRESSION); // We save with no compression for faster processing
		png_parameters.push_back(0);
		png_param_on = true;
	}
    for(int id = 0; id < n_cameras_; id++){
        string file_name = save_dir_ + "/cam" + itos(id) + "/" + itos(current_seq_) + ".png";
	    cv::imwrite(file_name, *(buf_imgs_ + id), png_parameters);
    }

    // save lidars
    for(int id = 0; id <n_lidars_; id++){
        string file_name = save_dir_ + "/lidar" + itos(id) + "/" + itos(current_seq_) + ".pcd";
        //saveLidarData(file_name, *(buf_lidars_ + id));
        saveLidarDataRingTime(file_name, id);
    }

    // save association
    string file_name = save_dir_ + "/association.txt";
    std::ofstream output_file(file_name, std::ios::app);
    output_file.precision(6);
    output_file.setf(std::ios_base::fixed, std::ios_base::floatfield);
    if(output_file.is_open()){
        output_file << buf_time_ << " ";
        for(int i = 0; i < n_cameras_; i++) output_file << "/cam" << i << "/" << current_seq_ << ".png ";
        for(int i = 0; i < n_lidars_; i++) output_file << "/lidar" << i << "/" << current_seq_ << ".pcd ";
        output_file << "\n";
    }
};