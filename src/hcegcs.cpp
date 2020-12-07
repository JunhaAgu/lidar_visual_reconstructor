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
    buf_imgs_.reserve(n_cameras_);

    // command message publisher.
    pub_msg_command_ = nh_.advertise<std_msgs::Int32>("/hhi/msg",1);

    // initialize image container & subscribers.
    flag_imgs_ = nullptr;
    if(n_cameras_ > 0){
        flag_imgs_ = new bool[n_cameras_];
        for(int i = 0; i < n_cameras_; i++) {
            flag_imgs_[i] = false;
            string name_temp = "/" + itos(i) + "/image_raw";
            topicnames_imgs_.push_back(name_temp);
            subs_imgs_.push_back(it_.subscribe(topicnames_imgs_[i], 1, boost::bind(&HCEGCS::callbackImage, this, _1, i)));
            buf_imgs_.push_back(cv::Mat());
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

    // initialize servers
    server_lidarimagedata_ = nh_.advertiseService("srv_lidar_image_data",&HCEGCS::serverCallbackLidarImageData,this);

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
    cout << "10 Hz (forced) streaming mode\n";
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
    ros::Duration(0.1).sleep();
};

void HCEGCS::snapshotMode(){
    // send single query to all sensors.
    bool is_query_ok = sendSingleQueryToAllSensors();
    
    // Save all data
    if(is_query_ok) saveAllData(); // save and purge the current data!
    else cout << "   fail to save...\n";
};

void HCEGCS::runAlgorithms(){
    // 1) use current boom, arm, bucket, body angles
    // use L (distance from boom rotating axis to boom lidar)
    // use w (boom lidar real pose)
    

    // 2) Calculate all poses of sensors at the time.

    // 3) fire signal for running algorithm to 'planner' 
    // Then, 'planner' requests profile poly. to 'GCS' (with specific order of poly.)
    // Then, 'GCS' requests profile 3D points to 'Recon'.
    // Then, 'Recon' requests lidar images data to 'GCS'.
    // Then. 'GCS' acquires snapshot data and sends it to 'Recon'
    // Then, 'Recon' calculate all 3D terrain and sends 3D profiles to 'GCS"
    // Then, 'GCS' calculate profile polynomial coefficients, sends them to 'Planner'.

    // Finally, 'Planner' iteratively calculates and publishs control inputs until end of a procedure.
    // Simultaneously, 'GCS' subscribes control inputs and the 'callback' function publishes the control inputs to Arduino.
    // 'GCS' responds to the 'planner's request, and then 

};

void HCEGCS::setTestLidarImages(string dir, float theta){
    // dir =/home/larrkchlaptop/catkin_ws/src/lidar_visual_reconstructor/test_data
    // plane: cam0 cam1 lidar0 lidar1 (cabin cam0, cabin cam1)
    // up: cam0 cam1 lidar0 lidar1 (cabin cam0, cabin cam1)
    // down: cam2 cam3 lidar1 lidar0 (boom cam0, boom cam1)

    // test set is for upper mound.
    // selected indexes
    cout << "set test lidar images...\n";
    int ids_cam[2]     = {0, 1};
    int id_major_lidar = 0;
    int id_sub_lidar   = 1;
    
    string img_dir = dir + '/' + "cabin_cam_0.png";
    cv::Mat img0, img1;
    img0 = cv::imread(img_dir, CV_LOAD_IMAGE_GRAYSCALE);
    if(img0.data == nullptr) throw std::runtime_error("img 0 is empty. Directory to image could be wrong.\n");
    img_dir = dir + '/' + "cabin_cam_1.png";
    img1 = cv::imread(img_dir, CV_LOAD_IMAGE_GRAYSCALE);
    if(img1.data == nullptr) throw std::runtime_error("img 1 is empty. Directory to image could be wrong.\n");

    // fillout images
    img0.copyTo(buf_imgs_[0]);
    img1.copyTo(buf_imgs_[1]);

    cout << " [TEST] images OK!\n";
    
    // visualization
    cv::namedWindow("img0",CV_WINDOW_AUTOSIZE);
    cv::namedWindow("img1",CV_WINDOW_AUTOSIZE);
    cv::imshow("img0", img0);
    cv::imshow("img1", img1);
    cv::waitKey(0);


    // load pcd
    string pcd_dir = dir + '/' + "cabin_lidar.pcd";
    ifstream pcdfile;
    pcdfile.open(pcd_dir.c_str());
    string s;
    for(int i = 0; i < 11; i++) getline(pcdfile, s); // get rid of header part.
    int cnt = 0;
    while(!pcdfile.eof()){
        getline(pcdfile, s);
        if(!s.empty()){
            stringstream ss;
            ss << s;
            float x,y,z,time,intensity;
            int ring;
            ss >> x;
            ss >> y;
            ss >> z;
            ss >> intensity;
            ss >> ring;
            ss >> time;
            *(buf_lidars_x[id_major_lidar]+cnt) = x;
            *(buf_lidars_y[id_major_lidar]+cnt) = y;
            *(buf_lidars_z[id_major_lidar]+cnt) = z;
            *(buf_lidars_intensity[id_major_lidar]+cnt) = intensity;
            *(buf_lidars_ring[id_major_lidar]+cnt) = ring;
            *(buf_lidars_time[id_major_lidar]+cnt) = time;
            ++cnt;
        }
    }
    pcdfile.close();
    buf_lidars_npoints[id_major_lidar] = cnt;
    cout << " read done :" << cnt <<" \n";

    pcd_dir = dir + '/' + "boom_lidar.pcd";
    pcdfile.open(pcd_dir.c_str());
    for(int i = 0; i < 11; i++) getline(pcdfile, s); // get rid of header part.
    cnt = 0;
    while(!pcdfile.eof()){
        getline(pcdfile, s);
        if(!s.empty()){
            stringstream ss;
            ss << s;
            float x,y,z,time,intensity;
            int ring;
            ss >> x;
            ss >> y;
            ss >> z;
            ss >> intensity;
            ss >> ring;
            ss >> time;
            *(buf_lidars_x[id_sub_lidar]+cnt) = x;
            *(buf_lidars_y[id_sub_lidar]+cnt) = y;
            *(buf_lidars_z[id_sub_lidar]+cnt) = z;
            *(buf_lidars_intensity[id_sub_lidar]+cnt) = intensity;
            *(buf_lidars_ring[id_sub_lidar]+cnt) = ring;
            *(buf_lidars_time[id_sub_lidar]+cnt) = time;
            ++cnt;
        }
    }
    pcdfile.close();
    buf_lidars_npoints[id_sub_lidar] = cnt;
    cout << " read done :" << cnt <<" \n";


    // theta simulated.
    theta_ = theta;

    cout << "simulated boom angle: " << theta_/3.141592*180.0f <<" [deg]\n";
    cout << " Load test files : done!\n";
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
	buf_imgs_[id] = cv_ptr->image;

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
	    cv::imwrite(file_name, buf_imgs_[id], png_parameters);
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

bool HCEGCS::serverCallbackLidarImageData(hce_autoexcavator::lidarImageDataStamped::Request &req,
        hce_autoexcavator::lidarImageDataStamped::Response &res)
{
    cout << "Service server [LidarImageData] is received!! request type: " << req.request_type<< "\n";
    if(buf_lidars_npoints[0] > 0){
        res.n_pts0 = buf_lidars_npoints[0];
        for(int i = 0; i < buf_lidars_npoints[0]; ++i){
            res.x0.push_back(*(buf_lidars_x[0]+i));
            res.y0.push_back(*(buf_lidars_y[0]+i));
            res.z0.push_back(*(buf_lidars_z[0]+i));
            res.ring0.push_back(*(buf_lidars_ring[0]+i));
            res.intensity0.push_back(*(buf_lidars_intensity[0]+i));
            res.time0.push_back(*(buf_lidars_time[0]+i));
        }
    }
    else{
        ROS_WARN("Lidar 0 data is not ready!\n");
        return false;
    }
    if(buf_lidars_npoints[1] > 0){
        res.n_pts1 = buf_lidars_npoints[1];
        for(int i = 0; i < buf_lidars_npoints[1]; ++i){
            res.x1.push_back(*(buf_lidars_x[1]+i));
            res.y1.push_back(*(buf_lidars_y[1]+i));
            res.z1.push_back(*(buf_lidars_z[1]+i));
            res.ring1.push_back(*(buf_lidars_ring[1]+i));
            res.intensity1.push_back(*(buf_lidars_intensity[1]+i));
            res.time1.push_back(*(buf_lidars_time[1]+i));
        }
    }
    else{
        ROS_WARN("Lidar 1 data is not ready!\n");
        return false;
    }
    if(buf_imgs_[0].rows > 0){
        sensor_msgs::fillImage(res.img0, sensor_msgs::image_encodings::MONO8,
                buf_imgs_[0].rows, buf_imgs_[0].cols, buf_imgs_[0].step, buf_imgs_[0].data);    
    }
    else{
        ROS_WARN("Image data is not ready!\n");
        return false;
    }
    int i = 0;
    if(buf_imgs_[i].rows > 0){
        sensor_msgs::fillImage(res.img0, sensor_msgs::image_encodings::MONO8,
                buf_imgs_[i].rows, buf_imgs_[i].cols, buf_imgs_[i].step, buf_imgs_[i].data);    
    }
    else{
        ROS_WARN("Image data is not ready!\n");
        return false;
    }
    i = 1;
    if(buf_imgs_[i].rows > 0){
        sensor_msgs::fillImage(res.img1, sensor_msgs::image_encodings::MONO8,
                buf_imgs_[i].rows, buf_imgs_[i].cols, buf_imgs_[i].step, buf_imgs_[i].data);    
    }
    else{
        ROS_WARN("Image data is not ready!\n");
        return false;
    }
    
    return true;
};


bool HCEGCS::serverCallbackRelativeLidarPose(hce_autoexcavator::relativeLidarPoseStamped::Request &req,
        hce_autoexcavator::relativeLidarPoseStamped::Response &res)
{
    cout << "Service server [relativeLidarPose] is received!!\n";
    res.header.stamp = ros::Time::now();
    res.header.seq   = -1;
    res.header.frame_id = -1;
    
    // TODO: get theta!
    Eigen::Matrix3f R_l0l1;    Eigen::Vector3f t_l0l1;
    Eigen::Matrix4f T_l0l1;
    calcRelativeLidarPose(theta_, R_l0l1, t_l0l1);
    T_l0l1 << R_l0l1, t_l0l1, 0, 0, 0, 1;
    cout << " T_l0l1:\n" 
    << T_l0l1 << "\n";
    Eigen::Matrix<float,6,1> xi_l0l1;
    sophuslie::SE3Log(T_l0l1, xi_l0l1);

    res.tx = 0.0f;
    res.ty = 0.0f;
    res.tz = 0.0f;
    res.wx = 0.0f;
    res.wy = 0.0f;
    res.wz = 0.0f;
};


void HCEGCS::calcRelativeLidarPose(const float& theta, Eigen::Matrix3f& R_l0l1, Eigen::Vector3f& t_l0l1){
    // TODO: exact calculation!!!!!

    R_l0l1 = Eigen::Matrix3f::Identity();
    t_l0l1 << 0,0,0;
};