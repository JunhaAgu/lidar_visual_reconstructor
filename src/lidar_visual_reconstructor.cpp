#include "lidar_visual_reconstructor.hpp"

LidarVisualReconstructor::LidarVisualReconstructor(ros::NodeHandle& nh)
: nh_(nh)
{
    MAX_LVL_PYR_ = 4;

    n_lidars_  = 2;
    n_cameras_ = 4;

    n_channels_.push_back(16);
    n_channels_.push_back(16);
    
    for(int i = 0; i < n_lidars_; ++i)  pcls_.push_back(new LidarPcl(n_channels_[i]));
    for(int i = 0; i < n_cameras_; ++i) cams_.push_back(new Camera());

    // load camera intrinsics
    string dir_cam_intrinsics = "/home/larrkchlaptop/catkin_ws/src/lidar_visual_reconstructor/params/camera_intrinsics.yaml";
    loadCameraIntrinsics(dir_cam_intrinsics);

    // load sensors extrinsics
    string dir_sensor_extrinsics = "/home/larrkchlaptop/catkin_ws/src/lidar_visual_reconstructor/params/sensor_extrinsics.yaml";
    loadSensorExtrinsics(dir_sensor_extrinsics);

    // By using initialized camera params, calculate all camera undistortion maps
    for(auto iter = cams_.begin(); iter != cams_.end(); ++iter)
        (*iter)->generateUndistortMaps();

    // fill out frames.
    for(int i = 0; i < n_cameras_; ++i) frames_.push_back(new Frame(cams_[i], MAX_LVL_PYR_));

    // Initiate messages
    

    // Initiate services
    nh_.serviceClient<hce_autoexcavator::LidarImageDataStamped>("srv_lidar_image_data");

};
LidarVisualReconstructor::~LidarVisualReconstructor(){

};

bool LidarVisualReconstructor::serverCallbackProfilePoints(hce_autoexcavator::ProfilePointsStamped::Request &req,
        hce_autoexcavator::ProfilePointsStamped::Response &res)
{   
    // run algorithm.
    this->run();

    // Fill out response for 'GCS' node.

};

void LidarVisualReconstructor::loadCameraIntrinsics(string& dir){
    // load yaml file
    cv::FileStorage fs(dir, cv::FileStorage::READ);
    if(!fs.isOpened()) throw std::runtime_error("intrinsic file cannot be found!\n");
    cout << "intrinsic file is loaded...\n";

    // fill out params.
    // 0: cabin lower
    // 1: cabin upper
    // 2: boom lower
    // 3: boom upper
    int rows_tmp, cols_tmp;
    cv::Mat cvK_tmp, cvD_tmp;

    rows_tmp = fs["cabin.cam0.height"];
    cols_tmp = fs["cabin.cam0.width"];
    fs["cabin.cam0.K"] >> cvK_tmp;
    fs["cabin.cam0.D"] >> cvD_tmp;
    cams_[0]->initParams(cols_tmp, rows_tmp, cvK_tmp, cvD_tmp);

    rows_tmp = fs["cabin.cam1.height"];
    cols_tmp = fs["cabin.cam1.width"];
    fs["cabin.cam1.K"] >> cvK_tmp;
    fs["cabin.cam1.D"] >> cvD_tmp;
    cams_[1]->initParams(cols_tmp, rows_tmp, cvK_tmp, cvD_tmp);

    rows_tmp = fs["boom.cam0.height"];
    cols_tmp = fs["boom.cam0.width"];
    fs["boom.cam0.K"] >> cvK_tmp;
    fs["boom.cam0.D"] >> cvD_tmp;
    cams_[2]->initParams(cols_tmp, rows_tmp, cvK_tmp, cvD_tmp);

    rows_tmp = fs["boom.cam1.height"];
    cols_tmp = fs["boom.cam1.width"];
    fs["boom.cam1.K"] >> cvK_tmp;
    fs["boom.cam1.D"] >> cvD_tmp;
    cams_[3]->initParams(cols_tmp, rows_tmp, cvK_tmp, cvD_tmp);
};

void LidarVisualReconstructor::loadSensorExtrinsics(string& dir){
    // load yaml file
    cv::FileStorage fs(dir, cv::FileStorage::READ);
    if(!fs.isOpened()) throw std::runtime_error("extrinsic file cannot be found!\n");
    cout << "extrinsic file is loaded...\n";

    // fill out params.
    // 0: cabin
    // 1: boom
    cv::Mat T_cl0_tmp;
    cv::Mat T_c0c1_tmp;
    Eigen::Matrix4f T_cl0_eigen_tmp;
    Eigen::Matrix4f T_c0c1_eigen_tmp; 
    fs["cabin.T_cl0"] >> T_cl0_tmp;
    fs["cabin.T_c0c1"] >> T_c0c1_tmp;

    cv::cv2eigen(T_cl0_tmp, T_cl0_eigen_tmp);
    cv::cv2eigen(T_c0c1_tmp, T_c0c1_eigen_tmp);  
    T_cl0_.push_back(T_cl0_eigen_tmp);
    T_c0c1_.push_back(T_c0c1_eigen_tmp);

    fs["boom.T_cl0"] >> T_cl0_tmp;
    fs["boom.T_c0c1"] >> T_c0c1_tmp;
    cv::cv2eigen(T_cl0_tmp, T_cl0_eigen_tmp);
    cv::cv2eigen(T_c0c1_tmp, T_c0c1_eigen_tmp);   
    T_cl0_.push_back(T_cl0_eigen_tmp);
    T_c0c1_.push_back(T_c0c1_eigen_tmp);
};

// 'run()' function is executed when 'GCS' requests profile 3D points.
// Thus, this function should be in the 'callback' function for service.
bool LidarVisualReconstructor::run(){
    if(client_lidarimagedata_.call(srv_lidarimagedata_)){
        ROS_INFO("Service is requested by 'lidar_visual_reconstructor' node.\n");

        // 
        
        // Delaunay ... 

        // KLT ...

        // Densification ...

        // Extract profile 3D points ... 

        // Respond to 'GCS' node.
        

        return true;
    }
    else{
        ROS_ERROR("Failed to call service by 'lidar_visual_reconstructor' node.\n");
        return false;
    }
};