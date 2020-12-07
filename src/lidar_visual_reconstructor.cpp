#include "lidar_visual_reconstructor.hpp"

LidarVisualReconstructor::LidarVisualReconstructor(ros::NodeHandle& nh)
: nh_(nh)
{
    // TODO: get params from yaml or launch.
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
    client_lidarimagedata_    = nh_.serviceClient<hce_autoexcavator::lidarImageDataStamped>("/gcs_node/srv_lidar_image_data");
    client_relativelidarpose_ = nh_.serviceClient<hce_autoexcavator::relativeLidarPoseStamped>("/gcs_node/srv_relative_lidar_pose");
};

LidarVisualReconstructor::~LidarVisualReconstructor(){
    for(auto iter = pcls_.begin(); iter != pcls_.end(); ++iter)
        if(*iter != nullptr) delete *iter;
    for(auto iter = cams_.begin(); iter != cams_.end(); ++iter)
        if(*iter != nullptr) delete *iter;
    for(auto iter = frames_.begin(); iter != frames_.end(); ++iter)
        if(*iter != nullptr) delete *iter;
};

bool LidarVisualReconstructor::serverCallbackProfilePoints(hce_autoexcavator::profilePointsStamped::Request &req,
        hce_autoexcavator::profilePointsStamped::Response &res)
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
    // Request 1) real-time relative lidar pose data (induced by boom angle)
    srv_relativelidarpose_.request.header.stamp = ros::Time::now();
    if(client_relativelidarpose_.call(srv_relativelidarpose_)){
        ROS_INFO("'GCS:relative lidar pose': OK. by 'Recon node'.\n");

        Eigen::Matrix<float,6,1> xi_l0l1;
        Eigen::Matrix4f T_l0l1;
        xi_l0l1(0) = srv_relativelidarpose_.response.vx;
        xi_l0l1(1) = srv_relativelidarpose_.response.vy;
        xi_l0l1(2) = srv_relativelidarpose_.response.vz;
        xi_l0l1(3) = srv_relativelidarpose_.response.wx;
        xi_l0l1(4) = srv_relativelidarpose_.response.wy;
        xi_l0l1(5) = srv_relativelidarpose_.response.wz;

        sophuslie::se3Exp(xi_l0l1, T_l0l1);
        
        cout << " recon node T_l0l1: \n" << T_l0l1 << "\n";
    }
    else{
        ROS_ERROR("Failed to call service 'GCS:relative lidar pose' by 'Recon node'.\n");
        return false;
    }

    // Request 2) Image and lidar data
    srv_lidarimagedata_.request.header.stamp = ros::Time::now();
    srv_lidarimagedata_.request.header.seq = -1;
    srv_lidarimagedata_.request.header.frame_id = "Origin: recon_node";
    srv_lidarimagedata_.request.request_type = 0;

    if(client_lidarimagedata_.call(srv_lidarimagedata_)){
        ROS_INFO("'GCS:lidar image data': OK. by 'Recon node'.\n");
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(srv_lidarimagedata_.response.img0,
            sensor_msgs::image_encodings::MONO8);
        frames_[0]->constructFrame(cv_ptr->image);
        cv_ptr = cv_bridge::toCvCopy(srv_lidarimagedata_.response.img1,
            sensor_msgs::image_encodings::MONO8);
        frames_[1]->constructFrame(cv_ptr->image);
        
        cv::namedWindow("0 image", CV_WINDOW_AUTOSIZE);
        cv::namedWindow("1 image", CV_WINDOW_AUTOSIZE);
        cv::imshow("0 image", frames_[0]->img());
        cv::imshow("1 image", frames_[1]->img());
        cv::waitKey(1000); // both imshow s are independently affected by waitKey.
        // If there are two windows, total waiting time becomes 1000*2 = 2000 ms.
        
        // fill out LidarPcl.
        hce_autoexcavator::lidarImageDataStamped::Response& res_lidarimage = srv_lidarimagedata_.response;
        if(pcls_[0]->n_channels != res_lidarimage.n_channels0) throw std::runtime_error("Not matched dimension (recon node)\n");
        if(pcls_[1]->n_channels != res_lidarimage.n_channels1) throw std::runtime_error("Not matched dimension (recon node)\n");
        
        pcls_[0]->count = res_lidarimage.n_pts0;
        for(int i = 0; i < pcls_[0]->count; ++i){
            *(pcls_[0]->x+i) = res_lidarimage.x0[i];
            *(pcls_[0]->y+i) = res_lidarimage.y0[i];
            *(pcls_[0]->z+i) = res_lidarimage.z0[i];
            *(pcls_[0]->ring+i) = res_lidarimage.ring0[i];
            *(pcls_[0]->time+i) = res_lidarimage.time0[i];
            *(pcls_[0]->intensity+i) = res_lidarimage.intensity0[i];
        }
       
        pcls_[1]->count = res_lidarimage.n_pts1;
        for(int i = 0; i < pcls_[1]->count; ++i){
            *(pcls_[1]->x+i) = res_lidarimage.x1[i];
            *(pcls_[1]->y+i) = res_lidarimage.y1[i];
            *(pcls_[1]->z+i) = res_lidarimage.z1[i];
            *(pcls_[1]->ring+i) = res_lidarimage.ring1[i];
            *(pcls_[1]->time+i) = res_lidarimage.time1[i];
            *(pcls_[1]->intensity+i) = res_lidarimage.intensity1[i];
        }

        // TODO: find valid data within specific area (user-definable)        

        // TODO: Calculate psi theta, and sort index_rings!
        
        // 
        
        // Delaunay ... 

        // KLT ...

        // Densification ...

        // Extract profile 3D points ... 

        // Respond to 'GCS' node.
        

        return true;
    }
    else{
        ROS_ERROR("Failed to call service 'GCS:lidar image data' by 'Recon node'.\n");
        return false;
    }
};