#include "lidar_visual_reconstructor.hpp"

LidarVisualReconstructor::LidarVisualReconstructor(ros::NodeHandle& nh)
: nh_(nh)
{
    // TODO: get params from yaml or launch.
    MAX_LVL_PYR_ = 4;

    n_lidars_  = 2;
    n_cameras_ = 4;

    vector<int> n_channels;
    n_channels.push_back(16);
    n_channels.push_back(16);
    
    for(int i = 0; i <  n_lidars_; ++i) pcls_.push_back(new LidarPcl(n_channels[i]));
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

void LidarVisualReconstructor::limitRanges(){
    // manipulate mask!
    //
    float x_lims0[2] = {0, 50}; 
    float y_lims0[2] = {-10,10};
    float z_lims0[2] = {-5,3};
    float x_lims1[2] = {0,10};
    float y_lims1[2] = {-50,20};
    float z_lims1[2] = {-5,5};

    // 1) Simple range limits
    // 1-1) for lidar0 
    int cnt = 0;
    bool* mask_ptr = pcls_[0]->mask;
    for(int i = 0; i < pcls_[0]->count; ++i, ++mask_ptr){
        *mask_ptr = *mask_ptr &&
        *(pcls_[0]->x + i) > x_lims0[0] && *(pcls_[0]->x + i) < x_lims0[1] &&
        *(pcls_[0]->y + i) > y_lims0[0] && *(pcls_[0]->y + i) < y_lims0[1] &&
        *(pcls_[0]->z + i) > z_lims0[0] && *(pcls_[0]->z + i) < z_lims0[1];
        if(*mask_ptr) ++cnt;
    }
    cout << " valid 0 : "<< cnt << "\n";
    // 1-2) for lidar1
    cnt = 0;
    mask_ptr = pcls_[1]->mask;
    for(int i = 0; i < pcls_[1]->count; ++i, ++mask_ptr){
        *mask_ptr = *mask_ptr &&
        *(pcls_[1]->x + i) > x_lims1[0] && *(pcls_[1]->x + i) < x_lims1[1] &&
        *(pcls_[1]->y + i) > y_lims1[0] && *(pcls_[1]->y + i) < y_lims1[1] &&
        *(pcls_[1]->z + i) > z_lims1[0] && *(pcls_[1]->z + i) < z_lims1[1];
        if(*mask_ptr) ++cnt;
    }
    cout << " valid 1 : "<< cnt << "\n";

    // 2) Warp all points of LiDARs to cam0 frame.
    // And, in image test is also executed! 
    float x_lims_cam[2] = {-5,5}; 
    float y_lims_cam[2] = {-5,5};
    float z_lims_cam[2] = {0.2, 40};
    int n_cols = cams_[0]->cols();
    int n_rows = cams_[0]->rows();
    float invz = -1;
    Eigen::Vector3f X_tmp, X_warp; 
    Eigen::Vector2f pts_tmp;


    mask_ptr = pcls_[0]->mask;
    cnt = 0;
    for(int i = 0; i < pcls_[0]->count; ++i, ++mask_ptr){
        X_tmp(0) = *(pcls_[0]->x + i);
        X_tmp(1) = *(pcls_[0]->y + i);
        X_tmp(2) = *(pcls_[0]->z + i);
        
        X_warp = T_cl0_[0].block<3,3>(0,0)*X_tmp + T_cl0_[0].block<3,1>(0,3);

        invz = 1.0f/X_warp(2);
        pts_tmp(0) = cams_[0]->fx()*X_warp(0)*invz + cams_[0]->cx();
        pts_tmp(1) = cams_[0]->fy()*X_warp(1)*invz + cams_[0]->cy();
        
        *mask_ptr = *mask_ptr &&
        pts_tmp(0) > 0 && pts_tmp(0) < n_cols &&
        pts_tmp(1) > 0 && pts_tmp(1) < n_rows &&
        X_warp(0) > x_lims_cam[0] && X_warp(0) < x_lims_cam[1] &&
        X_warp(1) > y_lims_cam[0] && X_warp(1) < y_lims_cam[1] &&
        X_warp(2) > z_lims_cam[0] && X_warp(2) < z_lims_cam[1];
        if(*mask_ptr) ++cnt;
    } 
    cout << " valid 0 : "<< cnt << "\n";

    mask_ptr = pcls_[1]->mask;
    cnt = 0;
    for(int i = 0; i < pcls_[1]->count; ++i, ++mask_ptr) {
        X_tmp(0) = *(pcls_[1]->x + i);
        X_tmp(1) = *(pcls_[1]->y + i);
        X_tmp(2) = *(pcls_[1]->z + i);
        
        X_warp = T_cl0_[0].block<3,3>(0,0)*(T_l0l1_.block<3,3>(0,0)*X_tmp + T_l0l1_.block<3,1>(0,3))+ T_cl0_[0].block<3,1>(0,3);
        invz = 1.0f/X_warp(2);
        pts_tmp(0) = cams_[0]->fx()*X_warp(0)*invz + cams_[0]->cx();
        pts_tmp(1) = cams_[0]->fy()*X_warp(1)*invz + cams_[0]->cy();
        
        *mask_ptr = *mask_ptr &&
        pts_tmp(0) > 0 && pts_tmp(0) < n_cols &&
        pts_tmp(1) > 0 && pts_tmp(1) < n_rows &&
        X_warp(0) > x_lims_cam[0] && X_warp(0) < x_lims_cam[1] &&
        X_warp(1) > y_lims_cam[0] && X_warp(1) < y_lims_cam[1] &&
        X_warp(2) > z_lims_cam[0] && X_warp(2) < z_lims_cam[1];
        if(*mask_ptr) ++cnt;
    }
    cout << " valid 1 : "<< cnt << "\n";

    if(1){
        cv::Scalar orange(0, 165, 255), blue(255, 0, 0), magenta(255, 0, 255);

        cv::Mat img_8u;
        frames_[0]->img().convertTo(img_8u, CV_8UC3);
        mask_ptr = pcls_[0]->mask;
        for(int i = 0; i < pcls_[0]->count; ++i, ++mask_ptr){
            if(*mask_ptr){
                X_tmp(0) = *(pcls_[0]->x + i);
                X_tmp(1) = *(pcls_[0]->y + i);
                X_tmp(2) = *(pcls_[0]->z + i);
                
                X_warp = T_cl0_[0].block<3,3>(0,0)*X_tmp + T_cl0_[0].block<3,1>(0,3);

                float invz = 1.0f/X_warp(2);
                pts_tmp(0) = cams_[0]->fx()*X_warp(0)*invz + cams_[0]->cx();
                pts_tmp(1) = cams_[0]->fy()*X_warp(1    )*invz + cams_[0]->cy();
                cv::Point pt(pts_tmp(0),pts_tmp(1));
                cv::circle(img_8u, pt, 1, magenta);
            }
        }
        mask_ptr = pcls_[1]->mask;
        for(int i = 0; i < pcls_[1]->count; ++i,++mask_ptr){
            if(*mask_ptr){
                X_tmp(0) = *(pcls_[1]->x + i);
                X_tmp(1) = *(pcls_[1]->y + i);
                X_tmp(2) = *(pcls_[1]->z + i);

                X_warp = T_cl0_[0].block<3,3>(0,0)*(T_l0l1_.block<3,3>(0,0)*X_tmp + T_l0l1_.block<3,1>(0,3))+ T_cl0_[0].block<3,1>(0,3);

                float invz = 1.0f/X_warp(2);
                pts_tmp(0) = cams_[0]->fx()*X_warp(0)*invz + cams_[0]->cx();
                pts_tmp(1) = cams_[0]->fy()*X_warp(1)*invz + cams_[0]->cy();
                cv::Point pt(pts_tmp(0),pts_tmp(1));
                cv::circle(img_8u, pt, 1, magenta);
            }
        }

        cv::namedWindow("limitRanges", CV_WINDOW_AUTOSIZE);
        cv::imshow("limitRanges", img_8u);
        cv::waitKey(0);
    } //end if
    
    // delete invalid points (mask == 0)
    pcls_[0]->deleteMaskInvalid();
    pcls_[1]->deleteMaskInvalid();
    
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
        xi_l0l1(0) = srv_relativelidarpose_.response.vx;
        xi_l0l1(1) = srv_relativelidarpose_.response.vy;
        xi_l0l1(2) = srv_relativelidarpose_.response.vz;
        xi_l0l1(3) = srv_relativelidarpose_.response.wx;
        xi_l0l1(4) = srv_relativelidarpose_.response.wy;
        xi_l0l1(5) = srv_relativelidarpose_.response.wz;

        sophuslie::se3Exp(xi_l0l1, T_l0l1_);
        
        cout << " recon node T_l0l1: \n" << T_l0l1_ << "\n";
    }
    else{
        ROS_ERROR("Failed to call service 'GCS:relative lidar pose' by 'Recon node'.\n");
        return false;
    }

    // Request 2) Image and lidar data
    srv_lidarimagedata_.request.header.stamp = ros::Time::now();
    srv_lidarimagedata_.request.header.seq   = -1;
    srv_lidarimagedata_.request.header.frame_id = "Origin: recon_node";
    srv_lidarimagedata_.request.request_type    = 0;

    if(client_lidarimagedata_.call(srv_lidarimagedata_)){
        ROS_INFO("'GCS:lidar image data': OK. by 'Recon node'.\n");
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(srv_lidarimagedata_.response.img0,
            sensor_msgs::image_encodings::MONO8);
        frames_[0]->constructFrame(cv_ptr->image);
        cv_ptr = cv_bridge::toCvCopy(srv_lidarimagedata_.response.img1,
            sensor_msgs::image_encodings::MONO8);
        frames_[1]->constructFrame(cv_ptr->image);
        
        if(0) {
            cv::namedWindow("0 image", CV_WINDOW_AUTOSIZE);
            cv::namedWindow("1 image", CV_WINDOW_AUTOSIZE);
            cv::imshow("0 image", frames_[0]->img_raw());
            cv::imshow("1 image", frames_[1]->img_raw());
            cv::waitKey(1000); // both imshow s are independently affected by waitKey.
            // If there are two windows, total waiting time becomes 1000*2 = 2000 ms.
        }
       
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
            *(pcls_[0]->mask+i) = true;
        }
        
        pcls_[1]->count = res_lidarimage.n_pts1;
        for(int i = 0; i < pcls_[1]->count; ++i){
            *(pcls_[1]->x+i) = res_lidarimage.x1[i];
            *(pcls_[1]->y+i) = res_lidarimage.y1[i];
            *(pcls_[1]->z+i) = res_lidarimage.z1[i];
            *(pcls_[1]->ring+i) = res_lidarimage.ring1[i];
            *(pcls_[1]->time+i) = res_lidarimage.time1[i];
            *(pcls_[1]->intensity+i) = res_lidarimage.intensity1[i];
            *(pcls_[1]->mask+i) = true;
        }

        
        // find valid data within specific area (user-definable)        
        limitRanges();

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