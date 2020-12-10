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

    // Construct Constrained DT without initialization.
    cdt_ = new ConstrainedDT(); // without initialization.
};

LidarVisualReconstructor::~LidarVisualReconstructor(){
    for(auto iter = pcls_.begin(); iter != pcls_.end(); ++iter)
        if(*iter != nullptr) delete *iter;
    for(auto iter = cams_.begin(); iter != cams_.end(); ++iter)
        if(*iter != nullptr) delete *iter;
    for(auto iter = frames_.begin(); iter != frames_.end(); ++iter)
        if(*iter != nullptr) delete *iter;
    if(cdt_ != nullptr) delete cdt_;
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
    // cout << " valid 0 : "<< cnt << "\n";
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
    // cout << " valid 1 : "<< cnt << "\n";

    // 2) Warp all points of LiDARs to cam0 frame.
    // And, in image test is also executed! 
    float x_lims_cam[2] = {-5,5}; 
    float y_lims_cam[2] = {-5,5};
    float z_lims_cam[2] = {0.2, 40};
    int n_cols = cams_[0]->cols();
    int n_rows = cams_[0]->rows();
    float invz = -1;
    EVec3f X_tmp, X_warp; 
    EVec2f pts_tmp;


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
    // cout << " valid 0 : "<< cnt << "\n";

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
    // cout << " valid 1 : "<< cnt << "\n";

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
    if(!fs.isOpened()) throw std::runtime_error("extrinsic file cannoEVec2fbe found!\n");
    cout << "extrinsic file is loaded...\n";

    // fill out params.
    // 0: cabin
    // 1: boom
    cv::Mat T_cl0_tmp;
    cv::Mat T_c0c1_tmp;
    EMat4f T_cl0_eigen_tmp;
    EMat4f T_c0c1_eigen_tmp; 
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

        // initialize all containers 
        db_.resize(0);

       
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

        // gather ring index into 'index_rings'
        pcls_[0]->gatherRingIndex();
        pcls_[1]->gatherRingIndex();
        
        // Calculate psi theta rho
        pcls_[0]->generateThetaPsi();
        pcls_[1]->generateThetaPsi(); // OK
        
        // Sort index_rings! (sorted by psi ascending order)
        pcls_[0]->sortRingIndexByPsi();
        pcls_[1]->sortRingIndexByPsi();

        // Find and fixing jumping index (add 2*pi).
        pcls_[0]->reorderingIndexAtJumping();
        pcls_[1]->reorderingIndexAtJumping();

        // Find intersections
        int MAX_ITER_DIVISION = 50; // TODO: global parameter.

        float dtheta_step = 2.0f; // 2.0 degrees
        vector<cv::Mat> idxs_cross;
        idxs_cross.push_back(cv::Mat::zeros(pcls_[0]->n_channels, pcls_[1]->n_channels,CV_32SC1));
        idxs_cross.push_back(cv::Mat::zeros(pcls_[0]->n_channels, pcls_[1]->n_channels,CV_32SC1));

        EVec3f X0_near, X0_mid, X0_far;
        EVec3f X1_near, X1_mid, X1_far;
        EVec3f X_temp;
        float sign_near, sign_mid, sign_far;

        EVec3f X0_m1, X0_p1;
        EVec3f X1_m1, X1_p1;
        float err_m1, err_mid, err_p1;

        for(int ch0 = 0; ch0 < pcls_[0]->n_channels; ++ch0) {
            float CC0 = std::tan((-15.0f+(float)ch0*dtheta_step)*D2R);
            for(int ch1 = 0; ch1 < pcls_[1]->n_channels; ++ch1) {
                int i_near = 0; 
                int i_far = pcls_[1]->index_rings[ch1].size() - 1;
                X_temp(0) = *(pcls_[1]->x + pcls_[1]->index_rings[ch1][i_near]); 
                X_temp(1) = *(pcls_[1]->y + pcls_[1]->index_rings[ch1][i_near]);
                X_temp(2) = *(pcls_[1]->z + pcls_[1]->index_rings[ch1][i_near]);
                X1_near = T_l0l1_.block<3,3>(0,0)*X_temp + T_l0l1_.block<3,1>(0,3);

                X_temp(0) = *(pcls_[1]->x + pcls_[1]->index_rings[ch1][i_far]);
                X_temp(1) = *(pcls_[1]->y + pcls_[1]->index_rings[ch1][i_far]);
                X_temp(2) = *(pcls_[1]->z + pcls_[1]->index_rings[ch1][i_far]);
                X1_far  = T_l0l1_.block<3,3>(0,0)*X_temp + T_l0l1_.block<3,1>(0,3);

                sign_near = X1_near(2) - CC0*sqrtf(X1_near(0)*X1_near(0) + X1_near(1)*X1_near(1));
                sign_far  = X1_far(2)  - CC0*sqrtf(X1_far(0)*X1_far(0)   + X1_far(1)*X1_far(1));

                if(sign_near * sign_far > 0){
                    idxs_cross[1].at<int>(ch0,ch1) = -1;
                    continue;
                }
                else{
                    for(int iter = 0; iter < MAX_ITER_DIVISION; ++iter){
                        int idx_mid = (int)std::floor((float)(i_near + i_far)/2.0f);
                        //cout << "near mid far: "<<i_near <<","<<idx_mid<<","<<i_far<<endl;
                        X_temp(0) = *(pcls_[1]->x + pcls_[1]->index_rings[ch1][idx_mid]);
                        X_temp(1) = *(pcls_[1]->y + pcls_[1]->index_rings[ch1][idx_mid]);
                        X_temp(2) = *(pcls_[1]->z + pcls_[1]->index_rings[ch1][idx_mid]);
                        X1_mid = T_l0l1_.block<3,3>(0,0)*X_temp + T_l0l1_.block<3,1>(0,3);

                        // test!
                        sign_mid = X1_mid(2) - CC0*sqrt(X1_mid(0)*X1_mid(0) + X1_mid(1)*X1_mid(1));
                        if(sign_mid * sign_near > 0) i_near = idx_mid;
                        else i_far = idx_mid;

                        if(abs(i_near - i_far) < 2){
                            // find nearest point.
                            X_temp(0) = *(pcls_[1]->x + pcls_[1]->index_rings[ch1][idx_mid-1]);
                            X_temp(1) = *(pcls_[1]->y + pcls_[1]->index_rings[ch1][idx_mid-1]);
                            X_temp(2) = *(pcls_[1]->z + pcls_[1]->index_rings[ch1][idx_mid-1]);
                            X1_m1 = T_l0l1_.block<3,3>(0,0)*X_temp + T_l0l1_.block<3,1>(0,3);
                            err_m1 = abs(X1_m1(2) - CC0*sqrt(X1_m1(0)*X1_m1(0) + X1_m1(1)*X1_m1(1)));

                            err_mid = abs(sign_mid);
                            X_temp(0) = *(pcls_[1]->x + pcls_[1]->index_rings[ch1][idx_mid+1]);
                            X_temp(1) = *(pcls_[1]->y + pcls_[1]->index_rings[ch1][idx_mid+1]);
                            X_temp(2) = *(pcls_[1]->z + pcls_[1]->index_rings[ch1][idx_mid+1]);
                            X1_p1 = T_l0l1_.block<3,3>(0,0)*X_temp + T_l0l1_.block<3,1>(0,3);
                            err_p1 = abs(X1_p1(2) - CC0*sqrt(X1_p1(0)*X1_p1(0) + X1_p1(1)*X1_p1(1)));
                            if(err_mid < err_p1){
                                if(err_mid < err_m1) idxs_cross[1].at<int>(ch0,ch1) = idx_mid; // mid
                                else idxs_cross[1].at<int>(ch0,ch1) = idx_mid - 1; // m1
                            }
                            else{
                                if(err_p1 < err_m1) idxs_cross[1].at<int>(ch0,ch1) = idx_mid+1; // p1
                                else idxs_cross[1].at<int>(ch0,ch1) = idx_mid - 1; // m1
                            }
                            break;
                        }
                    }
                }
            }
        } // end for lidar1

        EMat4f T_l1l0_ = T_l0l1_.inverse();
        for(int ch1 = 0; ch1 < pcls_[1]->n_channels; ++ch1) {
            float CC1 = std::tan((-15.0f+(float)ch1*dtheta_step)*D2R);
            for(int ch0 = 0; ch0 < pcls_[0]->n_channels; ++ch0) {
                int i_near = 0; 
                int i_far = pcls_[0]->index_rings[ch0].size() - 1;
                X_temp << *(pcls_[0]->x + pcls_[0]->index_rings[ch0][i_near]),
                          *(pcls_[0]->y + pcls_[0]->index_rings[ch0][i_near]),
                          *(pcls_[0]->z + pcls_[0]->index_rings[ch0][i_near]);
                X0_near = T_l1l0_.block<3,3>(0,0)*X_temp + T_l1l0_.block<3,1>(0,3);

                X_temp << *(pcls_[0]->x + pcls_[0]->index_rings[ch0][i_far]),
                          *(pcls_[0]->y + pcls_[0]->index_rings[ch0][i_far]),
                          *(pcls_[0]->z + pcls_[0]->index_rings[ch0][i_far]);
                X0_far  = T_l1l0_.block<3,3>(0,0)*X_temp + T_l1l0_.block<3,1>(0,3);

                sign_near = X0_near(2) - CC1*sqrt(X0_near(0)*X0_near(0) + X0_near(1)*X0_near(1));
                sign_far  = X0_far(2)  - CC1*sqrt(X0_far(0)*X0_far(0)   + X0_far(1)*X0_far(1));

                if(sign_near * sign_far > 0){
                    idxs_cross[0].at<int>(ch0,ch1) = -1;
                    continue;
                }
                else{
                    for(int iter = 0; iter < MAX_ITER_DIVISION; ++iter){
                        int idx_mid = (int)std::floor((float)(i_near + i_far)/2.0f);
                        X_temp << *(pcls_[0]->x + pcls_[0]->index_rings[ch0][idx_mid]),
                                  *(pcls_[0]->y + pcls_[0]->index_rings[ch0][idx_mid]),
                                  *(pcls_[0]->z + pcls_[0]->index_rings[ch0][idx_mid]);
                        X0_mid = T_l1l0_.block<3,3>(0,0)*X_temp +T_l1l0_.block<3,1>(0,3);
                        // test!
                        sign_mid = X0_mid(2) - CC1*sqrt(X0_mid(0)*X0_mid(0) + X0_mid(1)*X0_mid(1));
                        if(sign_mid * sign_near > 0) i_near = idx_mid;
                        else i_far = idx_mid;

                        if(abs(i_near - i_far) < 2){
                            // find nearest point.
                            X_temp << *(pcls_[0]->x + pcls_[0]->index_rings[ch0][idx_mid-1]),
                                    *(pcls_[0]->y + pcls_[0]->index_rings[ch0][idx_mid-1]),
                                    *(pcls_[0]->z + pcls_[0]->index_rings[ch0][idx_mid-1]);
                            X0_m1 = T_l1l0_.block<3,3>(0,0)*X_temp + T_l1l0_.block<3,1>(0,3);
                            err_m1 = abs(X0_m1(2) - CC1*sqrt(X0_m1(0)*X0_m1(0) + X0_m1(1)*X0_m1(1)));

                            err_mid = abs(sign_mid);

                            X_temp << *(pcls_[0]->x + pcls_[0]->index_rings[ch0][idx_mid+1]),
                                      *(pcls_[0]->y + pcls_[0]->index_rings[ch0][idx_mid+1]),
                                      *(pcls_[0]->z + pcls_[0]->index_rings[ch0][idx_mid+1]);
                            X0_p1 = T_l1l0_.block<3,3>(0,0)*X_temp + T_l1l0_.block<3,1>(0,3);
                            err_p1 = abs(X0_p1(2) - CC1*sqrt(X0_p1(0)*X0_p1(0) + X0_p1(1)*X0_p1(1)));
                            if(err_mid < err_p1)
                                if(err_mid < err_m1) idxs_cross[0].at<int>(ch0,ch1) = idx_mid; // mid
                                else idxs_cross[0].at<int>(ch0,ch1) = idx_mid - 1; // m1
                            else
                                if(err_p1 < err_m1) idxs_cross[0].at<int>(ch0,ch1) = idx_mid+1; // p1
                                else idxs_cross[0].at<int>(ch0,ch1) = idx_mid - 1; // m1

                            break;
                        }
                    }
                }
            }
        } // end for lidar0

        for(int ch0 = 0; ch0 < pcls_[0]->n_channels; ++ch0){
            for(int ch1 = 0; ch1 < pcls_[1]->n_channels; ++ch1){
                if(idxs_cross[0].at<int>(ch0,ch1) < 0 ||idxs_cross[1].at<int>(ch0,ch1) < 0){
                    idxs_cross[0].at<int>(ch0,ch1) = -1;
                    idxs_cross[1].at<int>(ch0,ch1) = -1;
                }
            }
        }
        // cout << "idxs_cross0:\n" << idxs_cross[0]<<"\n\n";
        // cout << "idxs_cross1:\n" << idxs_cross[1]<<"\n\n";


        // equidistant points insertions.
        float dist_step = 0.3f; // 0.3 [m] equidistances

        // For lidar0
        map<int, vector<int>> idxs_augment_l0;
        for(int ch0 = 0; ch0 < pcls_[0]->n_channels; ++ch0){
            for(int ch1 = 0; ch1 < pcls_[1]->n_channels+1; ++ch1){
                idxs_augment_l0.insert(std::pair<int,vector<int>>(ch1+ch0*(pcls_[1]->n_channels+1),vector<int>(0)));
            }
        }
        

        // Find augment points
        EVec3f X_start, X_end;
        EVec3f X_curr, dX;
        for(int ch0 = 0; ch0 < pcls_[0]->n_channels; ++ch0){
            // (1) Fill aug. points between intersections.
            for(int n_seg = 1; n_seg < pcls_[1]->n_channels; ++n_seg){
                int i_start = idxs_cross[0].at<int>(ch0, n_seg-1);
                int i_end   = idxs_cross[0].at<int>(ch0, n_seg);

                // if ch0 has an intersection with ch1 and the next intersection is not -1,
                if(i_start > -1 && i_end > -1){
                    int sign_stp = 1;
                    if(i_start > i_end) sign_stp = -1; // inverse direction
                    int idx_tmp = pcls_[0]->index_rings[ch0][i_start];
                    X_start << *(pcls_[0]->x + idx_tmp), *(pcls_[0]->y + idx_tmp), *(pcls_[0]->z + idx_tmp);
                    idx_tmp = pcls_[0]->index_rings[ch0][i_end];
                    X_end << *(pcls_[0]->x + idx_tmp), *(pcls_[0]->y + idx_tmp), *(pcls_[0]->z + idx_tmp);
                    float dist = (X_end-X_start).norm();
                    if(dist > 1.5*dist_step){
                        int n_steps = ceil(dist/dist_step);
                        float step_tmp = dist/(float)n_steps;
                        for(int jj = i_start; jj != i_end+sign_stp; jj += sign_stp){
                            idx_tmp = pcls_[0]->index_rings[ch0][jj];
                            X_curr << *(pcls_[0]->x + idx_tmp), *(pcls_[0]->y + idx_tmp), *(pcls_[0]->z + idx_tmp);
                            dX = X_curr - X_start;
                            if(dX.norm() > step_tmp){
                                idxs_augment_l0[n_seg+ch0*(pcls_[1]->n_channels+1)].push_back(jj);
                                X_start = X_curr;
                            }
                        } // end for
                        if((X_curr-X_end).norm() < step_tmp) idxs_augment_l0[n_seg+ch0*(pcls_[1]->n_channels+1)].pop_back();
                    }// end if
                }// end if
            }// end for n_seg
            
            
            // (2) Find front point. 
            int ch1_front = 0;
            int int_a, int_b;
            while(ch1_front < pcls_[1]->n_channels){
                if(idxs_cross[0].at<int>(ch0,ch1_front) > -1) break;
                ++ch1_front;
            }
            if(ch1_front > pcls_[1]->n_channels - 1) int_a = -1; // no intersection.
            else int_a = idxs_cross[0].at<int>(ch0,ch1_front);

            // (3) Find back point
            int ch1_back = pcls_[1]->n_channels-1;
            while(ch1_back > -1){
                if(idxs_cross[0].at<int>(ch0,ch1_back) > -1) break;
                --ch1_back;
            }
            if(ch1_back < 0) int_b = -1; // no intersection
            else int_b = idxs_cross[0].at<int>(ch0,ch1_back);

            // (4) Is index increasing? or decreasing ?
            int i_initial, i_final;
            if(int_a>int_b) { // decreasing
                i_initial = pcls_[0]->index_rings[ch0].size()-1;
                i_final = 0;
            } else { // increasing 
                i_final = pcls_[0]->index_rings[ch0].size()-1;
                i_initial = 0;
            }

            // (5) fill front area
            if(i_initial > -1 && int_a > -1){
                int sign_stp = 1;
                if(int_a < i_initial) sign_stp = -1;
                int idx_tmp = pcls_[0]->index_rings[ch0][i_initial];
                X_start << *(pcls_[0]->x + idx_tmp), *(pcls_[0]->y + idx_tmp), *(pcls_[0]->z + idx_tmp);
                idx_tmp = pcls_[0]->index_rings[ch0][int_a];
                X_end << *(pcls_[0]->x + idx_tmp), *(pcls_[0]->y + idx_tmp), *(pcls_[0]->z + idx_tmp);
                float dist = (X_end-X_start).norm();
                if(dist > 1.5*dist_step){
                    int n_steps = ceil(dist/dist_step);
                    float step_tmp = dist/(float)n_steps;
                    for(int jj = i_initial; jj != int_a+sign_stp; jj += sign_stp){
                        idx_tmp = pcls_[0]->index_rings[ch0][jj];
                        X_curr << *(pcls_[0]->x + idx_tmp), *(pcls_[0]->y + idx_tmp), *(pcls_[0]->z + idx_tmp);
                        dX = X_curr - X_start;
                        if(dX.norm() > step_tmp){
                            idxs_augment_l0[ch1_front+ch0*(pcls_[1]->n_channels+1)].push_back(jj);
                            X_start = X_curr;
                        }
                    } // end for
                    if((X_curr-X_end).norm() < step_tmp) idxs_augment_l0[ch1_front+ch0*(pcls_[1]->n_channels+1)].pop_back();
                }// end if
            }

            // (6) fill back area
            if(int_b > -1 && i_final > -1){
                int sign_stp = 1;
                if(int_b > i_final) sign_stp = -1;
                int idx_tmp = pcls_[0]->index_rings[ch0][int_b];
                X_start << *(pcls_[0]->x + idx_tmp), *(pcls_[0]->y + idx_tmp), *(pcls_[0]->z + idx_tmp);
                idx_tmp = pcls_[0]->index_rings[ch0][i_final];
                X_end << *(pcls_[0]->x + idx_tmp), *(pcls_[0]->y + idx_tmp), *(pcls_[0]->z + idx_tmp);
                float dist = (X_end-X_start).norm();
                if(dist > 1.5*dist_step){
                    int n_steps = ceil(dist/dist_step);
                    float step_tmp = dist/(float)n_steps;
                    for(int jj = int_b; jj != i_final+sign_stp; jj += sign_stp){
                        idx_tmp = pcls_[0]->index_rings[ch0][jj];
                        X_curr << *(pcls_[0]->x + idx_tmp), *(pcls_[0]->y + idx_tmp), *(pcls_[0]->z + idx_tmp);
                        dX = X_curr - X_start;
                        if(dX.norm() > step_tmp){
                            idxs_augment_l0[(ch1_back+1)+ch0*(pcls_[1]->n_channels+1)].push_back(jj);
                            X_start = X_curr;
                        }
                    } // end for
                    if((X_curr-X_end).norm() < step_tmp) idxs_augment_l0[(ch1_back+1)+ch0*(pcls_[1]->n_channels+1)].pop_back();
                }   
            }
        }// end for ch0

        // // visualization idxs_augment_l0 and idxs_augment_l1
        // for(int ch0 = 0; ch0 < pcls_[0]->n_channels; ++ch0){
        //     for(int ch1 = 0; ch1 < pcls_[1]->n_channels+1; ++ch1){
        //         int n_elem = idxs_augment_l0[ch1 + ch0*(pcls_[1]->n_channels+1)].size();
        //         cout << "[";
        //         //for(auto itr = idxs_augment_l0[ch1 + ch0*(pcls_[1]->n_channels+1)].begin(); itr != idxs_augment_l0[ch1 + ch0*(pcls_[1]->n_channels+1)].end(); ++itr)
        //         //    cout << *itr <<", ";
        //         cout << idxs_augment_l0[ch1 + ch0*(pcls_[1]->n_channels+1)].size();
        //         cout <<"],";
        //     }
        //     cout << "\n";
        // }



        // For lidar1
        map<int, vector<int>> idxs_augment_l1;
        for(int ch1 = 0; ch1 < pcls_[1]->n_channels; ++ch1){
            for(int ch0 = 0; ch0 < pcls_[0]->n_channels+1; ++ch0){
                idxs_augment_l1.insert(std::pair<int,vector<int>>(ch0+ch1*(pcls_[0]->n_channels+1),vector<int>(0)));
            }
        }
        

        // Find augment points
        for(int ch1 = 0; ch1 < pcls_[1]->n_channels; ++ch1)
        { // (1) Fill aug. points between intersections.
            for(int n_seg = 1; n_seg < pcls_[0]->n_channels; ++n_seg)
            {
                int i_start = idxs_cross[1].at<int>(n_seg-1, ch1);
                int i_end   = idxs_cross[1].at<int>(n_seg,   ch1);

                // if ch1 has an intersection with ch1 and the next intersection is not -1,
                if(i_start > -1 && i_end > -1)
                {
                    int sign_stp = 1;
                    if(i_start > i_end) sign_stp = -1; // inverse direction

                    int idx_tmp = pcls_[1]->index_rings[ch1][i_start];
                    X_start <<  *(pcls_[1]->x + idx_tmp), *(pcls_[1]->y + idx_tmp), *(pcls_[1]->z + idx_tmp);
                    idx_tmp     = pcls_[1]->index_rings[ch1][i_end];
                    X_end   <<  *(pcls_[1]->x + idx_tmp), *(pcls_[1]->y + idx_tmp), *(pcls_[1]->z + idx_tmp);
                    float dist = (X_end-X_start).norm();
                    if(dist > 1.5*dist_step)
                    {
                        int n_steps = ceil(dist/dist_step);
                        float step_tmp = dist/(float)n_steps;

                        for(int jj = i_start; jj != i_end+sign_stp; jj += sign_stp) // i_end cannot be reached!!!!!
                        {
                            idx_tmp = pcls_[1]->index_rings[ch1][jj];
                            X_curr << *(pcls_[1]->x + idx_tmp), *(pcls_[1]->y + idx_tmp), *(pcls_[1]->z + idx_tmp);
                            dX = X_curr - X_start;
                            if(dX.norm() > step_tmp)
                            {
                                idxs_augment_l1[n_seg+ch1*(pcls_[0]->n_channels+1)].push_back(jj);
                                X_start = X_curr;
                            }
                        } // end for
                        
                        if((X_curr-X_end).norm() < step_tmp) idxs_augment_l1[n_seg+ch1*(pcls_[0]->n_channels+1)].pop_back();
                    }// end if
                }// end if
            }// end for n_seg
            
            
            // (2) Find front point. 
            int ch0_front = 0;
            int int_a, int_b;
            while(ch0_front < pcls_[0]->n_channels){
                if(idxs_cross[1].at<int>(ch0_front, ch1) > -1) break;
                ++ch0_front;
            }
            if(ch0_front > pcls_[0]->n_channels - 1) int_a = -1; // no intersection.
            else int_a = idxs_cross[1].at<int>(ch0_front,ch1);

            // (3) Find back point
            int ch0_back = pcls_[0]->n_channels-1;
            while(ch0_back > -1){
                if(idxs_cross[1].at<int>(ch0_back,  ch1) > -1) break;
                --ch0_back;
            }
            if(ch0_back < 0) int_b = -1; // no intersection
            else int_b = idxs_cross[1].at<int>(ch0_back,ch1);

            // (4) Is index increasing? or decreasing ?
            int i_initial, i_final;
            if(int_a > int_b) { // decreasing
                i_initial = pcls_[1]->index_rings[ch1].size()-1;
                i_final   = 0;
            } else { // increasing 
                i_final   = pcls_[1]->index_rings[ch1].size()-1;
                i_initial = 0;
            }
        
            // (5) fill front area
            if(i_initial > -1 && int_a > -1){
                int sign_stp = 1;
                if(int_a < i_initial) sign_stp = -1;
                int idx_tmp = pcls_[1]->index_rings[ch1][i_initial];
                X_start << *(pcls_[1]->x + idx_tmp), *(pcls_[1]->y + idx_tmp), *(pcls_[1]->z + idx_tmp);
                idx_tmp = pcls_[1]->index_rings[ch1][int_a];
                X_end << *(pcls_[1]->x + idx_tmp), *(pcls_[1]->y + idx_tmp), *(pcls_[1]->z + idx_tmp);
                float dist = (X_end-X_start).norm();
                if(dist > 1.5*dist_step){
                    int n_steps = ceil(dist/dist_step);
                    float step_tmp = dist/(float)n_steps;
                    for(int jj = i_initial; jj != int_a+sign_stp; jj += sign_stp){
                        idx_tmp = pcls_[1]->index_rings[ch1][jj];
                        X_curr << *(pcls_[1]->x + idx_tmp), *(pcls_[1]->y + idx_tmp), *(pcls_[1]->z + idx_tmp);
                        dX = X_curr - X_start;
                        if(dX.norm() > step_tmp){
                            idxs_augment_l1[ch0_front+ch1*(pcls_[0]->n_channels+1)].push_back(jj);
                            X_start = X_curr;
                        }
                    } // end for
                    if((X_curr-X_end).norm() < step_tmp) idxs_augment_l1[ch0_front+ch1*(pcls_[0]->n_channels+1)].pop_back();
                }// end if
            }

            // (6) fill back area
            if(int_b > -1 && i_final > -1){
                int sign_stp = 1;
                if(int_b > i_final) sign_stp = -1;
                int idx_tmp = pcls_[1]->index_rings[ch1][int_b];
                X_start << *(pcls_[1]->x + idx_tmp), *(pcls_[1]->y + idx_tmp), *(pcls_[1]->z + idx_tmp);
                idx_tmp = pcls_[1]->index_rings[ch1][i_final];
                X_end << *(pcls_[1]->x + idx_tmp), *(pcls_[1]->y + idx_tmp), *(pcls_[1]->z + idx_tmp);
                float dist = (X_end-X_start).norm();
                if(dist > 1.5*dist_step){
                    int n_steps = ceil(dist/dist_step);
                    float step_tmp = dist/(float)n_steps;
                    for(int jj = int_b; jj != i_final+sign_stp; jj += sign_stp){
                        idx_tmp = pcls_[1]->index_rings[ch1][jj];
                        X_curr << *(pcls_[1]->x + idx_tmp), *(pcls_[1]->y + idx_tmp), *(pcls_[1]->z + idx_tmp);
                        dX = X_curr - X_start;
                        if(dX.norm() > step_tmp){
                            idxs_augment_l1[(ch0_back+1)+ch1*(pcls_[0]->n_channels+1)].push_back(jj);
                            X_start = X_curr;
                        }
                    } // end for
                    if((X_curr-X_end).norm() < step_tmp) idxs_augment_l1[(ch0_back+1)+ch1*(pcls_[0]->n_channels+1)].pop_back();
                }   
            }
        }// end for ch0

        // // visualization idxs_augment_l0 and idxs_augment_l1
        // for(int ch1 = 0; ch1 < pcls_[1]->n_channels; ++ch1){
        //     for(int ch0 = 0; ch0 < pcls_[0]->n_channels+1; ++ch0){
        //         int n_elem = idxs_augment_l1[ch0 + ch1*(pcls_[0]->n_channels+1)].size();
        //         cout << "[";
        //         //for(auto itr = idxs_augment_l1[ch0 + ch1*(pcls_[0]->n_channels+1)].begin(); itr != idxs_augment_l1[ch0 + ch1*(pcls_[0]->n_channels+1)].end(); ++itr)
        //         //    cout << *itr <<", ";
        //         cout <<idxs_augment_l1[ch0+ch1*(pcls_[0]->n_channels+1)].size();
                
        //         cout <<"],";
        //     }
        //     cout << "\n";
        // }


        // Visualization on the figure.
        if(1   ){
            cv::Scalar orange(0, 165, 255), blue(255, 0, 0), magenta(255, 0, 255);

            cv::Mat img_8u;
            cv::cvtColor(frames_[0]->img(),img_8u,CV_GRAY2BGR);
            EVec3f X_tmp, X_warp;
            EVec2f pts_tmp;
            for(int ch0 = 0; ch0 < pcls_[0]->n_channels; ++ch0){
                for(int ch1 =0; ch1 < pcls_[1]->n_channels; ++ch1){
                    int idx_tmp = pcls_[0]->index_rings[ch0][idxs_cross[0].at<int>(ch0,ch1)];

                    X_tmp(0) = *(pcls_[0]->x + idx_tmp);
                    X_tmp(1) = *(pcls_[0]->y + idx_tmp);
                    X_tmp(2) = *(pcls_[0]->z + idx_tmp);
                    X_warp = T_cl0_[0].block<3,3>(0,0)*X_tmp + T_cl0_[0].block<3,1>(0,3);

                    float invz = 1.0f/X_warp(2);
                    pts_tmp(0) = cams_[0]->fx()*X_warp(0)*invz + cams_[0]->cx();
                    pts_tmp(1) = cams_[0]->fy()*X_warp(1)*invz + cams_[0]->cy();
                    cv::Point pt(pts_tmp(0),pts_tmp(1));
                    cv::circle(img_8u, pt, 2, magenta);
                }
            }
            for(int ch0 = 0; ch0 < pcls_[0]->n_channels; ++ch0){
                for(int ch1 =0; ch1 < pcls_[1]->n_channels; ++ch1){
                    int idx_tmp = pcls_[1]->index_rings[ch1][idxs_cross[1].at<int>(ch0,ch1)];

                    X_tmp(0) = *(pcls_[1]->x + idx_tmp);
                    X_tmp(1) = *(pcls_[1]->y + idx_tmp);
                    X_tmp(2) = *(pcls_[1]->z + idx_tmp);
                    X_warp = T_cl0_[0].block<3,3>(0,0)*(T_l0l1_.block<3,3>(0,0)*X_tmp + T_l0l1_.block<3,1>(0,3))+ T_cl0_[0].block<3,1>(0,3);

                    float invz = 1.0f/X_warp(2);
                    pts_tmp(0) = cams_[0]->fx()*X_warp(0)*invz + cams_[0]->cx();
                    pts_tmp(1) = cams_[0]->fy()*X_warp(1)*invz + cams_[0]->cy();
                    cv::Point pt(pts_tmp(0),pts_tmp(1));
                    cv::circle(img_8u, pt, 2, magenta);
                }
            }
            for(int ch1 = 0; ch1 < pcls_[1]->n_channels; ++ch1){
                for(int ch0 = 0; ch0 < pcls_[0]->n_channels+1; ++ch0){
                    int n_elem = idxs_augment_l1[ch0 + ch1*(pcls_[0]->n_channels+1)].size();
                    for(auto itr = idxs_augment_l1[ch0 + ch1*(pcls_[0]->n_channels+1)].begin(); itr != idxs_augment_l1[ch0 + ch1*(pcls_[0]->n_channels+1)].end(); ++itr){
                        int idx_tmp = pcls_[1]->index_rings[ch1][*itr];
                        X_tmp(0) = *(pcls_[1]->x + idx_tmp);
                        X_tmp(1) = *(pcls_[1]->y + idx_tmp);
                        X_tmp(2) = *(pcls_[1]->z + idx_tmp);
                        X_warp = T_cl0_[0].block<3,3>(0,0)*(T_l0l1_.block<3,3>(0,0)*X_tmp + T_l0l1_.block<3,1>(0,3))+ T_cl0_[0].block<3,1>(0,3);

                        float invz = 1.0f/X_warp(2);
                        pts_tmp(0) = cams_[0]->fx()*X_warp(0)*invz + cams_[0]->cx();
                        pts_tmp(1) = cams_[0]->fy()*X_warp(1)*invz + cams_[0]->cy();
                        cv::Point pt(pts_tmp(0),pts_tmp(1));
                        cv::circle(img_8u, pt, 1, magenta);
                    }                    
                }
            }
            for(int ch0 = 0; ch0 < pcls_[0]->n_channels; ++ch0){
                for(int ch1 = 0; ch1 < pcls_[1]->n_channels+1; ++ch1){
                    int n_elem = idxs_augment_l0[ch1 + ch0*(pcls_[1]->n_channels+1)].size();
                    for(auto itr = idxs_augment_l0[ch1 + ch0*(pcls_[1]->n_channels+1)].begin(); itr != idxs_augment_l0[ch1 + ch0*(pcls_[1]->n_channels+1)].end(); ++itr){
                        int idx_tmp = pcls_[0]->index_rings[ch0][*itr];
                        X_tmp(0) = *(pcls_[0]->x + idx_tmp);
                        X_tmp(1) = *(pcls_[0]->y + idx_tmp);
                        X_tmp(2) = *(pcls_[0]->z + idx_tmp);
                        X_warp = T_cl0_[0].block<3,3>(0,0)*X_tmp + T_cl0_[0].block<3,1>(0,3);

                        float invz = 1.0f/X_warp(2);
                        pts_tmp(0) = cams_[0]->fx()*X_warp(0)*invz + cams_[0]->cx();
                        pts_tmp(1) = cams_[0]->fy()*X_warp(1)*invz + cams_[0]->cy();
                        cv::Point pt(pts_tmp(0),pts_tmp(1));
                        cv::circle(img_8u, pt, 1, magenta);
                    }                    
                }
            }

            cv::namedWindow("intersections", CV_WINDOW_AUTOSIZE);
            cv::imshow("intersections", img_8u);
            cv::waitKey(0);
        } //end if        

        // Extract all intersecting points!
        // (1) add intersects
        int cnt_tmp = 0;
        for(int ch0 = 0; ch0 < pcls_[0]->n_channels; ++ch0) {
            for(int ch1 = 0; ch1 < pcls_[1]->n_channels; ++ch1) {
                if(idxs_cross[0].at<int>(ch0,ch1) > -1){
                    int idx0 = pcls_[0]->index_rings[ch0][idxs_cross[0].at<int>(ch0,ch1)];
                    int idx1 = pcls_[1]->index_rings[ch1][idxs_cross[1].at<int>(ch0,ch1)];
                    db_.emplace_back();
                    db_[cnt_tmp].X_ <<
                        *(pcls_[0]->x + idx0), *(pcls_[0]->y + idx0), *(pcls_[0]->z + idx0);
                    db_[cnt_tmp].state_ = 1;
                    db_[cnt_tmp].info_ << 0, ch0, idx0, idx1;
                    ++cnt_tmp;
                }
            }
        }
        // (2) add lidar0 augment points
        for(int ch0 = 0; ch0 < pcls_[0]->n_channels; ++ch0) {
            for(int n_seg = 0; n_seg < pcls_[1]->n_channels+1; ++n_seg) {
                if(idxs_augment_l0[n_seg+ch0*(pcls_[1]->n_channels+1)].size() > 0){
                    for(auto itr = idxs_augment_l0[n_seg+ch0*(pcls_[1]->n_channels+1)].begin(); 
                            itr != idxs_augment_l0[n_seg+ch0*(pcls_[1]->n_channels+1)].end(); ++itr){
                        int idx0 = pcls_[0]->index_rings[ch0][*itr];
                        db_.emplace_back();
                        db_[cnt_tmp].X_ <<
                            *(pcls_[0]->x + idx0), *(pcls_[0]->y + idx0), *(pcls_[0]->z + idx0);
                        db_[cnt_tmp].state_ = 0;
                        db_[cnt_tmp].info_ << 0, ch0, idx0, -1;
                        ++cnt_tmp;
                    }
                }
            }
        }

        // (3) add lidar1 augment points
        EVec3f X_tmp;
        for(int ch1 = 0; ch1 < pcls_[1]->n_channels; ++ch1) {
            for(int n_seg = 0; n_seg < pcls_[0]->n_channels+1; ++n_seg) {
                if(idxs_augment_l1[n_seg+ch1*(pcls_[0]->n_channels+1)].size() > 0){
                    for(auto itr = idxs_augment_l1[n_seg+ch1*(pcls_[0]->n_channels+1)].begin(); 
                            itr != idxs_augment_l1[n_seg+ch1*(pcls_[0]->n_channels+1)].end(); ++itr){
                        int idx1 = pcls_[1]->index_rings[ch1][*itr];
                        db_.emplace_back();
                        X_tmp << *(pcls_[1]->x + idx1), *(pcls_[1]->y + idx1), *(pcls_[1]->z + idx1);
                        db_[cnt_tmp].X_ = T_l0l1_.block<3,3>(0,0)*X_tmp + T_l0l1_.block<3,1>(0,3);
                        db_[cnt_tmp].state_ = 0;
                        db_[cnt_tmp].info_ << 1, ch1, idx1, -1;
                        ++cnt_tmp;
                    }
                }
            }
        }      

        // Projections.
        EVec3f X_warp;
        EVec2f pts_tmp;
        cout << " Pixel points: \n";
        for(auto itr = db_.begin(); itr != db_.end(); ++itr){
            EVec3f X_warp = T_cl0_[0].block<3,3>(0,0)*(itr->X_) + T_cl0_[0].block<3,1>(0,3);
            float invz = 1.0f/X_warp(2);
            itr->pts_(0) = cams_[0]->fx()*X_warp(0)*invz + cams_[0]->cx();
            itr->pts_(1) = cams_[0]->fy()*X_warp(1)*invz + cams_[0]->cy();
            cout << itr->pts_(0) << "," << itr->pts_(1) << "\n";
        }

        // Visualization on the figure.
        if(1){
            cv::Scalar orange(0, 165, 255), blue(255, 0, 0), magenta(255, 0, 255);
            cv::Mat img_8u;
            cv::cvtColor(frames_[0]->img(),img_8u,CV_GRAY2BGR);
            for(auto itr = db_.begin(); itr != db_.end(); ++itr)
                cv::circle(img_8u, cv::Point(itr->pts_(0),itr->pts_(1)), 3, magenta);
            cv::namedWindow("Final points", CV_WINDOW_AUTOSIZE);
            cv::imshow("Final points", img_8u);
            cv::waitKey(0);
        } //end if  

        // Delaunay ... 
        cdt_->initializeDT(db_);
        cdt_->executeNormalDT();
        cdt_->showAllTriangles();

        if(1){
            cv::Scalar orange(0, 165, 255), blue(255, 0, 0), magenta(255, 0, 255);
            cv::Mat img_8u;
            cv::cvtColor(frames_[0]->img(),img_8u,CV_GRAY2BGR);
            for(auto itr = cdt_->getTriangleMap().begin(); itr != cdt_->getTriangleMap().end(); ++itr){
                int i0 = itr->second->idx[0];
                int i1 = itr->second->idx[1];
                int i2 = itr->second->idx[2];
                // cv::line(img_8u, 
                //     cv::Point(db_[i0].pts_(0), db_[i0].pts_(1)),
                //     cv::Point(db_[i1].pts_(0), db_[i1].pts_(1)), blue, 3, CV_AA);
                // cv::line(img_8u, 
                //     cv::Point(db_[i1].pts_(0), db_[i1].pts_(1)),
                //     cv::Point(db_[i2].pts_(0), db_[i2].pts_(1)), blue, 3, CV_AA);
                // cv::line(img_8u, 
                //     cv::Point(db_[i0].pts_(0), db_[i0].pts_(1)),
                //     cv::Point(db_[i2].pts_(0), db_[i2].pts_(1)), blue, 3, CV_AA);
            }
            for(auto itr = db_.begin(); itr != db_.end(); ++itr)
                cv::circle(img_8u, cv::Point(itr->pts_(0),itr->pts_(1)), 3, magenta);
            
            cv::namedWindow("Delaunay results", CV_WINDOW_AUTOSIZE);
            cv::imshow("Delaunay results", img_8u);
            cv::waitKey(0);
        }


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