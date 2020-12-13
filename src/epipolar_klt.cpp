#include "epipolar_klt.h"


EpipolarKLT::EpipolarKLT() 
{  
};

void EpipolarKLT::runEpipolarKLT(
            const vector<cv::Mat>& Ik, const vector<cv::Mat>& Ic, 
            const vector<cv::Mat>& du_k, const vector<cv::Mat>& dv_k,
            const vector<cv::Mat>& du_c, const vector<cv::Mat>& dv_c,
            const int& win_sz, const int& MAX_ITER,
            const float& logalpha, const float& beta, 
            vector<PointDB>& db) {
    // In this function, MAX_LVL, n_cols, n_rows are set.
    int MAX_PYR_LVL = Ik.size();
    vector<int> n_cols_pyr_;
    vector<int> n_rows_pyr_;

    cout << " EKLT: registered image lvl: " << MAX_PYR_LVL <<"\n";
    n_pts_ = db.size();
    cout << " EKLT: # of input points: " << n_pts_ << "\n";

    for(int i = 0; i < MAX_PYR_LVL; ++i) {
        n_cols_pyr_.push_back(Ik[i].cols);
        n_rows_pyr_.push_back(Ik[i].rows);
    }

    if(win_sz % 2 == 1) std::runtime_error("EKLT: a window size must be odd number!\n");
    int win_half = (win_sz - 1)/2; // my convention! 
    int M = (2 * win_half + 1)*(2 * win_half + 1);
    // M_ = 2*win_sz_*win_sz_ + 2*win_sz_ + 1; // fast setting.
    vector<cv::Point2f> patch;
    for (int u = -win_half; u < win_half + 1; ++u)
        for (int v = -win_half; v < win_half + 1; ++v)
            patch.emplace_back((float)u, (float)v);

    double* Ik_vector = new double[M];


    // Do EKLT. for all points!
    double thres_huber = 10.0;
    double SCALER      = 2.0;


    double J_tmp, r;
    for(int i = 0; i < n_pts_; ++i) {
        db[i].err_klt_normal_ = -1.0; // error initialization

        // Get points.
        double invpow = 1.0 / (pow(2,MAX_PYR_LVL-1));
        Eigen::Vector2f pt_near = db[i].pts_guess_near_*invpow;
        Eigen::Vector2f pt_far  = db[i].pts_guess_far_ *invpow;
        Eigen::Vector2f l       = pt_far - pt_near;
        double s_far = l.norm();
        l /= s_far;
        double s = (db[i].pts_prior_*invpow- pt_near).norm(); // from prior information.
        cout <<"["<<i<<"]pt near far: "<< pt_near(0) <<"," <<pt_near(1)<<"/" <<pt_far(0)<<"," <<pt_far(1) <<"\n";
        cout <<"prior: "<< db[i].pts_prior_(0)*invpow <<"," << db[i].pts_prior_(1)*invpow <<"\n"; 
        cout <<"["<<i<<"] start s: " << s<<endl;
        Eigen::Vector2f pt_k(db[i].pts_(0), db[i].pts_(1));
        pt_k *= invpow;

        Eigen::Vector3f interp_tmp;
        Eigen::Vector2f delta_pt;
        double Ic_warp, dx_warp, dy_warp, Ik_single, r, w, J;
        double JtwJ_scalar = 0, Jtwr_scalar = 0;
        double delta_s = 0, r2_sum = 0;
        bool a =true;
        for(int lvl = MAX_PYR_LVL-1; lvl > -1; --lvl){
            // calculate reference image brightness
            for(int j = 0; j < M; ++j)
                *(Ik_vector + j) = improc::interpImageSingle(Ik[lvl], patch[j].x + pt_k(0),  patch[j].y + pt_k(1));

            if(a && lvl == MAX_PYR_LVL-1){
                for(int j = 0; j < M; ++j)
                    cout<<*(Ik_vector+j)<<",";
                cout <<"\n";
                a=false;
            }
            
            //cout << "lvl i delta pt: "<<lvl<<", "<<i <<", "<< delta_pt(0) <<"," <<delta_pt(1)<<"\n";

            // ITERATION!!!
            for(int iter = 0; iter < MAX_ITER; ++iter){
                delta_pt = s*l;

                JtwJ_scalar = 0;
                Jtwr_scalar = 0;
                r2_sum = 0;
                // Generate reference patch. (M+1) residual.
                int cnt_valid = 0;
                for(int j = 0; j < M; ++j){
                    float u_warp = patch[j].x + pt_near(0) + delta_pt(0);
                    float v_warp = patch[j].y + pt_near(1) + delta_pt(1);

                    if((u_warp < 1) || (u_warp > n_cols_pyr_[lvl] || (v_warp < 1) || (v_warp > n_rows_pyr_[lvl]))) // outside of image.
                        continue;
                   
                    // calculate and push edge residual, Huber weight, Jacobian, and Hessian
                    improc::interpImageSingle3(Ic[lvl], du_c[lvl], dv_c[lvl], u_warp, v_warp, interp_tmp);
                    Ic_warp = interp_tmp(0);
                    dx_warp = interp_tmp(1);
                    dy_warp = interp_tmp(2);

                    //Ic_warp = improc::interpImageSingle(Ic[lvl],   u_warp, v_warp);
                    //dx_warp = improc::interpImageSingle(du_c[lvl], u_warp, v_warp);
                    //dy_warp = improc::interpImageSingle(dv_c[lvl], u_warp, v_warp);
                    
                    if(Ic_warp < 0 || *(Ik_vector + j) < 0) continue;

                     // valid pixel!
                    ++cnt_valid;
                    
                    r = Ic_warp - *(Ik_vector + j);
                    w = fabs(r) < thres_huber ? 1.0f : thres_huber / fabs(r);
                    J = dx_warp*l(0) + dy_warp*l(1);

                    JtwJ_scalar += w*J*J;
                    Jtwr_scalar += J*w*r;
                    r2_sum += r*r;
                }

                r = -sqrt(cnt_valid)*SCALER*(log(s) + log(-s+s_far) + 2.0f*log(0.5f*s_far));
                J = -sqrt(cnt_valid)*SCALER*(2*s-s_far)/(s*(s-s_far));
                w = fabs(r) < thres_huber ? 1.0f : thres_huber / fabs(r);
                JtwJ_scalar += w*J*J;
                Jtwr_scalar += J*w*r;

                // Calculate step size!
                delta_s = -Jtwr_scalar/JtwJ_scalar;
                // cout << " delta s: "<< delta_s <<"\n";
                s += delta_s;
                if(s < 0.01f) s = 0.01f;
                if(s > s_far) s = s_far - 0.01f;

                if(lvl == 0) {
                    db[i].pts_tracked_ << s*l(0) + pt_near(0), s*l(1) + pt_near(1);
                    db[i].err_klt_normal_ = sqrt(r2_sum/cnt_valid);
                }
                if( abs(delta_s) < pow(10.0f,-(5-0.25f*(lvl+1)) ) ){
                    if(lvl == 0){
                        if(db[i].err_klt_normal_ > 20.0f) db[i].klt_valid_ = false;
                    }
                    break;
                }
            }
            
            // resolution up.
            pt_k    *= 2;
            pt_near *= 2;
            pt_far  *= 2;
            s_far   *= 2;
            s       *= 2;
        }
        cout << "[" << i <<"]: s.. [" << 0 << "   " << s << "   "  << s_far << "]\n";
    }

    delete[] Ik_vector;

};

EpipolarKLT::~EpipolarKLT(){
};



// private methods
//
//
//
//
//
inline void EpipolarKLT::update(const Vec6& Jt, const float& r, const float& weight, float& err_ssd)
{
    // cout << "Jt.transpose()*Jt:\n" << Jt*(Jt.transpose()) << endl;
    JtWJ.noalias() += (Jt*Jt.transpose())*weight;
    mJtWr.noalias() -= Jt*(r*weight);
    err_ssd += r*r*weight;
}

void EpipolarKLT::solveGaussNewtonStep(Vec6& delta) {
    delta = JtWJ.ldlt().solve(mJtWr);
};