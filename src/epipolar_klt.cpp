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
    n_pts_ = db.size();
#ifdef _VERBOSE_
    cout << " EKLT: registered image lvl: " << MAX_PYR_LVL <<"\n";
    cout << " EKLT: # of input points: " << n_pts_ << "\n";
#endif

    if(win_sz % 2 == 1) std::runtime_error("EKLT: a window size must be odd number!\n");
    int win_half = (win_sz - 1)/2; // my convention! 
    int M = (2 * win_half + 1)*(2 * win_half + 1);
    // M_ = 2*win_sz_*win_sz_ + 2*win_sz_ + 1; // fast setting.
    vector<cv::Point2f> patch;
    patch.reserve(M);
    for (int u = -win_half; u < win_half + 1; ++u)
        for (int v = -win_half; v < win_half + 1; ++v)
            patch.emplace_back((float)u, (float)v);

    float* Ik_vector = new float[M];


    // Do EKLT. for all points!
    float thres_huber = 10.0;
    float SCALER      = 2.0;


    float J_tmp, r;
    Eigen::Vector2f pt_near, pt_far, l, pt_k;
    Eigen::Vector3f interp_tmp;
    Eigen::Vector2f delta_pt;
    for(int i = 0; i < n_pts_; ++i) {
        db[i].err_klt_normal_ = -1.0; // error initialization

        // Get points.
        float invpow = 1.0 / (pow(2,MAX_PYR_LVL-1));
        pt_near = db[i].pts_guess_near_*invpow;
        pt_far  = db[i].pts_guess_far_ *invpow;
        l       = pt_far - pt_near;
        float s_far = l.norm();
        l /= s_far;
        float s = (db[i].pts_prior_*invpow- pt_near).norm(); // from prior information.
        // cout <<"["<<i<<"]pt near far: "<< pt_near(0) <<"," <<pt_near(1)<<"/" <<pt_far(0)<<"," <<pt_far(1) <<"\n";
        // cout <<"prior: "<< db[i].pts_prior_(0)*invpow <<"," << db[i].pts_prior_(1)*invpow <<"\n"; 
        // cout <<"["<<i<<"] start s: " << s<<endl;
        pt_k << db[i].pts_(0), db[i].pts_(1);
        pt_k *= invpow;

        float Ic_warp, dx_warp, dy_warp, Ik_single, r, w, J;
        float JtwJ_scalar = 0, Jtwr_scalar = 0;
        float delta_s = 0, r2_sum = 0;
        for(int lvl = MAX_PYR_LVL-1; lvl > -1; --lvl){
            // calculate reference image brightness
            for(int j = 0; j < M; ++j)
                *(Ik_vector + j) = improc::interpImageSingle(Ik[lvl], patch[j].x + pt_k(0),  patch[j].y + pt_k(1));
            
            // ITERATION!!!
            for(int iter = 0; iter < MAX_ITER; ++iter){
                delta_pt = s*l;

                JtwJ_scalar = 0;
                Jtwr_scalar = 0;
                r2_sum = 0;
                // Generate reference patch. (M+1) residual.
                int cnt_valid = 0;
                float u_warp , v_warp, Ik_now;
                for(int j = 0; j < M; ++j){
                    u_warp = patch[j].x + pt_near(0) + delta_pt(0);
                    v_warp = patch[j].y + pt_near(1) + delta_pt(1);
                    Ik_now = *(Ik_vector + j);

                    if((u_warp < 1) || (u_warp > Ik[lvl].cols) || (v_warp < 1) || (v_warp > Ik[lvl].rows)) // outside of image.
                        continue;
                   
                    // calculate and push edge residual, Huber weight, Jacobian, and Hessian
                    improc::interpImageSingle3(Ic[lvl], du_c[lvl], dv_c[lvl], u_warp, v_warp, interp_tmp);
                    Ic_warp = interp_tmp(0);
                    dx_warp = interp_tmp(1);
                    dy_warp = interp_tmp(2);
                    
                    if(Ic_warp < 0 || Ik_now < 0) continue;

                     // valid pixel!
                    ++cnt_valid;
                    
                    r = Ic_warp - Ik_now;
                    w = fabs(r) < thres_huber ? 1.0f : thres_huber / fabs(r);
                    J = dx_warp*l(0) + dy_warp*l(1);

                    float Jw = J*w;
                    JtwJ_scalar += Jw*J;
                    Jtwr_scalar += Jw*r;
                    r2_sum += r*r;
                }
                float sqrtscaler =  -sqrt(cnt_valid)*SCALER;
                r = sqrtscaler*(log(s) + log(-s+s_far) + 2.0f*log(0.5f*s_far));
                J = sqrtscaler*(2*s-s_far)/(s*(s-s_far));
                w = fabs(r) < thres_huber ? 1.0f : thres_huber / fabs(r);
                JtwJ_scalar += w*J*J;
                Jtwr_scalar += J*w*r;

                // Calculate step size!
                delta_s = -Jtwr_scalar/JtwJ_scalar;
                s += delta_s;
                if(s < 0.01f) s = 0.01f;
                if(s > s_far) s = s_far - 0.01f;
                if(std::isnan(delta_s)) s = 0.1f;

                if(lvl == 0) {
                    db[i].pts_tracked_ << s*l(0) + pt_near(0), s*l(1) + pt_near(1);
                    db[i].err_klt_normal_ = sqrt(r2_sum/cnt_valid);
                    db[i].s_normal_ = s;
                }
                if( abs(delta_s) < pow(10.0f, -(5-0.25f*(lvl+1)) ) ){
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
#ifdef _VERBOSE_
        cout << "[" << i <<"]: s.. [" << 0 << "   " << s << "   "  << s_far << "]\n";
#endif
    }

    delete[] Ik_vector;

};

void EpipolarKLT::runAffineBrightnessCompensation(const vector<cv::Mat>& Ik, const vector<cv::Mat>& Ic, 
        const vector<PointDB>& db, float& logalpha_res, float& beta_res)
{
    int win_sz = 10;
    int MAX_ITER = 2222;
    float thres_huber = 10;

    int M = (2 * win_sz + 1)*(2 * win_sz + 1);
    vector<cv::Point2f> patch;
    for (int u = -win_sz; u < win_sz + 1; ++u)
        for (int v = -win_sz; v < win_sz + 1; ++v)
            patch.emplace_back((float)u, (float)v);

    int n_pts = db.size();
    int n_total = M*n_pts;
    float* I1 = new float[n_total];
    float* I2 = new float[n_total];

    int cnt = 0; 
    for(int i = 0; i < n_pts; ++i){
        for(int j = 0; j < M; ++j){
            *(I1+cnt) = improc::interpImageSingle(Ik[0],
                patch[j].x + db[i].pts_(0), patch[j].y + db[i].pts_(1));
            *(I2+cnt) = improc::interpImageSingle(Ic[0],
                patch[j].x + db[i].pts_tracked_(0), patch[j].y + db[i].pts_tracked_(1));

            if( ( isnan(*(I1+cnt)) || isnan(*(I2+cnt)) ) ) continue;
            ++cnt;
        }
    }
    // A = [I2,1].....
    // ab_initial = (A'*A)^-1*A'*I1
    // A'*A = [sum I2*I2, sum I2; sum I2, cnt] --> (A'*A)^-1 = [1, -I2; -I2, I2*I2]/(I2*I2-)
    Eigen::Vector2f b(0,0); // == A'*I1 == [sum I2*I1, sum I1]'
    Eigen::Matrix2f AtA; AtA.setZero();
    float I1_tmp,I2_tmp;
    for(int i = 0; i < cnt; ++i){
        I1_tmp = *(I1+i);
        I2_tmp = *(I2+i);
        AtA(0,0) += I2_tmp*I2_tmp;
        AtA(0,1) += I2_tmp;
        b(0) += I2_tmp*I1_tmp;
        b(1) += I1_tmp;
    }
    AtA(1,0) = AtA(0,1);
    AtA(1,1) = cnt;

    Eigen::Vector2f alphabeta = AtA.inverse()*b;
#ifdef _VERBOSE_
    cout <<"ab initial: "<<alphabeta(0) << "," << alphabeta(1) << "\n";
#endif
    // iterations with Huber thres.
    Eigen::Vector2f delta_ab;
    Eigen::Matrix2f JtwJ;
    Eigen::Vector2f Jtwr;
    float w;
    for(int iter = 0; iter < MAX_ITER; ++iter){
        JtwJ << 0,0,0,0;
        Jtwr << 0,0;
        for(int i=0; i < cnt; ++i){
            I1_tmp = *(I1+i);
            I2_tmp = *(I2+i);
            float r = alphabeta(0)*I2_tmp + alphabeta(1) - I1_tmp;
            w = fabs(r) < thres_huber ? 1.0f : thres_huber / fabs(r);
            JtwJ(0,0) += I2_tmp*I2_tmp*w;
            JtwJ(0,1) += I2_tmp*w;
            JtwJ(1,1) += w;
            Jtwr(0) += w*I2_tmp*r;
            Jtwr(1) += w*r;
        }
        delta_ab = -JtwJ.inverse()*Jtwr;
        alphabeta += delta_ab;
#ifdef _VERBOSE_
        cout << "iter [" << iter <<"] / ab: [" << alphabeta(0) <<", " << alphabeta(1) <<"] \n";
#endif
        if(delta_ab.norm() < 1e-6) break;
    }

    logalpha_res = alphabeta(0);
    beta_res     = alphabeta(1);
    delete[] I1;
    delete[] I2;
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