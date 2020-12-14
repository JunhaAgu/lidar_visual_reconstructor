#include "epipolar_klt.h"


EpipolarKLT::EpipolarKLT(int win_sz, bool flag_fastpattern)
: win_sz_(win_sz)
{  
    if(win_sz % 2 == 1) std::runtime_error("EKLT: a window size must be odd number!\n");
    win_half_ = (win_sz_- 1)/2; // my convention! 
    this->generatePattern(win_half_, patch_, true);
    M_ = patch_.size();
    cout <<"# of patch points : "<< M_ << "\n";

    // For a point !
    err_sse_       = (float*)custom_aligned_malloc(sizeof(float)*4000);
    mask_          = (int*)custom_aligned_malloc(sizeof(int)*4000);
    upattern_      = (float*)custom_aligned_malloc(sizeof(float)*4000);
    vpattern_      = (float*)custom_aligned_malloc(sizeof(float)*4000);
    buf_vp_ref_    = (float*)custom_aligned_malloc(sizeof(float)*4000);
    buf_up_ref_    = (float*)custom_aligned_malloc(sizeof(float)*4000);
    buf_up_warp_   = (float*)custom_aligned_malloc(sizeof(float)*4000);
    buf_vp_warp_   = (float*)custom_aligned_malloc(sizeof(float)*4000);
    buf_Ik_        = (float*)custom_aligned_malloc(sizeof(float)*4000);
    buf_Ic_warp_   = (float*)custom_aligned_malloc(sizeof(float)*4000);
    buf_du_c_warp_ = (float*)custom_aligned_malloc(sizeof(float)*4000);
    buf_dv_c_warp_ = (float*)custom_aligned_malloc(sizeof(float)*4000);
    buf_residual_  = (float*)custom_aligned_malloc(sizeof(float)*4000);
    buf_weight_    = (float*)custom_aligned_malloc(sizeof(float)*4000);
    SSEData_       = (float*)custom_aligned_malloc(sizeof(float)* 4 * 21);
};

EpipolarKLT::~EpipolarKLT(){
    if(err_sse_       != nullptr) custom_aligned_free((void*)err_sse_       );
    if(mask_          != nullptr) custom_aligned_free((void*)mask_          );
    if(upattern_      != nullptr) custom_aligned_free((void*)upattern_      );
    if(vpattern_      != nullptr) custom_aligned_free((void*)vpattern_      );
    if(buf_vp_ref_    != nullptr) custom_aligned_free((void*)buf_vp_ref_    );
    if(buf_up_ref_    != nullptr) custom_aligned_free((void*)buf_up_ref_    );
    if(buf_up_warp_   != nullptr) custom_aligned_free((void*)buf_up_warp_   );
    if(buf_vp_warp_   != nullptr) custom_aligned_free((void*)buf_vp_warp_   );
    if(buf_Ik_        != nullptr) custom_aligned_free((void*)buf_Ik_        );
    if(buf_Ic_warp_   != nullptr) custom_aligned_free((void*)buf_Ic_warp_   );
    if(buf_du_c_warp_ != nullptr) custom_aligned_free((void*)buf_du_c_warp_ );
    if(buf_dv_c_warp_ != nullptr) custom_aligned_free((void*)buf_dv_c_warp_ );
    if(buf_residual_  != nullptr) custom_aligned_free((void*)buf_residual_  );
    if(buf_weight_    != nullptr) custom_aligned_free((void*)buf_weight_    );
    if(SSEData_       != nullptr) custom_aligned_free((void*)SSEData_       );
};

void EpipolarKLT::runEpipolarKLT(
            const vector<cv::Mat>& Ik, const vector<cv::Mat>& Ic, 
            const vector<cv::Mat>& du_k, const vector<cv::Mat>& dv_k,
            const vector<cv::Mat>& du_c, const vector<cv::Mat>& dv_c,
            const int& win_sz, const int& MAX_ITER,
            const float& logalpha, const float& beta, 
            vector<PointDB>& db) {
    
#ifdef _VERBOSE_
    cout << " EKLT: registered image lvl: " << MAX_PYR_LVL <<"\n";
    cout << " EKLT: # of input points: " << n_pts_ << "\n";
#endif
    if(win_sz % 2 == 1) std::runtime_error("EKLT: a window size must be odd number!\n");

    // In this function, MAX_LVL, n_cols, n_rows are set.
    int MAX_PYR_LVL = Ik.size();
    n_pts_ = db.size();
    
    float* Ik_vector  = new float[M_];
    float* Ic_vector  = new float[M_];
    float* duc_vector = new float[M_];
    float* dvc_vector = new float[M_];

    // Do EKLT. for all points!
    float thres_huber = 10.0;
    float SCALER      = 2.0;

    float r;
    Vec2 pt_near, pt_far, l, pt_k;
    Vec2 delta_pt;
    Vec3 interp_tmp;
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

        pt_k << db[i].pts_(0), db[i].pts_(1);
        pt_k *= invpow;

        float Ic_warp, dx_warp, dy_warp, Ik_single, r, w, J;
        float JtwJ_scalar = 0, Jtwr_scalar = 0;
        float delta_s = 0, r2_sum = 0;
        for(int lvl = MAX_PYR_LVL-1; lvl > -1; --lvl){
            // calculate reference image brightness
            improc::interpImageSingleRegularPatch(Ik[lvl], pt_k(0), pt_k(1), patch_,  Ik_vector);
            // for(int j = 0; j < M; ++j)
            //     *(Ik_vector + j) = improc::interpImageSingle(Ik[lvl], patch[j].x + pt_k(0),  patch[j].y + pt_k(1));

            
            // ITERATION!!!
            for(int iter = 0; iter < MAX_ITER; ++iter){
                delta_pt = s*l;

                JtwJ_scalar = 0;
                Jtwr_scalar = 0;
                r2_sum = 0;
                // Generate reference patch. (M+1) residual.
                int cnt_valid = 0;
                float u_warp , v_warp, Ic_warp, Ik_now;

                improc::interpImageSingle3RegularPatch(Ic[lvl], du_c[lvl], dv_c[lvl],
	 	            pt_near(0) + delta_pt(0), pt_near(1) + delta_pt(1), patch_, Ic_vector, duc_vector, dvc_vector);
                // improc::interpImageSingleRegularPatch(Ic[lvl],   pt_near(0) + delta_pt(0), pt_near(1) + delta_pt(1), patch,  Ic_vector);
                // improc::interpImageSingleRegularPatch(du_c[lvl], pt_near(0) + delta_pt(0), pt_near(1) + delta_pt(1), patch,  duc_vector);
                // improc::interpImageSingleRegularPatch(dv_c[lvl], pt_near(0) + delta_pt(0), pt_near(1) + delta_pt(1), patch,  dvc_vector);

                for(int j = 0; j < M_; ++j){
                    Ik_now  = *(Ik_vector + j);
                    Ic_warp = *(Ic_vector + j);

                    if(Ic_warp < 0 || Ik_now < 0) continue;
                    dx_warp = *(duc_vector + j);
                    dy_warp = *(dvc_vector + j);
                   
                    // calculate and push edge residual, Huber weight, Jacobian, and Hessian
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
                // for(int j = 0; j < M; ++j){
                //     u_warp = patch[j].x + pt_near(0) + delta_pt(0);
                //     v_warp = patch[j].y + pt_near(1) + delta_pt(1);
                //     Ik_now = *(Ik_vector + j);

                //     if((u_warp < 1) || (u_warp > Ik[lvl].cols) || (v_warp < 1) || (v_warp > Ik[lvl].rows)) // outside of image.
                //         continue;
                   
                //     // calculate and push edge residual, Huber weight, Jacobian, and Hessian
                //     improc::interpImageSingle3(Ic[lvl], du_c[lvl], dv_c[lvl], u_warp, v_warp, interp_tmp);
                //     Ic_warp = interp_tmp(0);
                //     dx_warp = interp_tmp(1);
                //     dy_warp = interp_tmp(2);
                    
                //     if(Ic_warp < 0 || Ik_now < 0) continue;

                //      // valid pixel!
                //     ++cnt_valid;
                    
                //     r = Ic_warp - Ik_now;
                //     w = fabs(r) < thres_huber ? 1.0f : thres_huber / fabs(r);
                //     J = dx_warp*l(0) + dy_warp*l(1);

                //     float Jw = J*w;
                //     JtwJ_scalar += Jw*J;
                //     Jtwr_scalar += Jw*r;
                //     r2_sum += r*r;
                // }
                float sqrtscaler =  -sqrt(cnt_valid)*SCALER;
                r = sqrtscaler*(log(s) + log(-s+s_far) + 2.0f*log(0.5f*s_far));
                J = sqrtscaler*(2*s-s_far)/(s*(s-s_far));
                w = fabs(r) < thres_huber ? 1.0f : thres_huber / fabs(r);
                JtwJ_scalar += w*J*J;
                Jtwr_scalar += J*w*r;

                // Calculate step size!
                delta_s = -0.5*Jtwr_scalar/JtwJ_scalar;
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
//#ifdef _VERBOSE_
        cout << "[" << i <<"]: s.. [" << 0 << "   " << s << "   "  << s_far << "]\n";
//#endif
    }

    delete[] Ik_vector;
    delete[] Ic_vector;
    delete[] duc_vector;
    delete[] dvc_vector;

};

void EpipolarKLT::runAffineBrightnessCompensation(const vector<cv::Mat>& Ik, const vector<cv::Mat>& Ic, 
        const vector<PointDB>& db, float& alpha_res, float& beta_res)
{
    // different parameters from global params.
    int   win_sz      = 5;
    int   MAX_ITER    = 300;
    float thres_huber = 5;

    vector<cv::Point2f> patch;
    for (int v = -win_sz; v < win_sz + 1; ++v)
        for (int u = -win_sz; u < win_sz + 1; ++u)
            patch.emplace_back((float)u, (float)v);

    int M       = patch.size();
    int n_pts   = db.size();
    int n_total = M*n_pts;
    
    float* I1 = new float[n_total];
    float* I2 = new float[n_total];       
    float* I1_i = new float[M];
    float* I2_i = new float[M];

    int cnt = 0; 
    for(int i = 0; i < n_pts; ++i){
        improc::interpImageSingleRegularPatch(Ik[0], db[i].pts_(0), db[i].pts_(1), patch, I1_i);
        improc::interpImageSingleRegularPatch(Ic[0], db[i].pts_tracked_(0), db[i].pts_tracked_(1), patch, I2_i);
        
        for(int j = 0; j < M; ++j){
            float I1_tmp = *(I1_i+j);
            float I2_tmp = *(I2_i+j);
            *(I1+cnt) = I1_tmp;
            *(I2+cnt) = I2_tmp;
            
            if( I1_tmp < 0 || I2_tmp < 0 ||  isnan(I1_tmp) || isnan(I2_tmp) ) continue;
            ++cnt;
        }
    }
    delete[] I1_i;
    delete[] I2_i;

    // A = [I2,1].....
    // ab_initial = (A'*A)^-1*A'*I1
    // A'*A = [sum I2*I2, sum I2; sum I2, cnt] --> (A'*A)^-1 = [1, -I2; -I2, I2*I2]/(I2*I2-)
    Vec2 b(0,0); // == A'*I1 == [sum I2*I1, sum I1]'
    Mat22 AtA; AtA.setZero();
    for(int i = 0; i < cnt; ++i){
        float I1_tmp   = *(I1+i);
        float I2_tmp   = *(I2+i);
        AtA(0,0) += I2_tmp*I2_tmp;
        AtA(0,1) += I2_tmp;
        b(0)     += I2_tmp*I1_tmp;
        b(1)     += I1_tmp;
    }
    AtA(1,0) = AtA(0,1);
    AtA(1,1) = cnt;

    Vec2 alphabeta = AtA.inverse()*b;
#ifdef _VERBOSE_
    cout <<"ab initial: "<<alphabeta(0) << "," << alphabeta(1) << "\n";
#endif
    // iterations with Huber thres.
    Vec2 delta_ab;
    Eigen::Matrix2f JtwJ;
    Vec2 Jtwr;
    float w;
    for(int iter = 0; iter < MAX_ITER; ++iter){
        JtwJ << 0,0,0,0;
        Jtwr << 0,0;
        for(int i=0; i < cnt; ++i){
            float I1_tmp = *(I1+i);
            float I2_tmp = *(I2+i);
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
        if(delta_ab.norm() < 1e-4) break;
    }

    alpha_res = alphabeta(0);
    beta_res  = alphabeta(1);
    delete[] I1;
    delete[] I2;
};


void EpipolarKLT::runEpipolarAffineKLT(
            const vector<cv::Mat>& Ik, const vector<cv::Mat>& Ic, 
            const vector<cv::Mat>& du_k, const vector<cv::Mat>& dv_k,
            const vector<cv::Mat>& du_c, const vector<cv::Mat>& dv_c,
            const int& win_sz, const int& MAX_ITER,
            const float& alpha, const float& beta, 
            vector<PointDB>& db) {
#ifdef _VERBOSE_
    cout << " EKLT: registered image lvl: " << MAX_PYR_LVL <<"\n";
    cout << " EKLT: # of input points: " << n_pts_ << "\n";
#endif
    if(win_sz % 2 == 1) std::runtime_error("EKLT: a window size must be odd number!\n");

    // In this function, MAX_LVL, n_cols, n_rows are set.
    int MAX_PYR_LVL = Ik.size();
    n_pts_ = db.size();

    int win_half = (win_sz - 1)/2; // my convention! 
    
    vector<vector<cv::Point2f>> patch_lvl(MAX_PYR_LVL,vector<cv::Point2f>()); // scaled pattern for each levels.
    for(int lvl = 0; lvl < MAX_PYR_LVL; ++lvl){
        float pattern_scaler = 1.0f/pow(1.2, lvl);
        this->generatePattern(win_half, patch_lvl[lvl], true);
        for(auto itr = patch_lvl[lvl].begin(); itr != patch_lvl[lvl].end(); ++itr){
            itr->x *= pattern_scaler;
            itr->y *= pattern_scaler;
        }
    }
    
    int M = patch_lvl[0].size();
    float* Ik_vector  = new float[M];
    float* Ic_vector  = new float[M];
    float* duc_vector = new float[M];
    float* dvc_vector = new float[M];

    // Do EKLT. for all points!
    float thres_huber = 10.0;
    float SCALER      = 0.1;

    Vec2 pt_k, delta_pt;
    for(int i = 0; i < n_pts_; ++i) {
        db[i].err_klt_affine_ = -1.0; // error initialization

        // Get points.
        float invpow = 1.0 / (pow(2,MAX_PYR_LVL-1));
        Vec2 pt_near = db[i].pts_guess_near_*invpow;
        Vec2 pt_far  = db[i].pts_guess_far_ *invpow;
        Vec2 l       = pt_far - pt_near;
        float s_far  = l.norm(); l /= s_far;
        float s = (db[i].pts_tracked_*invpow - pt_near).norm(); // from prior information.

        pt_k << db[i].pts_(0), db[i].pts_(1);
        pt_k *= invpow;

        float Ic_warp, dx_warp, dy_warp, Ik_single, r, w;
        Vec5 J;
        Mat55 JtwJ; JtwJ.setZero();
        Vec5 mJtwr; mJtwr.setZero();
        Vec5 abcds; abcds.setZero();
        Vec5 delta_abcds; delta_abcds.setZero();
        abcds(4) = s;
        
        float r2_sum = 0;
        for(int lvl = MAX_PYR_LVL-1; lvl > -1; --lvl){
            // calculate reference image brightness
            for(int j = 0; j < M; ++j)
                *(Ik_vector + j) = improc::interpImageSingle(Ik[lvl], patch_lvl[lvl][j].x + pt_k(0),  patch_lvl[lvl][j].y + pt_k(1));

            // ITERATION!!!
            Vec3 vec3_tmp;
            for(int iter = 0; iter < MAX_ITER; ++iter){
                JtwJ.setZero();
                mJtwr.setZero();
                r2_sum = 0;

                float exp_a = exp(abcds(0));
                float exp_d = exp(abcds(3));
                s           = abcds(4);
                delta_pt    = s*l;

                // Generate reference patch. (M+1) residual.
                int cnt_valid = 0;
                float u_warp , v_warp, Ic_warp, Ik_now;

                for(int j = 0; j < M; ++j) {
                    u_warp = exp_a*patch_lvl[lvl][j].x + abcds(1)*patch_lvl[lvl][j].y + delta_pt(0) + pt_near(0);
                    v_warp = abcds(2)*patch_lvl[lvl][j].x + exp_d*patch_lvl[lvl][j].y + delta_pt(1) + pt_near(1);
                    Ik_now = *(Ik_vector + j);

                    if((u_warp < 1) || (u_warp > Ik[lvl].cols) || (v_warp < 1) || (v_warp > Ik[lvl].rows)) // outside of image.
                        continue;

                    improc::interpImageSingle3(Ic[lvl], du_c[lvl], dv_c[lvl], u_warp, v_warp, vec3_tmp);
                    *(Ic_vector  + j) = alpha*vec3_tmp(0) + beta;
                    *(duc_vector + j) = alpha*vec3_tmp(1);
                    *(dvc_vector + j) = alpha*vec3_tmp(2);

                    Ik_now  = *(Ik_vector + j);
                    Ic_warp = *(Ic_vector + j);

                    if(Ic_warp < 0 || Ik_now < 0) continue;
                    dx_warp = *(duc_vector + j);
                    dy_warp = *(dvc_vector + j);
                   
                    // calculate and push edge residual, Huber weight, Jacobian, and Hessian
                     // valid pixel!
                    ++cnt_valid;
                    
                    float r = Ic_warp - Ik_now;
                    w = fabs(r) < thres_huber ? 1.0f : thres_huber / fabs(r);
                    J(0) = exp_a*dx_warp * patch_lvl[lvl][j].x;
                    J(1) =       dx_warp * patch_lvl[lvl][j].y;
                    J(2) =       dy_warp * patch_lvl[lvl][j].x;
                    J(3) = exp_d*dy_warp * patch_lvl[lvl][j].y;
                    J(4) = dx_warp*l(0) + dy_warp*l(1);

                    JtwJ.noalias()  += (J*J.transpose())*w;
                    mJtwr.noalias() -= J*(r*w);
                    r2_sum += r*r;
                }
                
                float sqrtscaler = -sqrt(cnt_valid)*SCALER;
                r = sqrtscaler*(log(s) + log(-s+s_far) + 2.0f*log(0.5f*s_far));
                J(0) = 0;
                J(1) = 0;
                J(2) = 0;
                J(3) = 0;
                J(4) = sqrtscaler*(2*s-s_far)/(s*(s-s_far));
                w = fabs(r) < thres_huber ? 1.0f : thres_huber / fabs(r);
                JtwJ.noalias()  += (J*J.transpose())*w;
                mJtwr.noalias() -= J*(r*w);
                r2_sum += r*r;

                // Calculate step size!
                delta_abcds = 0.5f * (JtwJ + 0.1*Mat55::Identity()).ldlt().solve(mJtwr);
                abcds += delta_abcds;
                if(abcds(4) < 0.01f) abcds(4) = 0.01f;
                if(abcds(4) > s_far) abcds(4) = s_far - 0.01f;
                if(std::isnan(abcds(4))) abcds(4) = 0.1f;

                if(lvl == 0) {
                    db[i].pts_tracked_ << s*l(0) + pt_near(0), s*l(1) + pt_near(1);
                    db[i].err_klt_affine_ = sqrt(r2_sum/cnt_valid);
                    db[i].s_affine_ = abcds(4);
                }
                if( abs(abcds(4)) < pow(10.0f, -(5-0.25f*(lvl+1)) ) ){
                    if(lvl == 0){
                        if(db[i].err_klt_affine_ > 20.0f) db[i].klt_valid_ = false;
                    }
                    break;
                }
            }
            
            // resolution up.
            pt_k     *= 2;
            pt_near  *= 2;
            pt_far   *= 2;
            s_far    *= 2;
            abcds(4) *= 2;
        }
#ifdef _VERBOSE_
        cout << "affine [" << i <<"]: s.. [" << 0 << "   " << s << "   "  << s_far << "]\n";
#endif
    }

    delete[] Ik_vector;
    delete[] Ic_vector;
    delete[] duc_vector;
    delete[] dvc_vector;
};


void EpipolarKLT::generatePattern(
    int win_half, vector<cv::Point2f>& patch, bool flag_fastpattern = false)
{
    if(flag_fastpattern){
        bool flip = true;
        for (int v = -win_half; v < win_half + 1; ++v){
            if(flip){
                for (int u = -win_half; u < win_half + 1; u+=2)
                    patch.emplace_back((float)u, (float)v);

                flip = !flip;
            }
            else{
                for (int u = -(win_half-1); u < (win_half-1) + 1; u+=2)
                    patch.emplace_back((float)u, (float)v);
                
                flip = !flip;
            }
        }
    }
    else
    {
        for (int v = -win_half; v < win_half + 1; ++v)
            for (int u = -win_half; u < win_half + 1; ++u)
                patch.emplace_back((float)u, (float)v);
    }
}
