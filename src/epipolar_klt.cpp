#include "epipolar_klt.h"



EpipolarKLT::EpipolarKLT(int win_sz, bool flag_fastpattern)
: win_sz_(win_sz)
{  
    if(win_sz % 2 == 1) std::runtime_error("EKLT: a window size must be odd number!\n");
    if(win_sz > 61)     std::runtime_error("EKLT: windowsize must be smaller than 61!\n");
    win_half_ = (win_sz_- 1)/2; // my convention! 
    this->generatePattern(win_half_, patch_, true);
    M_ = patch_.size();
    cout <<"# of patch points : "<< M_ << "\n";

    // For a point !
    // err_sse_       = (float*)custom_aligned_malloc(sizeof(float)*4000);
    // upattern_      = (float*)custom_aligned_malloc(sizeof(float)*4000);
    // vpattern_      = (float*)custom_aligned_malloc(sizeof(float)*4000);
    // buf_v_ref_     = (float*)custom_aligned_malloc(sizeof(float)*4000);
    // buf_u_ref_     = (float*)custom_aligned_malloc(sizeof(float)*4000);
    // buf_u_warp_    = (float*)custom_aligned_malloc(sizeof(float)*4000);
    // buf_v_warp_    = (float*)custom_aligned_malloc(sizeof(float)*4000);
    // buf_Ik_        = (float*)custom_aligned_malloc(sizeof(float)*4000);
    // buf_Ic_warp_   = (float*)custom_aligned_malloc(sizeof(float)*4000);
    // buf_du_c_warp_ = (float*)custom_aligned_malloc(sizeof(float)*4000);
    // buf_dv_c_warp_ = (float*)custom_aligned_malloc(sizeof(float)*4000);
    // buf_residual_  = (float*)custom_aligned_malloc(sizeof(float)*4000);
    // buf_weight_    = (float*)custom_aligned_malloc(sizeof(float)*4000);
    // SSEData_       = (float*)custom_aligned_malloc(sizeof(float)* 4 * 21);


    // I00_           = (float*)custom_aligned_malloc(sizeof(float)*4000);
    // I01_           = (float*)custom_aligned_malloc(sizeof(float)*4000);
    // I10_           = (float*)custom_aligned_malloc(sizeof(float)*4000);
    // I11_           = (float*)custom_aligned_malloc(sizeof(float)*4000);
    // mask_Ic_       = (int*)custom_aligned_malloc(sizeof(int)*4000);

    err_sse_       = (float*)aligned_alloc(64, sizeof(float)*64*64);
    upattern_      = (float*)aligned_alloc(64, sizeof(float)*64*64);
    vpattern_      = (float*)aligned_alloc(64, sizeof(float)*64*64);
    buf_v_ref_     = (float*)aligned_alloc(64, sizeof(float)*64*64);
    buf_u_ref_     = (float*)aligned_alloc(64, sizeof(float)*64*64);
    buf_u_warp_    = (float*)aligned_alloc(64, sizeof(float)*64*64);
    buf_v_warp_    = (float*)aligned_alloc(64, sizeof(float)*64*64);
    buf_Ik_        = (float*)aligned_alloc(64, sizeof(float)*64*64);
    buf_Ic_warp_   = (float*)aligned_alloc(64, sizeof(float)*64*64);
    buf_du_c_warp_ = (float*)aligned_alloc(64, sizeof(float)*64*64);
    buf_dv_c_warp_ = (float*)aligned_alloc(64, sizeof(float)*64*64);
    buf_residual_  = (float*)aligned_alloc(64, sizeof(float)*64*64);
    buf_weight_    = (float*)aligned_alloc(64, sizeof(float)*64*64);
    SSEData_       = (float*)aligned_alloc(64, sizeof(float)*64);


    I00_           = (float*)aligned_alloc(64, sizeof(float)*64*64);
    I01_           = (float*)aligned_alloc(64, sizeof(float)*64*64);
    I10_           = (float*)aligned_alloc(64, sizeof(float)*64*64);
    I11_           = (float*)aligned_alloc(64, sizeof(float)*64*64);
    mask_Ik_       = (float*)aligned_alloc(64, sizeof(float)*64*64);  

    mask_Ic_       = (float*)aligned_alloc(64, sizeof(float)*64*64);
    mask_duc_      = (float*)aligned_alloc(64, sizeof(float)*64*64);
    mask_dvc_      = (float*)aligned_alloc(64, sizeof(float)*64*64);
};

EpipolarKLT::~EpipolarKLT(){
    // if(err_sse_       != nullptr) custom_aligned_free((void*)err_sse_       );
    // if(mask_Ic_       != nullptr) custom_aligned_free((void*)mask_Ic_       );
    // if(upattern_      != nullptr) custom_aligned_free((void*)upattern_      );
    // if(vpattern_      != nullptr) custom_aligned_free((void*)vpattern_      );
    // if(buf_v_ref_     != nullptr) custom_aligned_free((void*)buf_v_ref_    );
    // if(buf_u_ref_     != nullptr) custom_aligned_free((void*)buf_u_ref_    );
    // if(buf_u_warp_    != nullptr) custom_aligned_free((void*)buf_u_warp_   );
    // if(buf_v_warp_    != nullptr) custom_aligned_free((void*)buf_v_warp_   );
    // if(buf_Ik_        != nullptr) custom_aligned_free((void*)buf_Ik_        );
    // if(buf_Ic_warp_   != nullptr) custom_aligned_free((void*)buf_Ic_warp_   );
    // if(buf_du_c_warp_ != nullptr) custom_aligned_free((void*)buf_du_c_warp_ );
    // if(buf_dv_c_warp_ != nullptr) custom_aligned_free((void*)buf_dv_c_warp_ );
    // if(buf_residual_  != nullptr) custom_aligned_free((void*)buf_residual_  );
    // if(buf_weight_    != nullptr) custom_aligned_free((void*)buf_weight_    );
    // if(SSEData_       != nullptr) custom_aligned_free((void*)SSEData_       );


    // if(I00_           != nullptr) custom_aligned_free((void*)I00_           );
    // if(I01_           != nullptr) custom_aligned_free((void*)I01_           );
    // if(I10_           != nullptr) custom_aligned_free((void*)I10_           );
    // if(I11_           != nullptr) custom_aligned_free((void*)I11_           );
    if(err_sse_       != nullptr) free(err_sse_       );
    if(upattern_      != nullptr) free(upattern_      );
    if(vpattern_      != nullptr) free(vpattern_      );
    if(buf_v_ref_     != nullptr) free(buf_v_ref_    );
    if(buf_u_ref_     != nullptr) free(buf_u_ref_    );
    if(buf_u_warp_    != nullptr) free(buf_u_warp_   );
    if(buf_v_warp_    != nullptr) free(buf_v_warp_   );
    if(buf_Ik_        != nullptr) free(buf_Ik_        );
    if(buf_Ic_warp_   != nullptr) free(buf_Ic_warp_   );
    if(buf_du_c_warp_ != nullptr) free(buf_du_c_warp_ );
    if(buf_dv_c_warp_ != nullptr) free(buf_dv_c_warp_ );
    if(buf_residual_  != nullptr) free(buf_residual_  );
    if(buf_weight_    != nullptr) free(buf_weight_    );
    if(SSEData_       != nullptr) free(SSEData_       );


    if(I00_           != nullptr) free(I00_           );
    if(I01_           != nullptr) free(I01_           );
    if(I10_           != nullptr) free(I10_           );
    if(I11_           != nullptr) free(I11_           );
    if(mask_Ik_       != nullptr) free(mask_Ik_       );

    if(mask_Ic_       != nullptr) free(mask_Ic_       );
    if(mask_duc_      != nullptr) free(mask_duc_      );
    if(mask_dvc_      != nullptr) free(mask_dvc_      );
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
    

    int win_half = (win_sz - 1)/2; // my convention! 
    vector<cv::Point2f> patch; 
    this->generatePattern(win_half, patch, true);
    int M = patch.size();

    float* Ik_vector  = new float[M];
    float* Ic_vector  = new float[M];
    float* duc_vector = new float[M];
    float* dvc_vector = new float[M];

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
            improc::interpImageSingleRegularPatch(Ik[lvl], pt_k(0), pt_k(1), patch,  Ik_vector);
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
	 	            pt_near(0) + delta_pt(0), pt_near(1) + delta_pt(1), patch, Ic_vector, duc_vector, dvc_vector);
                // improc::interpImageSingleRegularPatch(Ic[lvl],   pt_near(0) + delta_pt(0), pt_near(1) + delta_pt(1), patch,  Ic_vector);
                // improc::interpImageSingleRegularPatch(du_c[lvl], pt_near(0) + delta_pt(0), pt_near(1) + delta_pt(1), patch,  duc_vector);
                // improc::interpImageSingleRegularPatch(dv_c[lvl], pt_near(0) + delta_pt(0), pt_near(1) + delta_pt(1), patch,  dvc_vector);

                for(int j = 0; j < M; ++j){
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
        cout << "[" << i <<"]: s.. [" << 0 << "   " << db[i].s_normal_ << "   "  << s_far << "]\n";
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

    vector<cv::Point2f> pts_pattern;
    for(int i = 0; i < M; ++i) pts_pattern.emplace_back(0,0);

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
            improc::interpImageSingleRegularPatch(Ik[lvl], pt_k(0), pt_k(1), patch_lvl[lvl], Ik_vector);
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

                for(int j = 0; j < M; ++j){
                    pts_pattern[j].x = exp_a*patch_lvl[lvl][j].x + abcds(1)*patch_lvl[lvl][j].y + delta_pt(0) + pt_near(0);
                    pts_pattern[j].y = abcds(2)*patch_lvl[lvl][j].x + exp_d*patch_lvl[lvl][j].y + delta_pt(1) + pt_near(1);
                }
                improc::interpImageSingle3ArbitraryPatch(Ic[lvl],du_c[lvl],dv_c[lvl],pts_pattern,Ic_vector,duc_vector,dvc_vector);

                for(int j = 0; j < M; ++j) {
                    // u_warp = exp_a*patch_lvl[lvl][j].x + abcds(1)*patch_lvl[lvl][j].y + delta_pt(0) + pt_near(0);
                    // v_warp = abcds(2)*patch_lvl[lvl][j].x + exp_d*patch_lvl[lvl][j].y + delta_pt(1) + pt_near(1);

                    // if((u_warp < 1) || (u_warp > Ik[lvl].cols) || (v_warp < 1) || (v_warp > Ik[lvl].rows)) // outside of image.
                    //     continue;

                    // improc::interpImageSingle3(Ic[lvl], du_c[lvl], dv_c[lvl], u_warp, v_warp, vec3_tmp);
                    //*(Ic_vector  + j) = alpha*vec3_tmp(0) + beta;
                    //*(duc_vector + j) = alpha*vec3_tmp(1);
                    //*(dvc_vector + j) = alpha*vec3_tmp(2);

                    Ik_now  = *(Ik_vector + j);
                    Ic_warp = *(Ic_vector + j);

                    if(Ic_warp < 0 || Ik_now < 0) continue;
                    Ic_warp = alpha*Ic_warp + beta;
                    dx_warp = alpha*(*(duc_vector + j));
                    dy_warp = alpha*(*(dvc_vector + j));
                   
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
                r = -SCALER*(log(s) + log(-s+s_far) + 2.0f*log(0.5f*s_far));
                J(0) = 0;
                J(1) = 0;
                J(2) = 0;
                J(3) = 0;
                J(4) = -SCALER*(2*s-s_far)/(s*(s-s_far));
                w = fabs(r) < thres_huber ? 1.0f : thres_huber / fabs(r);
                JtwJ.noalias()  += (J*J.transpose())*w;
                mJtwr.noalias() -= J*(r*w);
                r2_sum += r*r;

                // Calculate step size!
                JtwJ(0,0) += 0.1;
                JtwJ(1,1) += 0.1;
                JtwJ(2,2) += 0.1;
                JtwJ(3,3) += 0.1;
                JtwJ(4,4) += 0.1;
                delta_abcds = 0.5f * JtwJ.ldlt().solve(mJtwr);
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
//#ifdef _VERBOSE_
        cout << "affine [" << i <<"]: s.. [" << 0 << "   " << db[i].s_affine_ << "   "  << s_far << "]\n";
//#endif
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


// SSE version codes
/* ==========================================================================
=============================================================================
========================================================================== */
void EpipolarKLT::sse_setPatchPattern(const vector<cv::Point2f>& patch){
    this->n_patch_       = patch.size();
    this->n_ssesteps_    = (int)floor(n_patch_ / 4);
    this->n_patch_trunc_ = 4*n_ssesteps_;    
    for(int i = 0; i < this->n_patch_trunc_ ; ++i) {
        *(upattern_ + i) = patch[i].x;
        *(vpattern_ + i) = patch[i].y;
    }
};

void EpipolarKLT::sse_generateReferencePoints(const float& u, const float& v){
    __m128 uuuu = _mm_set1_ps(u);
    __m128 vvvv = _mm_set1_ps(v);

    __m128 upat, vpat;
    
    for(int idx = 0; idx < this->n_patch_trunc_; idx += 4){
         // reference patch.
        upat = _mm_load_ps(upattern_ + idx);
        vpat = _mm_load_ps(vpattern_ + idx);
        _mm_store_ps(buf_u_ref_ + idx, _mm_add_ps(upat, uuuu));
        _mm_store_ps(buf_v_ref_ + idx, _mm_add_ps(vpat, vvvv));
    }
};

void EpipolarKLT::sse_warpAffine(const Vec5& params, const float& lx, const float& ly, const float& u, const float& v){
    __m128 expa4 = _mm_set1_ps(exp(params(0))); // exp(a)
    __m128 bbbb  = _mm_set1_ps(params(1));  // b
    __m128 cccc  = _mm_set1_ps(params(2));  // c 
    __m128 expd4 = _mm_set1_ps(exp(params(3))); // exp(d)
    
    __m128 uplusu4 = _mm_set1_ps(params(4)*lx + u);
    __m128 vplusv4 = _mm_set1_ps(params(4)*ly + v);
    
    __m128 upat, vpat;
    
    for(int idx = 0; idx < this->n_patch_trunc_; idx += 4){
        upat = _mm_load_ps(upattern_ + idx);
        vpat = _mm_load_ps(vpattern_ + idx);

        _mm_store_ps(buf_u_warp_ + idx,
            _mm_add_ps(
                    _mm_add_ps(_mm_mul_ps(upat,expa4),_mm_mul_ps(vpat,bbbb)),uplusu4));
       
        _mm_store_ps(buf_v_warp_ + idx,
            _mm_add_ps(
                    _mm_add_ps(_mm_mul_ps(upat,cccc),_mm_mul_ps(vpat,expd4)),vplusv4));    
    }
};

void EpipolarKLT::sse_interpReferenceImage(const cv::Mat& Ik){
    for(int i = 0; i < this->n_patch_; ++i){
        *(buf_Ik_ + i) = improc::interpImageSingle(Ik, *(buf_u_ref_ + i), *(buf_v_ref_ + i));
    }
};

void EpipolarKLT::sse_interpImage_simd(const cv::Mat& img, float* buf_u, float* buf_v,  float* res_img, float* mask){
    float* img_ptr = (float*)img.ptr<float>(0);    

    int n_cols = img.cols;
    int n_rows = img.rows;

    __m128 axaxaxax, ayayayay, uuuu, vvvv;
    __m128 u0u0u0u0, v0v0v0v0, I00, I01, I10, I11, I00mI10;
    __m128 cols4 = _mm_set1_ps((float)n_cols);
    __m128 rows4 = _mm_set1_ps((float)n_rows);
    __m128 v0cols4, v0colsu04;

    __m128 zeros   = _mm_setzero_ps();
    __m128 isvalid = _mm_set1_ps(0);

    float* idx4     = (float*)aligned_alloc(16, 4*4);

    for(int idx = 0; idx < this->n_patch_trunc_; idx += 4) {
        // I00, I01, I10, I11, ax, ay
        uuuu = _mm_load_ps(buf_u + idx);
        vvvv = _mm_load_ps(buf_v + idx);

        // inimage test. true : 0xffffffff, false: 0x00000000
        isvalid = _mm_and_ps(_mm_and_ps(_mm_cmpgt_ps(uuuu,zeros),_mm_cmplt_ps(uuuu,cols4)),
                             _mm_and_ps(_mm_cmpgt_ps(vvvv,zeros),_mm_cmplt_ps(vvvv,rows4)));

        // floor u and v
        u0u0u0u0 = _mm_floor_ps(uuuu);
        v0v0v0v0 = _mm_floor_ps(vvvv);

        // make invalid point into (0,0).
        u0u0u0u0 = _mm_and_ps(u0u0u0u0, isvalid);
        v0v0v0v0 = _mm_and_ps(v0v0v0v0, isvalid);

        // calculate ax ay
        axaxaxax = _mm_sub_ps(uuuu, u0u0u0u0);
        ayayayay = _mm_sub_ps(vvvv, v0v0v0v0);

        v0cols4   = _mm_mul_ps(cols4,   v0v0v0v0);
        v0colsu04 = _mm_add_ps(v0cols4, u0u0u0u0);

        _mm_store_ps(idx4, v0colsu04);

        int i0,i1,i2,i3;
        i0 = idx; 
        i1 = idx + 1;
        i2 = idx + 2;
        i3 = idx + 3;        
        *(I00_+ i0) = *(img_ptr + (int)( *idx4               ));   
        *(I00_+ i1) = *(img_ptr + (int)(*(idx4 + 1)          ));
        *(I00_+ i2) = *(img_ptr + (int)(*(idx4 + 2)          ));
        *(I00_+ i3) = *(img_ptr + (int)(*(idx4 + 3)          ));
        
        *(I10_+ i0) = *(img_ptr + (int)( *idx4       + 1     ));  
        *(I10_+ i1) = *(img_ptr + (int)(*(idx4 + 1)  + 1     ));
        *(I10_+ i2) = *(img_ptr + (int)(*(idx4 + 2)  + 1     ));
        *(I10_+ i3) = *(img_ptr + (int)(*(idx4 + 3)  + 1     ));

        *(I01_+ i0) = *(img_ptr + (int)( *idx4       + n_cols));  
        *(I01_+ i1) = *(img_ptr + (int)(*(idx4 + 1)  + n_cols));  
        *(I01_+ i2) = *(img_ptr + (int)(*(idx4 + 2)  + n_cols));  
        *(I01_+ i3) = *(img_ptr + (int)(*(idx4 + 3)  + n_cols));  

        *(I11_+ i0) = *(img_ptr + (int)( *idx4       + n_cols + 1));  
        *(I11_+ i1) = *(img_ptr + (int)(*(idx4 + 1)  + n_cols + 1));
        *(I11_+ i2) = *(img_ptr + (int)(*(idx4 + 2)  + n_cols + 1));
        *(I11_+ i3) = *(img_ptr + (int)(*(idx4 + 3)  + n_cols + 1));

        // bilinear interpolation
        I00 = _mm_load_ps(I00_ + idx);
        I01 = _mm_load_ps(I01_ + idx);
        I10 = _mm_load_ps(I10_ + idx);
        I11 = _mm_load_ps(I11_ + idx);
        I00mI10 = _mm_sub_ps(I00, I10);
        
        _mm_store_ps(res_img + idx,
            _mm_add_ps(
                _mm_mul_ps(axaxaxax,
                    _mm_sub_ps(
                        _mm_mul_ps(
                            ayayayay,
                            _mm_sub_ps(
                                _mm_add_ps(
                                    I11,
                                    I00mI10),
                                I01)), 
                        I00mI10)),
                _mm_add_ps(
                    _mm_mul_ps(
                        ayayayay, 
                        _mm_sub_ps(
                            I01,
                            I00)),
                    I00))
        );
        _mm_store_ps(mask + idx, isvalid);
    }

    for(int i = 0; i < this->n_patch_trunc_; ++i)
        *(res_img + i) = (*(mask + i) ? *(res_img  + i) : -1.0f);

    // for(int i = 0; i < this->n_patch_trunc_; ++i){
    //     cout << *(res_img + i) <<"\n";
    // }

    free(idx4);
};

void EpipolarKLT::sse_calcResidualAndWeight(const cv::Mat& Ic, const cv::Mat& du_c, const cv::Mat& dv_c,
    const float& alpha, const float& beta, const float& thres_huber){
    Vec3 interp;
    for(int i = 0; i < this->n_patch_trunc_; ++i){
        improc::interpImageSingle3(Ic, du_c, dv_c, *(buf_u_warp_ + i), *(buf_v_warp_ + i), interp);
        *(buf_Ic_warp_ + i)   = interp(0);
        *(buf_du_c_warp_ + i) = interp(1);
        *(buf_dv_c_warp_ + i) = interp(2);
    }
    __m128 Ik4, Ic4;
    for(int idx = 0; idx < this->n_patch_trunc_; idx += 4){
        Ik4 = _mm_load_ps(buf_Ik_ + idx);
        Ic4 = _mm_load_ps(buf_Ic_warp_ + idx);
        _mm_store_ps(buf_residual_ + idx, _mm_sub_ps(Ic4, Ik4) );
    }
    for(int i = 0; i < this->n_patch_trunc_; ++i){
        if(*(buf_Ik_ + i) < 0 || *(buf_Ic_warp_ + i) < 0){
            *(buf_weight_ + i) = 0.0f;
            continue;
        }
        *(buf_Ic_warp_ + i)   *= alpha;
        *(buf_Ic_warp_ + i)   += beta;
        *(buf_du_c_warp_ + i) *= alpha;
        *(buf_dv_c_warp_ + i) *= alpha;
        float r = *(buf_residual_ + i);
        *(buf_weight_ + i) = abs(r) < thres_huber ? 1.0f : thres_huber/abs(r);
    }
};

void EpipolarKLT::sse_calcResidualAndWeight_simd(const cv::Mat& Ic, const cv::Mat& du_c, const cv::Mat& dv_c,
    const float& alpha, const float& beta, const float& thres_huber)
{
    // calculate warped brightness, du, dv.
    this->sse_interpImage_simd(Ic,   buf_u_warp_, buf_v_warp_, buf_Ic_warp_,   mask_Ic_ );
    this->sse_interpImage_simd(du_c, buf_u_warp_, buf_v_warp_, buf_du_c_warp_, mask_duc_);
    this->sse_interpImage_simd(dv_c, buf_u_warp_, buf_v_warp_, buf_dv_c_warp_, mask_dvc_);
    for(int i = 0; i < this->n_patch_trunc_; ++i) {
        if(*(buf_Ic_warp_ + i) < 0 || *(buf_Ik_ + i) < 0){
            *(buf_weight_ + i) = 0.0f;
            continue;
        }
        *(buf_Ic_warp_ + i)   *= alpha;
        *(buf_Ic_warp_ + i)   += beta;
        *(buf_du_c_warp_ + i) *= alpha;
        *(buf_dv_c_warp_ + i) *= alpha; 
    }
    for(int idx = 0; idx < this->n_patch_trunc_; idx += 4){
        _mm_store_ps(buf_residual_ + idx, 
                _mm_sub_ps(_mm_load_ps(buf_Ic_warp_ + idx), _mm_load_ps(buf_Ik_ + idx)) );
    }
    for(int i = 0; i < this->n_patch_trunc_; ++i){
        float r = *(buf_residual_ + i);
        *(buf_weight_ + i) = abs(r) < thres_huber ? 1.0f : thres_huber/abs(r);
    }

};

void EpipolarKLT::sse_calcHessianAndJacobian(const Vec5& params, const float& lx, const float& ly, float& err_sse){
    __m128 J1, J2, J3, J4, J5; // four data at once
    __m128 du_warp4, dv_warp4, upattern4, vpattern4;
    __m128 expa4 = _mm_set1_ps(exp(params(0)));
    __m128 expd4 = _mm_set1_ps(exp(params(3)));
    __m128 lxlxlxlx = _mm_set1_ps(lx);
    __m128 lylylyly = _mm_set1_ps(ly);

    for(int idx = 0; idx < this->n_patch_trunc_; idx += 4) {
        du_warp4  = _mm_load_ps(buf_du_c_warp_ + idx);
        dv_warp4  = _mm_load_ps(buf_dv_c_warp_ + idx);
        upattern4 = _mm_load_ps(upattern_ + idx);
        vpattern4 = _mm_load_ps(vpattern_ + idx);

        //J(0) = exp_a*dx_warp * patch_lvl[lvl][j].x;
        J1 = _mm_mul_ps(_mm_mul_ps(du_warp4, expa4), upattern4); 
        //J(1) =       dx_warp * patch_lvl[lvl][j].y;
        J2 = _mm_mul_ps(du_warp4, vpattern4);
        //J(2) =       dy_warp * patch_lvl[lvl][j].x;
        J3 = _mm_mul_ps(dv_warp4, upattern4);
        //J(3) = exp_d*dy_warp * patch_lvl[lvl][j].y;
        J4 = _mm_mul_ps(_mm_mul_ps(dv_warp4, expd4), vpattern4);
        //J(4) = dx_warp*l(0) + dy_warp*l(1);
        J5 = _mm_add_ps( _mm_mul_ps(du_warp4, lxlxlxlx) ,_mm_mul_ps(dv_warp4, lylylyly) );        
    
        this->sse_update(J1, J2, J3, J4, J5, _mm_load_ps(buf_residual_ + idx), _mm_load_ps(buf_weight_ + idx), err_sse);
    }
};

void EpipolarKLT::sse_update(const __m128& J1, const __m128& J2, const __m128& J3, const __m128& J4, const __m128& J5,
    const __m128& residual, const __m128& weight, float& err_sse){
    
    // A.noalias() += J* J.transpose() * weight;
    memset(SSEData_, 0, sizeof(float) * 4 * 64);
    float* ptr = SSEData_;

    __m128 J1w = _mm_mul_ps(J1, weight);
    _mm_store_ps(ptr, _mm_add_ps(_mm_load_ps(ptr), _mm_mul_ps(J1w, J1))); ptr += 4;
    _mm_store_ps(ptr, _mm_add_ps(_mm_load_ps(ptr), _mm_mul_ps(J1w, J2))); ptr += 4;
    _mm_store_ps(ptr, _mm_add_ps(_mm_load_ps(ptr), _mm_mul_ps(J1w, J3))); ptr += 4;
    _mm_store_ps(ptr, _mm_add_ps(_mm_load_ps(ptr), _mm_mul_ps(J1w, J4))); ptr += 4;
    _mm_store_ps(ptr, _mm_add_ps(_mm_load_ps(ptr), _mm_mul_ps(J1w, J5))); ptr += 4;
    
    __m128 J2w = _mm_mul_ps(J2, weight);
    _mm_store_ps(ptr, _mm_add_ps(_mm_load_ps(ptr), _mm_mul_ps(J2w, J2))); ptr += 4;
    _mm_store_ps(ptr, _mm_add_ps(_mm_load_ps(ptr), _mm_mul_ps(J2w, J3))); ptr += 4;
    _mm_store_ps(ptr, _mm_add_ps(_mm_load_ps(ptr), _mm_mul_ps(J2w, J4))); ptr += 4;
    _mm_store_ps(ptr, _mm_add_ps(_mm_load_ps(ptr), _mm_mul_ps(J2w, J5))); ptr += 4;
   
    __m128 J3w = _mm_mul_ps(J3, weight);
    _mm_store_ps(ptr, _mm_add_ps(_mm_load_ps(ptr), _mm_mul_ps(J3w, J3))); ptr += 4;
    _mm_store_ps(ptr, _mm_add_ps(_mm_load_ps(ptr), _mm_mul_ps(J3w, J4))); ptr += 4;
    _mm_store_ps(ptr, _mm_add_ps(_mm_load_ps(ptr), _mm_mul_ps(J3w, J5))); ptr += 4;

    __m128 J4w = _mm_mul_ps(J4, weight);
    _mm_store_ps(ptr, _mm_add_ps(_mm_load_ps(ptr), _mm_mul_ps(J4w, J4))); ptr += 4;
    _mm_store_ps(ptr, _mm_add_ps(_mm_load_ps(ptr), _mm_mul_ps(J4w, J5))); ptr += 4;

    __m128 J5w = _mm_mul_ps(J5, weight);
    _mm_store_ps(ptr, _mm_add_ps(_mm_load_ps(ptr), _mm_mul_ps(J5w, J5))); ptr += 4;
   
    
    // b.noalias() -= J* (residual * weight);
    __m128 residualW = _mm_mul_ps(residual, weight);
    _mm_store_ps(ptr, _mm_add_ps(_mm_load_ps(ptr), _mm_mul_ps(residualW, J1))); ptr += 4;
    _mm_store_ps(ptr, _mm_add_ps(_mm_load_ps(ptr), _mm_mul_ps(residualW, J2))); ptr += 4;
    _mm_store_ps(ptr, _mm_add_ps(_mm_load_ps(ptr), _mm_mul_ps(residualW, J3))); ptr += 4;
    _mm_store_ps(ptr, _mm_add_ps(_mm_load_ps(ptr), _mm_mul_ps(residualW, J4))); ptr += 4;
    _mm_store_ps(ptr, _mm_add_ps(_mm_load_ps(ptr), _mm_mul_ps(residualW, J5))); ptr += 4;

    // error += res* res * weight
    _mm_store_ps(ptr, _mm_add_ps(_mm_load_ps(ptr), _mm_mul_ps(residualW, residual)));

    // update JtWJ_sse_,
    ptr = SSEData_;
    JtWJ_sse_(0, 0)                    += (*(ptr)      + *(++ptr) + *(++ptr) + *(++ptr));
    JtWJ_sse_(1, 0) = (JtWJ_sse_(0, 1) += (*(++ptr) + *(++ptr) + *(++ptr) + *(++ptr)) );
    JtWJ_sse_(2, 0) = (JtWJ_sse_(0, 2) += (*(++ptr) + *(++ptr) + *(++ptr) + *(++ptr)) );
    JtWJ_sse_(3, 0) = (JtWJ_sse_(0, 3) += (*(++ptr) + *(++ptr) + *(++ptr) + *(++ptr)) );
    JtWJ_sse_(4, 0) = (JtWJ_sse_(0, 4) += (*(++ptr) + *(++ptr) + *(++ptr) + *(++ptr)) );

    JtWJ_sse_(1, 1)                    += (*(++ptr) + *(++ptr) + *(++ptr) + *(++ptr));
    JtWJ_sse_(2, 1) = (JtWJ_sse_(1, 2) += (*(++ptr) + *(++ptr) + *(++ptr) + *(++ptr)) );
    JtWJ_sse_(3, 1) = (JtWJ_sse_(1, 3) += (*(++ptr) + *(++ptr) + *(++ptr) + *(++ptr)) );
    JtWJ_sse_(4, 1) = (JtWJ_sse_(1, 4) += (*(++ptr) + *(++ptr) + *(++ptr) + *(++ptr)) );

    JtWJ_sse_(2, 2)                    += (*(++ptr) + *(++ptr) + *(++ptr) + *(++ptr));
    JtWJ_sse_(3, 2) = (JtWJ_sse_(2, 3) += (*(++ptr) + *(++ptr) + *(++ptr) + *(++ptr)) );
    JtWJ_sse_(4, 2) = (JtWJ_sse_(2, 4) += (*(++ptr) + *(++ptr) + *(++ptr) + *(++ptr)) );

    JtWJ_sse_(3, 3)                    += (*(++ptr) + *(++ptr) + *(++ptr) + *(++ptr));
    JtWJ_sse_(4, 3) = (JtWJ_sse_(3, 4) += (*(++ptr) + *(++ptr) + *(++ptr) + *(++ptr)) );

    JtWJ_sse_(4, 4)                    += (*(++ptr) + *(++ptr) + *(++ptr) + *(++ptr));

//#define GET4(ptr_,i,j) (*(ptr_ + 4 * i + j))
    // JtWJ_sse_(0, 0)                    += (GET4(SSEData_, 0, 0) + GET4(SSEData_, 0, 1) + GET4(SSEData_, 0, 2) + GET4(SSEData_, 0, 3));
    // JtWJ_sse_(1, 0) = (JtWJ_sse_(0, 1) += (GET4(SSEData_, 1, 0) + GET4(SSEData_, 1, 1) + GET4(SSEData_, 1, 2) + GET4(SSEData_, 1, 3)));
    // JtWJ_sse_(2, 0) = (JtWJ_sse_(0, 2) += (GET4(SSEData_, 2, 0) + GET4(SSEData_, 2, 1) + GET4(SSEData_, 2, 2) + GET4(SSEData_, 2, 3)));
    // JtWJ_sse_(3, 0) = (JtWJ_sse_(0, 3) += (GET4(SSEData_, 3, 0) + GET4(SSEData_, 3, 1) + GET4(SSEData_, 3, 2) + GET4(SSEData_, 3, 3)));
    // JtWJ_sse_(4, 0) = (JtWJ_sse_(0, 4) += (GET4(SSEData_, 4, 0) + GET4(SSEData_, 4, 1) + GET4(SSEData_, 4, 2) + GET4(SSEData_, 4, 3)));

    // JtWJ_sse_(1, 1)                    += (GET4(SSEData_, 5, 0) + GET4(SSEData_, 5, 1) + GET4(SSEData_, 5, 2) + GET4(SSEData_, 5, 3));
    // JtWJ_sse_(2, 1) = (JtWJ_sse_(1, 2) += (GET4(SSEData_, 6, 0) + GET4(SSEData_, 6, 1) + GET4(SSEData_, 6, 2) + GET4(SSEData_, 6, 3)));
    // JtWJ_sse_(3, 1) = (JtWJ_sse_(1, 3) += (GET4(SSEData_, 7, 0) + GET4(SSEData_, 7, 1) + GET4(SSEData_, 7, 2) + GET4(SSEData_, 7, 3)));
    // JtWJ_sse_(4, 1) = (JtWJ_sse_(1, 4) += (GET4(SSEData_, 8, 0) + GET4(SSEData_, 8, 1) + GET4(SSEData_, 8, 2) + GET4(SSEData_, 8, 3)));

    // JtWJ_sse_(2, 2)                    += (GET4(SSEData_, 9, 0) + GET4(SSEData_, 9, 1) + GET4(SSEData_, 9, 2) + GET4(SSEData_, 9, 3));
    // JtWJ_sse_(3, 2) = (JtWJ_sse_(2, 3) += (GET4(SSEData_,10, 0) + GET4(SSEData_,10, 1) + GET4(SSEData_,10, 2) + GET4(SSEData_,10, 3)));
    // JtWJ_sse_(4, 2) = (JtWJ_sse_(2, 4) += (GET4(SSEData_,11, 0) + GET4(SSEData_,11, 1) + GET4(SSEData_,11, 2) + GET4(SSEData_,11, 3)));

    // JtWJ_sse_(3, 3)                    += (GET4(SSEData_,12, 0) + GET4(SSEData_,12, 1) + GET4(SSEData_,12, 2) + GET4(SSEData_,12, 3));
    // JtWJ_sse_(4, 3) = (JtWJ_sse_(3, 4) += (GET4(SSEData_,13, 0) + GET4(SSEData_,13, 1) + GET4(SSEData_,13, 2) + GET4(SSEData_,13, 3)));

    // JtWJ_sse_(4, 4)                    += (GET4(SSEData_,14, 0) + GET4(SSEData_,14, 1) + GET4(SSEData_,14, 2) + GET4(SSEData_,14, 3));

    // update mJtWr_sse
    mJtWr_sse_(0) -= (*(++ptr) + *(++ptr) + *(++ptr) + *(++ptr));
    mJtWr_sse_(1) -= (*(++ptr) + *(++ptr) + *(++ptr) + *(++ptr));
    mJtWr_sse_(2) -= (*(++ptr) + *(++ptr) + *(++ptr) + *(++ptr));
    mJtWr_sse_(3) -= (*(++ptr) + *(++ptr) + *(++ptr) + *(++ptr));
    mJtWr_sse_(4) -= (*(++ptr) + *(++ptr) + *(++ptr) + *(++ptr));

    // mJtWr_sse_(0) -= (GET4(SSEData_,15, 0) + GET4(SSEData_,15, 1) + GET4(SSEData_,15, 2) + GET4(SSEData_,15, 3));
    // mJtWr_sse_(1) -= (GET4(SSEData_,16, 0) + GET4(SSEData_,16, 1) + GET4(SSEData_,16, 2) + GET4(SSEData_,16, 3));
    // mJtWr_sse_(2) -= (GET4(SSEData_,17, 0) + GET4(SSEData_,17, 1) + GET4(SSEData_,17, 2) + GET4(SSEData_,17, 3));
    // mJtWr_sse_(3) -= (GET4(SSEData_,18, 0) + GET4(SSEData_,18, 1) + GET4(SSEData_,18, 2) + GET4(SSEData_,18, 3));
    // mJtWr_sse_(4) -= (GET4(SSEData_,19, 0) + GET4(SSEData_,19, 1) + GET4(SSEData_,19, 2) + GET4(SSEData_,19, 3));

    // update err.
    err_sse += (*(++ptr) + *(++ptr) + *(++ptr) + *(++ptr));
    // err_sse += (GET4(SSEData_,20, 0) + GET4(SSEData_,20, 1) + GET4(SSEData_,20, 2) + GET4(SSEData_,20, 3));
};

void EpipolarKLT::sse_solveGaussNewtonStep(Vec5& delta_params){
    for(int i = 0; i < 5; ++i) JtWJ_sse_(i,i) += 0.1; // weight!
    delta_params = JtWJ_sse_.ldlt().solve(mJtWr_sse_);
    delta_params *= 0.5; // downstep.
};



void EpipolarKLT::runEpipolarAffineKLT_SSE(
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
    n_pts_          = db.size();

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

    // Do EKLT. for all points!
    float thres_huber = 10.0;
    float SCALER      = 0.1;

    Vec2 pt_k;
    Vec5 abcds;
    Vec5 J, delta_abcds;
    for(int i = 0; i < n_pts_; ++i) {
        db[i].err_klt_affine_ = 0.0; // error initialization

        // Get points.
        float invpow = 1.0 / (pow(2, MAX_PYR_LVL-1));
        Vec2 pt_near = db[i].pts_guess_near_*invpow;
        Vec2 pt_far  = db[i].pts_guess_far_ *invpow;
        Vec2 l       = pt_far - pt_near;
        float s_far  = l.norm(); l /= s_far;
        float s = (db[i].pts_tracked_*invpow - pt_near).norm(); // from prior information.

        pt_k << db[i].pts_(0), db[i].pts_(1);
        pt_k *= invpow;

        // initialize params.
        abcds.setZero();
        abcds(4) = s;
        for(int lvl = MAX_PYR_LVL - 1; lvl > -1; --lvl) {
            this->sse_setPatchPattern(patch_lvl[lvl]);
            this->sse_generateReferencePoints(pt_k(0), pt_k(1));
            //this->sse_interpReferenceImage(Ik[lvl]);
            this->sse_interpImage_simd(Ik[lvl], buf_u_ref_, buf_v_ref_, buf_Ik_, mask_Ik_);

            // ITERATION!!!
            for(int iter = 0; iter < MAX_ITER; ++iter){
                JtWJ_sse_.setZero();
                mJtWr_sse_.setZero();

                this->sse_warpAffine(abcds, l(0), l(1), pt_near(0), pt_near(1));
                //this->sse_calcResidualAndWeight(Ic[lvl], du_c[lvl], dv_c[lvl], alpha, beta, thres_huber);
                this->sse_calcResidualAndWeight_simd(Ic[lvl], du_c[lvl], dv_c[lvl], alpha, beta, thres_huber);
                this->sse_calcHessianAndJacobian(abcds, l(0), l(1), db[i].err_klt_affine_);
                
                float r = -SCALER*(log(s) + log(-s+s_far) + 2.0f*log(0.5f*s_far));
                J(0) = 0;
                J(1) = 0;
                J(2) = 0;
                J(3) = 0;
                J(4) = -SCALER*(2*s-s_far)/(s*(s-s_far));
                float w = fabs(r) < thres_huber ? 1.0f : thres_huber / fabs(r);
                JtWJ_sse_.noalias()  += (J*J.transpose())*w;
                mJtWr_sse_.noalias() -= J*(r*w);
                db[i].err_klt_affine_ += r*r;


                this->sse_solveGaussNewtonStep(delta_abcds);
                //cout << "JtWJ_sse_:\n" << JtWJ_sse_ << "\n";
                abcds += delta_abcds;
                if(abcds(4) < 0.01f) abcds(4) = 0.01f;
                if(abcds(4) > s_far) abcds(4) = s_far - 0.01f;
                if(std::isnan(abcds(4))) abcds(4) = 0.1f;

                if(lvl == 0) {
                    db[i].pts_tracked_ << s*l(0) + pt_near(0), s*l(1) + pt_near(1);
                    db[i].err_klt_affine_ = sqrt(db[i].err_klt_affine_/M);
                    db[i].s_affine_ = abcds(4);
                    db[i].abcd_ << abcds(0), abcds(1), abcds(2), abcds(3);
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
        cout << "(sse) affine [" << i <<"]: s.. [" << db[i].s_normal_ << "   " << db[i].s_affine_ << "   "  << s_far << "]\n";
#endif
    }

};