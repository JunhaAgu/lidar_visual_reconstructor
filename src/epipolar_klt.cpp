#include "feature_tracker.h"


EpipolarKLT::EpipolarKLT(const int& n_cols, const int& n_rows, const int& MAX_PYR_LVL) 
{
    MAX_PYR_LVL_ = MAX_PYR_LVL;
    n_cols_pyr_.reserve(MAX_PYR_LVL_);
    n_rows_pyr_.reserve(MAX_PYR_LVL_);
    
    n_cols_pyr_.push_back(n_cols);
    n_rows_pyr_.push_back(n_rows);

    // for (int lvl = 2; lvl < MAX_PYR_LVL_; ++lvl).... 

    errs_ssd = (float*)custom_aligned_malloc(sizeof(float)*n_cols*n_rows);
    mask = (int*)custom_aligned_malloc(sizeof(int)*n_cols*n_rows);

    win_sz_ = 15;
    M_ = (2 * win_sz_ + 1)*(2 * win_sz_ + 1);
    patch_.reserve(M_);
    for (int u = -win_sz_; u < win_sz_ + 1; u++)
        for (int v = -win_sz_; v < win_sz_ + 1; v++)
            patch_.push_back(chk::Point2f((float)u, (float)v));


    // For SSE. Use only 80 pixels.
    upattern    = (float*)custom_aligned_malloc(sizeof(float) * 88);
    vpattern    = (float*)custom_aligned_malloc(sizeof(float) * 88);
    buf_up_ref  = (float*)custom_aligned_malloc(sizeof(float) * 88);
    buf_vp_ref  = (float*)custom_aligned_malloc(sizeof(float) * 88);
    buf_up_warp = (float*)custom_aligned_malloc(sizeof(float) * 88);
    buf_vp_warp = (float*)custom_aligned_malloc(sizeof(float) * 88);

    buf_Ik   = (float*)custom_aligned_malloc(sizeof(float) * 88);
    buf_du_k = (float*)custom_aligned_malloc(sizeof(float) * 88);
    buf_dv_k = (float*)custom_aligned_malloc(sizeof(float) * 88);

    buf_Ic_warp   = (float*)custom_aligned_malloc(sizeof(float) * 88);
    buf_du_c_warp = (float*)custom_aligned_malloc(sizeof(float) * 88);
    buf_dv_c_warp = (float*)custom_aligned_malloc(sizeof(float) * 88);

    buf_residual = (float*)custom_aligned_malloc(sizeof(float) * 88);
    buf_weight   = (float*)custom_aligned_malloc(sizeof(float) * 88);

    SSEData = (float*)custom_aligned_malloc(sizeof(float) * 4 * 28);

    errs_ssd_sse = (float*)custom_aligned_malloc(sizeof(float) * n_cols * n_rows);

    int cnt = 0;
    for (int i = -win_sz_ + 1; i < win_sz_; i += 2) {
        for (int j = -win_sz_ + 1; j < win_sz_; j += 2) {
            *(upattern + cnt) = j;
            *(vpattern + cnt) = i;
            ++cnt;
            //cout << "[" << j << "," << i << "]" << endl;
        }
    }
    for (int i = -win_sz_; i < win_sz_ + 1; i += 2) {
        for (int j = -win_sz_; j < win_sz_ + 1; j += 2) {
            *(upattern + cnt) = j;
            *(vpattern + cnt) = i;
            ++cnt;
            //cout << "[" << j << "," << i << "]" << endl;
        }
    }
    
    cout << "length of up vp : " << cnt << endl;
};

EpipolarKLT::~EpipolarKLT(){
    if (errs_ssd != nullptr) custom_aligned_free((void*)errs_ssd);
    if (errs_ncc != nullptr) custom_aligned_free((void*)errs_ncc);
    if (mask != nullptr) custom_aligned_free((void*)mask);
    if (upattern != nullptr) custom_aligned_free((void*)upattern);
    if (vpattern != nullptr) custom_aligned_free((void*)vpattern);

    if (buf_up_ref != nullptr) custom_aligned_free((void*)buf_up_ref);
    if (buf_vp_ref != nullptr) custom_aligned_free((void*)buf_vp_ref);
    if (buf_up_warp != nullptr) custom_aligned_free((void*)buf_up_warp);
    if (buf_vp_warp != nullptr) custom_aligned_free((void*)buf_vp_warp);

    if (buf_Ik != nullptr) custom_aligned_free((void*)buf_Ik);
    if (buf_du_k != nullptr) custom_aligned_free((void*)buf_du_k);
    if (buf_dv_k != nullptr) custom_aligned_free((void*)buf_dv_k);

    if (buf_Ic_warp != nullptr) custom_aligned_free((void*)buf_Ic_warp);
    if (buf_du_c_warp != nullptr) custom_aligned_free((void*)buf_du_c_warp);
    if (buf_dv_c_warp != nullptr) custom_aligned_free((void*)buf_dv_c_warp);

    if (buf_residual != nullptr) custom_aligned_free((void*)buf_residual);
    if (buf_weight != nullptr) custom_aligned_free((void*)buf_weight);

    if (SSEData != nullptr) custom_aligned_free((void*)SSEData);

    if (errs_ssd_sse != nullptr) custom_aligned_free((void*)errs_ssd_sse);
};
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