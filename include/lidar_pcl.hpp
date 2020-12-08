#ifndef _LIDARPCL_H_
#define _LIDARPCL_H_

#define PI 3.141592653589793238
#define D2R PI/180.0
#define R2D 180.0/PI

#include <iostream>
#include <vector>
#include "custom_memory.hpp"


using namespace std;

struct LidarPcl {
    int count;
    int n_channels;
    float* intensity;
    float* x;
    float* y;
    float* z;
    float* time;
    int* ring;
    vector<vector<int>> index_rings;
    float* theta;
    float* psi;
    float* rho;
    bool* mask;

    LidarPcl(int n_channels_) : n_channels(n_channels_) {
#ifdef _VERBOSE_
    cout << "lidar pcl is generated.\n";
#endif
        count = 0;
        intensity = (float*)custom_aligned_malloc(sizeof(float)*300000);
        x    = (float*)custom_aligned_malloc(sizeof(float)*300000);
        y    = (float*)custom_aligned_malloc(sizeof(float)*300000);
        z    = (float*)custom_aligned_malloc(sizeof(float)*300000);
        time = (float*)custom_aligned_malloc(sizeof(float)*300000);
        ring = (int*)custom_aligned_malloc(sizeof(int)*300000);
        

        theta = (float*)custom_aligned_malloc(sizeof(float)*300000);
        psi   = (float*)custom_aligned_malloc(sizeof(float)*300000);
        rho   = (float*)custom_aligned_malloc(sizeof(float)*300000);
        mask  = (bool*) custom_aligned_malloc(sizeof(bool) *300000);
        
        // initialize with 'true'
        bool* ptr = mask;
        for(int i = 0; i < 300000; ++i, ++ptr) *ptr = true;

        index_rings = vector<vector<int>>(n_channels, vector<int>(0));
    };
    ~LidarPcl(){
        if(intensity != nullptr) custom_aligned_free((void*)intensity);
        if(x != nullptr)     custom_aligned_free((void*)x);
        if(y != nullptr)     custom_aligned_free((void*)y);
        if(z != nullptr)     custom_aligned_free((void*)z);
        if(time != nullptr)  custom_aligned_free((void*)time);
        if(ring != nullptr)  custom_aligned_free((void*)ring);
        if(psi != nullptr)   custom_aligned_free((void*)psi);
        if(rho != nullptr)   custom_aligned_free((void*)rho);
        if(theta != nullptr) custom_aligned_free((void*)theta);
        if(mask != nullptr) custom_aligned_free((void*)mask);
    };

    void deleteMaskInvalid(){ // stack valid element from front.

        bool* cur_mask = mask;
        bool* itr_search_end = mask+count;
        bool* ins_mask = mask;
        float* ins_x = x; float* cur_x = x;
        float* ins_y = y; float* cur_y = y;
        float* ins_z = z; float* cur_z = z;
        float* ins_t = time; float* cur_t = time;
        int*   ins_r = ring; int* cur_r = ring;
        float* ins_i = intensity; float* cur_i = intensity;

        int cnt_valid = 0;
        for(; cur_mask < itr_search_end;
         ++cur_mask, ++cur_x, ++cur_y,++cur_z, ++cur_t, ++cur_r, ++cur_i) {
            if(*cur_mask) {
                ++cnt_valid;
                *(ins_mask++) = true;
                *(ins_x++) = *cur_x;
                *(ins_y++) = *cur_y;
                *(ins_z++) = *cur_z;
                *(ins_t++) = *cur_t;
                *(ins_r++) = *cur_r;
                *(ins_i++) = *cur_i;
            }
        }
        count = cnt_valid;
    };

    void gatherRingIndex(){
        // pre allocate memories
        for(int ch = 0; ch < n_channels; ++ch)
            index_rings[ch].reserve(4000);

        int* itr_ring = ring;
        int* itr_ring_end = ring + count;
        int cnt = 0;
        for(; itr_ring < itr_ring_end; ++itr_ring, ++cnt)
            index_rings[*itr_ring].push_back(cnt);
#ifdef _VERBOSE_
    for(int ch = 0; ch < n_channels; ++ch)
        cout << " ch["<<ch<<"] # elem: "<<index_rings[ch].size() <<"\n";
#endif
    };

    void generateThetaPsi(){
        float twopi = 2.0f*PI;

        int n_pts = count;
        float* itr_x_end = x + count;
        float* itr_x = x;
        float* itr_y = y;
        float* itr_z = z;
        float* itr_th  = theta;
        float* itr_psi = psi;
        float* itr_rho = rho;

        for(; itr_x < itr_x_end; ++itr_x, ++itr_y, ++itr_z, ++itr_th, ++itr_psi, ++itr_rho){
            float& x_ = *itr_x;
            float& y_ = *itr_y;
            float& z_ = *itr_z;

            *itr_rho = sqrt(x_*x_ + y_*y_ + z_*z_);
            *itr_th  = std::asin(z_/(*itr_rho));
            float invrhocos = 1.0f/((*itr_rho) * std::cos(*itr_th));

            float cospsi = (*itr_x)*invrhocos;
            float sinpsi = (*itr_y)*invrhocos; 
            if(cospsi >= 0.0){
                if(sinpsi >= 0.0) *itr_psi = std::acos(cospsi);// 1 quadrant
                else *itr_psi = twopi-std::acos(cospsi); // 4 quadrant
            }
            else{
                if(sinpsi >= 0.0) *itr_psi = PI-std::acos(-cospsi); // 2 quadrant
                else *itr_psi = PI+std::acos(-cospsi); // 3 quadrant;
            }
            if(*itr_psi >= twopi) *itr_psi -= twopi;
        }
#ifdef _VERBOSE_
    itr_th  = theta;
    itr_psi = psi;
    itr_rho = rho;
    for(; itr_th < theta + count; ++itr_th, ++itr_psi, ++itr_rho)
        cout << "rho theta psi: " << *itr_rho << "," << *itr_th << "," << *itr_psi <<"\n";
#endif
    };
};

#endif