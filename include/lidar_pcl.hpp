#ifndef _LIDARPCL_H_
#define _LIDARPCL_H_
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
};

#endif