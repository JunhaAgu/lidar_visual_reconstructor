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

    LidarPcl(int n_channels_) : n_channels(n_channels_) {
        cout << "lidar pcl is generated.\n";
        count = 0;
        intensity = (float*)custom_aligned_malloc(sizeof(float)*300000);
        x = (float*)custom_aligned_malloc(sizeof(float)*300000);
        y = (float*)custom_aligned_malloc(sizeof(float)*300000);
        z = (float*)custom_aligned_malloc(sizeof(float)*300000);
        time = (float*)custom_aligned_malloc(sizeof(float)*300000);
        ring = (int*)custom_aligned_malloc(sizeof(int)*300000);
        

        theta = (float*)custom_aligned_malloc(sizeof(float)*300000);
        psi = (float*)custom_aligned_malloc(sizeof(float)*300000);
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
    };
};

#endif