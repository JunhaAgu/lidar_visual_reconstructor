#ifndef _LIDARPCL_H_
#define _LIDARPCL_H_


#include <iostream>
#include <vector>
#include <algorithm> // std::sort ,std::stable_sort
#include <numeric> // std::iota
#include "custom_memory.hpp"


#define PI 3.141592653589793238
#define D2R PI/180.0
#define R2D 180.0/PI


using namespace std;

// stackoverflow, "c-sorting-and-keeping-track-of-indexes"
template <typename T>
vector<int> sortIndexes(const vector<T>& v){
    // initialize original index locations
    vector<int> idx(v.size());
    std::iota(idx.begin(), idx.end(), 0);

    // sort indexes based on comparing values in v
    // using std::stable_sort instead of std::sort
    // to avoid unnecessary index re-orderings
    // when v contains elements of equal values
    std::stable_sort(idx.begin(), idx.end(), [&v](int i1, int i2) {return v[i1] <= v[i2];});
    return idx;
};


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
        if(mask != nullptr)  custom_aligned_free((void*)mask);
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
        
        // pre allocate memories (or re-initialize).
        for(int ch = 0; ch < n_channels; ++ch){
            if(index_rings[ch].capacity() > 0) index_rings[ch].resize(0);
            else index_rings[ch].reserve(5000);
        }

        int* itr_ring = ring;
        int* itr_ring_end = ring + count;
        int cnt = 0;
        for(; itr_ring < itr_ring_end; ++itr_ring)
            index_rings[*itr_ring].push_back(cnt++);

#ifdef _VERBOSE_
    cout << " lidar_pcl cnt : " << cnt << endl;
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
            float x_ = *itr_x;
            float y_ = *itr_y;
            float z_ = *itr_z;

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

    void sortRingIndexByPsi(){
        vector<float> data_tmp;
        data_tmp.reserve(3000);
        vector<int> index_org;
        index_org.reserve(3000);
        vector<int> subindex_sorted;
        for(int c = 0; c < n_channels; ++c){
            data_tmp.resize(0);
            index_org.resize(0);
            for(auto itr = index_rings[c].begin(); itr != index_rings[c].end(); ++itr){
                data_tmp.push_back(*(psi+(*itr)));
                index_org.push_back(*itr);
            }
            subindex_sorted = sortIndexes(data_tmp); // ordering!
            for(int i =0; i < index_rings[c].size(); ++i)
                index_rings[c][i] = index_org[subindex_sorted[i]];
        }
    };

    void reorderingIndexAtJumping(){
        //string filePath = "/home/larrkchlaptop/res.txt";
        //ofstream writeFile(filePath.data());

        vector<int> index_copy;
        index_copy.reserve(3000);
        for(int ch = 0; ch < n_channels; ++ch){
            int n_pts_ch = index_rings[ch].size();
            int idx_jump = -1;
            for(int i= 0; i < n_pts_ch-1; ++i){
                float diff = *(psi + index_rings[ch][i+1]) - *(psi + index_rings[ch][i]);
                if(diff > 2){ // 2 radians
                    idx_jump = i;
                    break;
                }
            }
            if(idx_jump == -1) continue;
            // change index!
            index_copy.resize(0);
            for(auto itr = index_rings[ch].begin(); itr != index_rings[ch].end(); ++itr)
                index_copy.push_back(*itr);

#ifdef _VERBOSE_
    cout << "ch [" << ch <<"] jump [" << idx_jump << "]\n";
#endif
            /*std::stable_partition(index_rings[ch].begin(),index_rings[ch].end(),
                [&idx_jump](int i)->bool {return i > idx_jump;});*/
            
            for(int i = 0; i < n_pts_ch-idx_jump-1; ++i)
                index_rings[ch][i] = index_copy[i+idx_jump+1];
            for(int i = n_pts_ch-idx_jump-1; i < n_pts_ch; ++i)
                index_rings[ch][i] = index_copy[i-n_pts_ch+idx_jump+1];
           // for(auto itr = index_rings[ch].begin(); itr != index_rings[ch].end(); ++itr)
           //     writeFile << ch<<", "<<*itr<<", " << *(psi+*itr)
           //     <<", " << *(x+*itr) << ", " << *(y+*itr) << ", " <<*(z+*itr) 
           //     <<"\n";   
        }
        //writeFile.close();
    };
};

#endif