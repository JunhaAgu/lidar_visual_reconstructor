#include "frame.hpp"

Frame::Frame(int max_lvl_pyr){
    MAX_PYR_LVL = max_lvl_pyr;


    // preallocation
    img_pyr.reserve(MAX_PYR_LVL);
    du_pyr.reserve(MAX_PYR_LVL);
    dv_pyr.reserve(MAX_PYR_LVL);
    
};