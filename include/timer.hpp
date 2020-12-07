#ifndef _TIMER_H_
#define _TIMER_H_
#include <iostream>
#include <chrono>
using namespace std;

auto start = chrono::high_resolution_clock::now();
auto finish = chrono::high_resolution_clock::now();
auto gap = finish - start;

void tic(){
    start = chrono::high_resolution_clock::now();
}
double toc(bool flag_verbose){
    finish = chrono::high_resolution_clock::now();
    gap = finish - start;
    if(flag_verbose){
        cout << " exec time: " << (double)(gap/chrono::microseconds(1)) / 1000.0 << "[ms]\n";
    }
    return (double)(gap/chrono::microseconds(1))/1000.0;
}

#endif