//
//  timer.hpp
//  FC_Safe_vector
//
//  Created by blue-i on 02/09/2017.
//  Copyright © 2017 blue-i. All rights reserved.
//

#ifndef timer_hpp
#define timer_hpp

#include <stdio.h>
#include <ctime>
#include <cstdlib>
#include <iomanip>
#include <chrono>


class Timer {
    double timeout;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_t;
    
public:
    Timer(int freq)
    {
        this->timeout = 1000.000/freq;
        this->start_t = std::chrono::high_resolution_clock::now();
    }
    ~Timer()
    {
        
    }
    
    bool check(void);
    double set_timeout(double t);
    double get_timeout(void);
};
#endif /* timer_hpp */
