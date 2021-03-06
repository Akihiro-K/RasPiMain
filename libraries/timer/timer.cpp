//
//  timer.cpp
//  FC_Safe_vector
//
//  Created by blue-i on 02/09/2017.
//  Copyright © 2017 blue-i. All rights reserved.
//

#include "timer.hpp"


bool Timer::check(void)
{
    bool retval = false;
    std::chrono::high_resolution_clock::time_point now_t = std::chrono::high_resolution_clock::now();
    double ellapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now_t - this->start_t).count();
    if(ellapsed >= this->timeout)
    {
        this->start_t = now_t;
        retval = true;
    }

    return retval;
}
double Timer::set_timeout(double t)
{
   return this->timeout = 1000.00/t;

}
double Timer::get_timeout(void)
{
    return this->timeout;
}
double Timer::time_since_last_check(void)
{
  std::chrono::high_resolution_clock::time_point now_t = std::chrono::high_resolution_clock::now();
  double ellapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now_t-this->start_t).count();
  return ellapsed;
}
