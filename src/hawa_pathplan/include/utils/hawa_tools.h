#ifndef HAWA_TOOLS_H
#define HAWA_TOOLS_H


#include <chrono>


/**
 * @brief 
 * return time in seconds, epoch time. 
 */
double helper_get_time()
{
    return std::chrono::high_resolution_clock::now().time_since_epoch().count()/1000000000.0; 
}









#endif
