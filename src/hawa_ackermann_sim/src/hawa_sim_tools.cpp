
#include "hawa_ackermann_sim/hawa_sim_tools.h"



namespace hawa
{

/**
 * @brief Modulate the given angle into the range of [0, 2pi]
 * @param a The given angle.
 * @return The processed angle.
*/
double ClassHawaSimTools::mod2pi( double a)
{
    double angle = a;
    while (angle > 2*M_PI){
        angle -= 2*M_PI;
    }
    while (angle < 0){
        angle += 2*M_PI;
    }
    return angle;
}

/**
 * @brief return time in seconds, epoch time. 
 */
double ClassHawaSimTools::getTimeSecond()
{
    return std::chrono::high_resolution_clock::now().time_since_epoch().count()/1000000000.0; 
}


}

