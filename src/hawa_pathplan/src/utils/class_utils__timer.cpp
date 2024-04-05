
#include "utils/class_utils__timer.h"


namespace hawa
{

/**
 * @brief Call this when you want to Start the timing. 
*/
void ClassHawaTimer::startNow()
{
    m_t1_ = getTimeSecs();
}
 
/**
 * @brief Call this when you want to End the timing. 
*/
void ClassHawaTimer::endTiming()
{
    m_t2_ = getTimeSecs();
    m_dt_ = m_t2_ - m_t1_;
}

/**
 * @brief Call this when you want to get the value of duration. Note, endTiming() should 
 * be called before this function. 
 * @return the duration between the moments of executing startNow() and endTiming(). Double type.
*/
double ClassHawaTimer::getDuration()
{
    return m_dt_;
}

/**
 * @brief Call this when you want to get the value of duration. This function does not 
 * depend on endTiming().
 * @return the duration since the moments of executing startNow() until now. Double type.
*/
double ClassHawaTimer::getDurationNonStop()
{
    return getTimeSecs() - m_t1_;
}


/**
 * @brief return time in seconds, epoch time. 
 * @return time, double type.
 */
double ClassHawaTimer::getTimeSecs()
{
    return std::chrono::high_resolution_clock::now().time_since_epoch().count()/1000000000.0; 
}


}

