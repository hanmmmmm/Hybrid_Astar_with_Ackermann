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



class ClassHawaTimer
{
private:
    double m_t1_, m_t2_;
    double m_dt_;
public:
    ClassHawaTimer();
    ~ClassHawaTimer();

    void startNow();
    void endTiming();
    double getDuration();
    double getDurationNonStop();
};

ClassHawaTimer::ClassHawaTimer(/* args */)
{
}

/// @brief Call this when you want to Start the timing. 
void ClassHawaTimer::startNow()
{
    m_t1_ = helper_get_time();
}

/// @brief Call this when you want to End the timing. 
void ClassHawaTimer::endTiming()
{
    m_t2_ = helper_get_time();
    m_dt_ = m_t2_ - m_t1_;
}

ClassHawaTimer::~ClassHawaTimer()
{
}

/// @brief Call this when you want to get the value of duration. 
double ClassHawaTimer::getDuration()
{
    return m_dt_;
}


double ClassHawaTimer::getDurationNonStop()
{
    return helper_get_time() - m_t1_;
}



#endif
