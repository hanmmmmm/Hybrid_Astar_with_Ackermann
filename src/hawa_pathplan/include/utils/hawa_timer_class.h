#ifndef CLASS_HAWA_TIMER_H
#define CLASS_HAWA_TIMER_H



#include "hawa_tools.h"


/**
 * This is a class that would be used to timing a part of the code. 
*/
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

/**
 * @brief Call this when you want to Start the timing. 
*/
void ClassHawaTimer::startNow()
{
    m_t1_ = helperGetTime();
}
 
/**
 * @brief Call this when you want to End the timing. 
*/
void ClassHawaTimer::endTiming()
{
    m_t2_ = helperGetTime();
    m_dt_ = m_t2_ - m_t1_;
}

ClassHawaTimer::~ClassHawaTimer()
{
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
    return helperGetTime() - m_t1_;
}








#endif