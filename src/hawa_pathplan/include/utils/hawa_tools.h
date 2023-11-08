// MIT License

// Copyright (c) 2023 Mingjie

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


/**
 * This file contains several functions and classes that would be regulerly used 
 * in this project, but also they are just general tools instead of specific algorithm
 * things. 
*/

#ifndef HAWA_TOOLS_H
#define HAWA_TOOLS_H


#include <chrono>


/**
 * @brief return time in seconds, epoch time. 
 * @return time, double type.
 */
double helperGetTime()
{
    return std::chrono::high_resolution_clock::now().time_since_epoch().count()/1000000000.0; 
}


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
