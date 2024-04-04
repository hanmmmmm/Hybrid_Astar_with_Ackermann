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
 * @file hawa_timer_class.h
 * @author Mingjie
 * @brief This is a class that would be used to timing a part of the code. 
 * @version 0.2
 * @date 2023-11-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#ifndef CLASS_HAWA_TIMER_H
#define CLASS_HAWA_TIMER_H

#include "class_tool_general.h"

namespace hawa
{
    
/**
 * This is a class that would be used to timing a part of the code. 
*/
class ClassHawaTimer
{
private:
    double m_t1_, m_t2_;
    double m_dt_;
public:
    ClassHawaTimer(){};
    ~ClassHawaTimer(){};

    void startNow();
    void endTiming();
    double getDuration();
    double getDurationNonStop();
    static double getTimeSecs();
};

} // namespace hawa








#endif