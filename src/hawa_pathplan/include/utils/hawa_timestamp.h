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
 * @file hawa_timestamp.h
 * @author Mingjie
 * @brief This class provides time stamp managment.  
 * @version 0.2
 * @date 2023-11-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#ifndef HAWA_TIMESTAMP_CLASS_H
#define HAWA_TIMESTAMP_CLASS_H

#include "hawa_tools.h"

/**
 * @brief This class provides time stamp managment. 
*/
class ClassHawaTimeStamp
{
private:
    double m_stamp_;
public:
    ClassHawaTimeStamp(/* args */);
    ~ClassHawaTimeStamp();
    void stampNow();
    bool checkPass(const double duration_sec);
};

ClassHawaTimeStamp::ClassHawaTimeStamp(/* args */)
{
    m_stamp_ = 0;
}

ClassHawaTimeStamp::~ClassHawaTimeStamp()
{
}

void ClassHawaTimeStamp::stampNow()
{
    m_stamp_ = helperGetTime();
}


bool ClassHawaTimeStamp::checkPass(const double duration_sec)
{
    return (helperGetTime() - m_stamp_) > duration_sec;
}


#endif
