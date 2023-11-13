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

#ifndef HAWA_TIMESTAMP_CLASS_H
#define HAWA_TIMESTAMP_CLASS_H


#include "hawa_tools.h"

/**
 * 
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
