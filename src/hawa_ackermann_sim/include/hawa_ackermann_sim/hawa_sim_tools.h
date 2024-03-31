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
 * @file hawa_sim_tools.h
 * @author Mingjie
 * @brief Some commonly used functions in this project.
 * @version 0.2
 * @date 2023-11-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef HAWA_SIM_TOOLS_H
#define HAWA_SIM_TOOLS_H

#include <chrono>
// #include "math.hpp"
#include <math.h>

using std::chrono::high_resolution_clock;

/**
 * @brief Modulate the given angle into the range of [0, 2pi]
 * @param a The given angle.
 * @return The processed angle.
*/
double mod2pi( double a)
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
double getTimeSecond()
{
    return high_resolution_clock::now().time_since_epoch().count()/1000000000.0; 
}


#endif