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

#ifndef HAWA_TOOLS_PUREPURSUIT_H
#define HAWA_TOOLS_PUREPURSUIT_H

/*
* @file tools_purepursuit.h
* @author Mingjie
* @brief This file contains some tools for the pure pursuit algorithm.
*/

#include <math.h>

/**
 * @brief compute the Euclidean distance between 2 points. 
 * @param x1 
 * @param y1
 * @param x2
 * @param y2
 * @return distance in meter.
*/
inline double computeDistanceMeter(double x1, double y1, double x2, double y2)
{
    return std::sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) );
}

/**
 * @brief For hodling the parameters for pure_pursuit algorithm.
*/
struct StructParameters
{
    double target_longitude_speed_mps_;  // meter per second
    double look_ahead_distance_meter_;
    double look_ahead_coefficient_;

    double k_max_target_speed_mps_;
    double k_max_look_ahead_coefficient_;
    double k_min_wheelbase_meter_;
    double k_max_wheelbase_meter_;

    double wheelbase_meter_;  
    // wheelbase is the distance between the centers of the front and rear tires on an ackermann vehicle.
};

/**
 * @brief For holding the values of robot pose.
*/
struct StructPose
{
    double x_meter;
    double y_meter;
    double yaw_rad;
};


#endif