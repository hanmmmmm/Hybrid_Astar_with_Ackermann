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
 * @file hawa_data_containers.h
 * @author Mingjie
 * @brief This is a file containing several structs for storing groups of values. 
 * @version 0.2
 * @date 2023-11-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef HAWA_DATA_CONTAINERS_H
#define HAWA_DATA_CONTAINERS_H


#include "common_includes.h"


namespace hawa
{


/**
 * @brief Storing the pose in real world metric coordinate. 
*/
struct StructPoseReal
{
    double x = 0;  // meter
    double y = 0;  // meter
    double yaw = 0;  // radian

    StructPoseReal() {}

    StructPoseReal(double x_, double y_, double yaw_) : x(x_), y(y_), yaw(yaw_) {}
};



/**
 * @brief Storing the pose in grid-wise coordinate. 
*/
struct StructPoseGrid
{
    int x = 0;
    int y = 0;
    int yaw = 0;

    StructPoseGrid() {}

    StructPoseGrid(int x, int y, int yaw) : x(x), y(y), yaw(yaw) {}

    bool operator!=(const StructPoseGrid &p) const 
    {
        return ! (x == p.x && y == p.y && yaw == p.yaw) ;
    }
};


} // namespace hawa

#endif