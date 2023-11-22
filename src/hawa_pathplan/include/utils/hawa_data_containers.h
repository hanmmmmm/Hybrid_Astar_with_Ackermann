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

#include <iostream>
#include <math.h>
#include <array>

/**
 * @brief Storing the pose in real world metric coordinate. 
*/
struct StructPoseReal
{
    double x;  // meter
    double y;  // meter
    double yaw;  // radian

    StructPoseReal()
    {
    }

    StructPoseReal(double x_in, double y_in, double yaw_in)
    {
        x = x_in;
        y = y_in;
        yaw = yaw_in;
    }

    /**
     * @brief Convert the pose value to a std array.
    */
    inline std::array<double, 3> toArray3()
    {
        return std::array<double, 3> {x, y, yaw};
    }

    /**
     * @brief Setup the values from another StructPoseReal.
    */
    inline void setFrom(StructPoseReal* ptr_in)
    {
        this->x = ptr_in->x;
        this->y = ptr_in->y;
        this->yaw = ptr_in->yaw;
    }
    
};

/**
 * @brief Reset the values in a StructPoseReal back to 0.
*/
inline void structPoseRealReset(StructPoseReal* ptr_target)
{
    ptr_target->x = 0;
    ptr_target->y = 0;
    ptr_target->yaw = 0;
}

/**
 * @brief Storing the pose in grid-wise coordinate. 
*/
struct StructPoseGrid
{
    int x;
    int y;
    int yaw;

    StructPoseGrid()
    {
    }

    StructPoseGrid(int x, int y, int yaw):x(x), y(y), yaw(yaw)
    {
    }

    bool operator!=(const StructPoseGrid &p) const {
        return ! (x == p.x && y == p.y && yaw == p.yaw) ;
    }

    inline std::array<int, 3> toArray3()
    {
        return std::array<int, 3> {x, y, yaw};
    }

    inline std::array<int, 2> toArray2()
    {
        return std::array<int, 2> {x, y};
    }

};




#endif