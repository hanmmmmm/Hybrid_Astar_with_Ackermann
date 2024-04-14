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
 * This file is a collection of some tool functions used in the pure pursuit.
*/


#ifndef TOOLS_ANGLES
#define TOOLS_ANGLES

#include <array>
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/quaternion.hpp"

namespace hawa
{

class Tools
{
private:
    Tools();

public:
    ~Tools() {}

    static double mod2pi( double angle);
    static double computeYawOf2Points(double x1, double y1, double x2, double y2);
    static bool checkIfPointOnLeftOfLine(double ax, double ay, double bx, double by, double cx, double cy );
    static double quaternionToEularYaw(const double q_x, const double q_y, const double q_z, const double q_w);
    static double quaternionToEularYaw(const tf2::Quaternion q_in);
    static double quaternionToEularYaw(const geometry_msgs::msg::Quaternion q_in);
    static double calcAngleByThreePoints(std::array<double,3> p1, std::array<double,3> p2, std::array<double,3> p3);
};

} // namespace hawa




#endif