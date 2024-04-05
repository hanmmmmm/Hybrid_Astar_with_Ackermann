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


#ifndef HAWA_COMMON_INCLUDES_H
#define HAWA_COMMON_INCLUDES_H

#include <array>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <chrono>
#include <cmath>

#include <deque>

#include <Eigen/Geometry>

#include <iostream>

#include <limits>

#include <map>
#include <math.h>
#include <mutex>

#include <queue>

#include <set>
#include <stdexcept>
#include <string>

#include <unordered_map>

#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace hawa
{

static const double k_pi2 = M_PI * 2.0;

static double mod2pi(double angle)
{
    return angle - k_pi2 * std::floor(angle / k_pi2);
}



} // namespace hawa

#endif // HAWA_COMMON_INCLUDES_H