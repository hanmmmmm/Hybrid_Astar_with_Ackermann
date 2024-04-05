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
 * @file hawa_conversion_tools.h
 * @author Mingjie
 * @brief This is a file containing several functions for doing the conversion tasks.
 * @version 0.2
 * @date 2023-11-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#ifndef HAWA_CONVERSION_TOOLS_H
#define HAWA_CONVERSION_TOOLS_H

#include "common_includes.h"

#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Matrix3x3.h>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "custom_data_types.h"


namespace hawa
{

/**
 * @brief This class provides several functions for doing the conversion tasks.
*/
class ClassUtilsConverters
{
private:
    ClassUtilsConverters(){};
    
public:
    ~ClassUtilsConverters(){};

    static double pi2;

    // static double mod2pi(double angle);

    static tf2::Quaternion twodYawToTf2qua(double yaw_rad);

    static geometry_msgs::msg::Quaternion tf2quaToGeoQua(tf2::Quaternion tf2quaternion);

    static double StampedTransformToYaw(const geometry_msgs::msg::TransformStamped * ptr_trans);
    
    static double geoQuaToYaw(const geometry_msgs::msg::Quaternion* ptr_geoqua);

    static std::array<double, 3> cvtStructPoseReal2Array3(const StructPoseReal& pose);
    static std::array<double, 3> cvtStructPoseReal2Array3(StructPoseReal* pose);

    static inline void resetStructPoseReal(StructPoseReal* ptr_target);
    static inline void resetStructPoseReal(StructPoseReal& r_target);

};

} // namespace hawa



#endif
