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
 * @file class_purepursuit.h
 * @author Mingjie
 * @brief This is the implementation of the pure pursuit algorithm.
            The other parts like testing and ros-node are wrappers of this class.
 * @version 0.2
 * @date 2023-11-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#ifndef HAWA_CLASS_PUREPURSUIT_H
#define HAWA_CLASS_PUREPURSUIT_H


#include <iostream>
#include <vector>
#include <array>
#include <math.h>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "common/class_elemental_path2d_segment.h"
#include "common/class_elemental_pose2d.h"
#include "common/tools_angles.h"
#include "tools_purepursuit.h"

namespace hawa
{

/**
 * @brief The my implementation of the pure pursuit algorithm. 
 * This class has only the algorithm things, it does not have ros things. 
*/
class ClassPurePursuit
{
private:
    StructParameters m_parameters_;

    std::vector<std::array<geometry_msgs::msg::Point, 2>> m_vector_target_points_;

    geometry_msgs::msg::PointStamped m_actual_target_points_;

    StructPose m_robot_pose_;

    ClassPath2DSegment m_the_segment_;

private:
    void computeLookAheadDistnace();

public:
    ClassPurePursuit();
    ~ClassPurePursuit();

    bool setVehicleWheelbaseMeter(double val);  

    bool setTargetLinearSpeedMps(double val);

    bool setLookAheadCoefficient(double val);

    void setPathSegment(ClassPath2DSegment& r_segment);

    void setRobotPose(double x_meter, double y_meter, double yaw_rad);

    void findTargetPoint(bool& r_success);

    void getTargetPoint(geometry_msgs::msg::PointStamped& r_target);

    void solveForSpeedCommand(bool& r_success, double& r_steer_rad);

    void findPreciseTargetPoint(std::array<geometry_msgs::msg::Point, 2> two_pathpoints, 
                                geometry_msgs::msg::PointStamped& r_result_point);

};

} // namespace hawa


#endif