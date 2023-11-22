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

#include "tf2/LinearMath/Quaternion.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"

/**
 * @brief modulate the given angle.
*/
double mod2pi(double angle)
{
    while (angle > 2 * M_PI)
    {
        angle -= 2 * M_PI;
    }
    while (angle < 0)
    {
        angle += 2 * M_PI;
    }
    return angle;
}

/**
 * @brief For a given yaw angle, generate the tf quaternion, by assuming the roll and pitch are 0.
 * @param yaw_rad The given yaw angle.
 * @return the tf quaternion.
*/
tf2::Quaternion twodYawToTf2qua(double yaw_rad)
{
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, yaw_rad);
    myQuaternion = myQuaternion.normalize();
    return myQuaternion;
}

/**
 * @brief Convert a given tf2 quaternion to geometry_msgs quaternion.
*/
geometry_msgs::Quaternion tf2quaToGeoQua(tf2::Quaternion tf2quaternion)
{
    geometry_msgs::Quaternion myQuaternion;
    myQuaternion.w = tf2quaternion.getW();
    myQuaternion.x = tf2quaternion.getX();
    myQuaternion.y = tf2quaternion.getY();
    myQuaternion.z = tf2quaternion.getZ();
    return myQuaternion;
}

/**
 * @brief For a given tf stamped transform, find out the yaw value in it. 
 * @param ptr_tftrans The pointer to the transform. 
 * @return The yaw value. 
*/
double tfStampedTransformToYaw(const tf::StampedTransform* ptr_tftrans)
{
    // tf::Quaternion q(m_tf_robot_to_map_.getRotation().x(),
    //                 m_tf_robot_to_map_.getRotation().y(),
    //                 m_tf_robot_to_map_.getRotation().z(),
    //                 m_tf_robot_to_map_.getRotation().w());
    //  tfScalar yaw, pitch, roll;
    // tf::Matrix3x3 mat(q);
    // mat.getEulerYPR(yaw, pitch, roll);
    // m_start_pose_.yaw = mod_2pi(yaw);
    
    tf::Quaternion q(ptr_tftrans->getRotation().x(),
                    ptr_tftrans->getRotation().y(),
                    ptr_tftrans->getRotation().z(),
                    ptr_tftrans->getRotation().w());
    tfScalar yaw, pitch, roll;
    tf::Matrix3x3 mat(q);
    mat.getEulerYPR(yaw, pitch, roll);
    return mod2pi(yaw);
}

/**
 * @brief For a given geometry quaternion, find out the yaw value in it. 
 * @param ptr_geoqua The pointer to the quaternion. 
 * @return The yaw value. 
*/
double geoQuaToYaw(const geometry_msgs::Quaternion* ptr_geoqua)
{
    // tf::Quaternion q(msg->pose.orientation.x,
    //                  msg->pose.orientation.y,
    //                  msg->pose.orientation.z,
    //                  msg->pose.orientation.w  );
    // tfScalar yaw, pitch, roll;
    // tf::Matrix3x3 mat(q);
    // mat.getEulerYPR(yaw, pitch, roll);

    tf::Quaternion q(ptr_geoqua->x,
                     ptr_geoqua->y,
                     ptr_geoqua->z,
                     ptr_geoqua->w);
    tfScalar yaw, pitch, roll;
    tf::Matrix3x3 mat(q);
    mat.getEulerYPR(yaw, pitch, roll);
    return mod2pi(yaw);
}


#endif
