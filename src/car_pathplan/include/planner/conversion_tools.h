#ifndef CONVERSION_TOOLS_H
#define CONVERSION_TOOLS_H

#include "tf2/LinearMath/Quaternion.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"



tf2::Quaternion twoD_yaw_to_tf2qua(double yaw_rad)
{
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, yaw_rad);
    myQuaternion = myQuaternion.normalize();
    return myQuaternion;
}



geometry_msgs::Quaternion tf2qua_to_geoQua(tf2::Quaternion tf2quaternion)
{
    geometry_msgs::Quaternion myQuaternion;
    myQuaternion.w = tf2quaternion.getW();
    myQuaternion.x = tf2quaternion.getX();
    myQuaternion.y = tf2quaternion.getY();
    myQuaternion.z = tf2quaternion.getZ();
    return myQuaternion;
}

#endif
























