#ifndef HAWA_CONVERSION_TOOLS_H
#define HAWA_CONVERSION_TOOLS_H

#include "tf2/LinearMath/Quaternion.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"


double mod_2pi(double angle)
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

double tfStampedTransform_to_yaw(const tf::StampedTransform* tftrans)
{
    // tf::Quaternion q(m_tf_robot_to_map_.getRotation().x(),
    //                 m_tf_robot_to_map_.getRotation().y(),
    //                 m_tf_robot_to_map_.getRotation().z(),
    //                 m_tf_robot_to_map_.getRotation().w());
    //  tfScalar yaw, pitch, roll;
    // tf::Matrix3x3 mat(q);
    // mat.getEulerYPR(yaw, pitch, roll);
    // m_start_pose_.yaw = mod_2pi(yaw);
    
    tf::Quaternion q(tftrans->getRotation().x(),
                    tftrans->getRotation().y(),
                    tftrans->getRotation().z(),
                    tftrans->getRotation().w());
    tfScalar yaw, pitch, roll;
    tf::Matrix3x3 mat(q);
    mat.getEulerYPR(yaw, pitch, roll);
    return mod_2pi(yaw);
}

double geoQua_to_yaw(const geometry_msgs::Quaternion* geoqua)
{
    // tf::Quaternion q(msg->pose.orientation.x,
    //                  msg->pose.orientation.y,
    //                  msg->pose.orientation.z,
    //                  msg->pose.orientation.w  );
    // tfScalar yaw, pitch, roll;
    // tf::Matrix3x3 mat(q);
    // mat.getEulerYPR(yaw, pitch, roll);

    tf::Quaternion q(geoqua->x,
                     geoqua->y,
                     geoqua->z,
                     geoqua->w);
    tfScalar yaw, pitch, roll;
    tf::Matrix3x3 mat(q);
    mat.getEulerYPR(yaw, pitch, roll);
    return mod_2pi(yaw);
}


#endif
