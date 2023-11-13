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
 * @file class_node_test_pure_pursuit.h
 * @author Mingjie
 * @brief The class for testing and implementing the pure pursuit algorithm. This does not utilize the path segmentation featrue, so
 *  only it only works with single path-segments or a long smooth path.
 * @version 0.1
 * @date 2023-02-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef CLASS_NODE_PURE_PURSUIT
#define CLASS_NODE_PURE_PURSUIT


#include <iostream>
#include <mutex>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/Marker.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

#include "class_purepursuit.h"

#include "../common/class_elemental_path2d_segment.h"
#include "../common/tools_angles.h"
#include "../common/class_multi_segment_manager.h"

/**
 * @brief This class is made for utilizing the ClassPurePursuit, and let it being interactive 
 * with ROS.
*/
class ClassNodePurePursuit
{
private:
    ros::NodeHandle nh_;

    ClassPurePursuit m_controller_purepursuit_;

    ClassPath2DSegment m_current_path_segment_;
    nav_msgs::Odometry m_robot_odom_msg_;

    std::mutex m_mutex_path_;

    ClassHawaMultiSegmentManager m_segment_manager_;


    // For algorithm
    ros::Subscriber m_suber__path_;
    ros::Subscriber m_suber__odom_;
    ros::Publisher m_puber__ackerman_msg_;
    std::string m_topic_name__path_subscribed_;
    std::string m_topic_name__odom_subscribed_;
    std::string m_topic_name__ackerman_msg_;
    ros::Timer m_periodic_controller_;
    double m_controller_interval_;

    double m_expected_speed_mps_, m_look_ahead_ratio_;

    // For human interactivity only
    ros::Publisher m_puber__target_point_marker_;
    ros::Publisher m_puber__current_path_;
    ros::Subscriber m_suber__speed_mps_;
    ros::Subscriber m_suber__look_coefficient_;
    std::string m_topic_name__target_point_marker_;
    std::string m_topic_name__speed_mps_subscribed_;
    std::string m_topic_name__look_coefficient_subscribed_;
    std::string m_topic_name__current_path_published_;

    

private:
    void pathCallback(const nav_msgs::Path::ConstPtr &msg);

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

    void speedCallback(const std_msgs::Float64::ConstPtr &msg);

    void ratioCallback(const std_msgs::Float64::ConstPtr &msg);

    void controllerUpdate(const ros::TimerEvent &event);

    bool validateOdomMsg();

    double decideSpeedMps();

public:
    ClassNodePurePursuit(const ros::NodeHandle nh_in_);
    ~ClassNodePurePursuit();
};

ClassNodePurePursuit::ClassNodePurePursuit(const ros::NodeHandle nh_in_) : nh_{nh_in_}
{
    m_topic_name__target_point_marker_ = "/target_point_marker";
    m_topic_name__ackerman_msg_ = "/ackermann_cmd";

    m_topic_name__odom_subscribed_ = "/odometry";
    m_topic_name__path_subscribed_ = "/path";
    m_topic_name__current_path_published_ = "/current_segment";

    m_topic_name__speed_mps_subscribed_ = "/expected_speed_mps";
    m_topic_name__look_coefficient_subscribed_ = "/look_ahead_ratio";

    m_controller_interval_ = 0.1;

    m_expected_speed_mps_ = 0.6;
    m_look_ahead_ratio_ = 0.9;

    m_robot_odom_msg_.header.stamp.fromSec(0);

    m_puber__target_point_marker_ = nh_.advertise<visualization_msgs::Marker>(m_topic_name__target_point_marker_, 10);
    m_puber__ackerman_msg_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(m_topic_name__ackerman_msg_, 10);

    m_puber__current_path_ = nh_.advertise<nav_msgs::Path>(m_topic_name__current_path_published_, 10);

    m_suber__path_ = nh_.subscribe(m_topic_name__path_subscribed_, 1, &ClassNodePurePursuit::pathCallback, this);
    m_suber__odom_ = nh_.subscribe(m_topic_name__odom_subscribed_, 1, &ClassNodePurePursuit::odomCallback, this);

    m_suber__speed_mps_ = nh_.subscribe(m_topic_name__speed_mps_subscribed_, 1, &ClassNodePurePursuit::speedCallback, this);
    m_suber__look_coefficient_ = nh_.subscribe(m_topic_name__look_coefficient_subscribed_, 1, &ClassNodePurePursuit::ratioCallback, this);

    m_periodic_controller_ = nh_.createTimer(ros::Duration(m_controller_interval_), &ClassNodePurePursuit::controllerUpdate, this);

    ROS_INFO_STREAM("ClassPathPlanner inti Done");

    return;
}


ClassNodePurePursuit::~ClassNodePurePursuit()
{
}


/**
 * @brief callback when receiving path msgs. Pass it to the path manager module.
 * @param msg ros topic msg.
*/
void ClassNodePurePursuit::pathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    if(msg->poses.size() < 2)
    {
        ROS_WARN_STREAM("path msg size < 2.  Do not proceed.");
        return;
    }
    m_mutex_path_.lock();
    m_segment_manager_.setPath(*msg);
    m_mutex_path_.unlock();
}

/**
 * @brief For updating the value of the robot pose.
 * @param msg ros topic msg.
*/
void ClassNodePurePursuit::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    m_robot_odom_msg_.child_frame_id = msg->child_frame_id;
    m_robot_odom_msg_.header = msg->header;
    m_robot_odom_msg_.pose = msg->pose;
    m_robot_odom_msg_.twist = msg->twist;
}

/**
 * @brief For updating the value of the target speed in mps
 * @param msg ros topic msg.
*/
void ClassNodePurePursuit::speedCallback(const std_msgs::Float64::ConstPtr &msg)
{
    m_expected_speed_mps_ = msg->data;
}

/**
 * @brief For updating the value of the m_look_ahead_ratio_ by ros topic.
 * @param msg ros topic msg.
*/
void ClassNodePurePursuit::ratioCallback(const std_msgs::Float64::ConstPtr &msg)
{
    m_look_ahead_ratio_ = msg->data;
}

/**
 * @brief Main function to be called by timer regualerly.
 * @param event ros timer event.
*/
void ClassNodePurePursuit::controllerUpdate(const ros::TimerEvent &event)
{
    if( ! validateOdomMsg() )
    {
        ROS_WARN_STREAM("Invalid robot odom msg.  Do not proceed.");
        return;
    }

    m_mutex_path_.lock();

    ROS_INFO_STREAM("m_counter_current_segment_ " << m_segment_manager_.getCounter());

    m_segment_manager_.update(m_robot_odom_msg_.pose.pose.position.x, 
                    m_robot_odom_msg_.pose.pose.position.y,
                    quaternion_to_eular_yaw(m_robot_odom_msg_.pose.pose.orientation));
    if (m_segment_manager_.doesPathExist())
    {
        m_current_path_segment_ = m_segment_manager_.getCurrentSegment();
        m_controller_purepursuit_.setPathSegment(m_current_path_segment_);
        nav_msgs::Path _ros_current_path = m_segment_manager_.getCurrentSegment().toRosPath();
        _ros_current_path.header = m_segment_manager_.getOriginalPath().header;
        m_puber__current_path_.publish(_ros_current_path);
    }

    double _signed_speed_mps = 0;
    if (m_current_path_segment_.isForward())
    {
        // _signed_speed_mps = std::abs(m_expected_speed_mps_);
        _signed_speed_mps = std::abs(decideSpeedMps());
    }
    else
    {
        // _signed_speed_mps = - std::abs(m_expected_speed_mps_);
        _signed_speed_mps = - std::abs(decideSpeedMps());
    }

    if (m_segment_manager_.didFinishAll())
    {
        _signed_speed_mps = 0;
    }

    // the values for the parameters of the controller algorithm.
    m_controller_purepursuit_.setLookAheadCoefficient(m_look_ahead_ratio_);
    m_controller_purepursuit_.setTargetLinearSpeedMps(_signed_speed_mps);
    m_controller_purepursuit_.setVehicleWheelbaseMeter(0.25);
    m_controller_purepursuit_.setRobotPose(m_robot_odom_msg_.pose.pose.position.x, 
                                  m_robot_odom_msg_.pose.pose.position.y,
                                  quaternion_to_eular_yaw(m_robot_odom_msg_.pose.pose.orientation));

    bool _valid_target_point = false;
    m_controller_purepursuit_.findTargetPoint(_valid_target_point);
    if( ! _valid_target_point )
    {
        ROS_WARN_STREAM("Invalid target point.  Do not proceed.");
        m_mutex_path_.unlock();
        return;
    }

    // get the target point position, and visulize it in RVIZ as a sphere. 
    geometry_msgs::PointStamped _target_point;
    m_controller_purepursuit_.getTargetPoint(_target_point);

    visualization_msgs::Marker _target_point_marker;
    _target_point_marker.pose.position.x = _target_point.point.x;
    _target_point_marker.pose.position.y = _target_point.point.y;
    _target_point_marker.header.frame_id = "map";
    _target_point_marker.type = visualization_msgs::Marker::SPHERE;
    _target_point_marker.action = visualization_msgs::Marker::MODIFY;

    _target_point_marker.scale.x = 0.1;
    _target_point_marker.scale.y = 0.1;
    _target_point_marker.scale.z = 0.1;

    _target_point_marker.color.a = 0.9; // Don't forget to set the alpha!
    _target_point_marker.color.r = 1.0;
    _target_point_marker.color.g = 0.0;
    _target_point_marker.color.b = 0.0;
    m_puber__target_point_marker_.publish(_target_point_marker);
    // visulization done.

    bool _success;  double _steer_needed = 0.0;
    m_controller_purepursuit_.solveForSpeedCommand(_success, _steer_needed);

    std::cout << _steer_needed << std::endl;

    ackermann_msgs::AckermannDriveStamped  _akm_cmd_msg;
    _akm_cmd_msg.drive.speed = _signed_speed_mps;
    _akm_cmd_msg.drive.steering_angle = _steer_needed;
    _akm_cmd_msg.header.stamp = ros::Time::now();

    m_puber__ackerman_msg_.publish(_akm_cmd_msg);

    m_mutex_path_.unlock();
}

/**
 * @brief Check if the robot odom msg is up to date. 
 * @return true when valid;  false when invalid.
*/
bool ClassNodePurePursuit::validateOdomMsg()
{
    if( ros::Time::now().toSec() - m_robot_odom_msg_.header.stamp.toSec() >1.0 )
    {
        return false;
    }
    return true;
}

/**
 * @brief 
*/
double ClassNodePurePursuit::decideSpeedMps()
{
    double _dist_to_end = m_segment_manager_.getDistToEnd();
    double _dist_threshold = 1.0;
    double _min_speed_mps = 0.4;
    if (_dist_to_end >= _dist_threshold)
    {
        return m_expected_speed_mps_;
    }
    double _slope = (m_expected_speed_mps_ - _min_speed_mps) / _dist_threshold;

    return _slope * _dist_to_end + _min_speed_mps;
}

#endif 

