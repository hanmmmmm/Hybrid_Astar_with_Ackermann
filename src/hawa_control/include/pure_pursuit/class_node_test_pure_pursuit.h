#ifndef CLASS_NODE_TEST_PURE_PURSUIT
#define CLASS_NODE_TEST_PURE_PURSUIT

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


#include <iostream>
#include <chrono>
#include <mutex>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/Marker.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

#include "class_purepursuit.h"

#include "../common/class_elemental_path2d_segment.h"
#include "../common/tools_angles.h"


class ClassNodeTestPurePursuit
{
private:
    ros::NodeHandle nh_;

    ClassPurePursuit controller_pp_;

    ClassPath2DSegment the_path_segment_;
    nav_msgs::Odometry robot_odom_;

    std::mutex mutex_path_;
    bool FLAG_path_msg_is_new_;


    // For algorithm
    ros::Subscriber suber__path_;
    ros::Subscriber suber__odom_;
    ros::Publisher puber__ackerman_msg_;
    std::string topic_name__path_subscribed_;
    std::string topic_name__odom_subscribed_;
    std::string topic_name__ackerman_msg_;
    tf::TransformListener tf_listener_;

    ros::Timer periodic_controller_;
    double controller_interval_;

    double speed_mps_, look_ahead_ratio_;

    // For human interactivity only
    ros::Publisher puber__target_point_marker_;
    ros::Subscriber suber__speed_mps_;
    ros::Subscriber suber__look_coefficient_;
    std::string topic_name__target_point_marker_;
    std::string topic_name__speed_mps_subscribed_;
    std::string topic_name__look_coefficient_subscribed_;

    

private:
    void path_callback(const nav_msgs::Path::ConstPtr &msg);

    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);

    void speed_callback(const std_msgs::Float64::ConstPtr &msg);

    void ratio_callback(const std_msgs::Float64::ConstPtr &msg);

    void controller_update(const ros::TimerEvent &event);

    bool validate_odom_msg();

public:
    ClassNodeTestPurePursuit(const ros::NodeHandle nh_in_);
    ~ClassNodeTestPurePursuit();
};

ClassNodeTestPurePursuit::ClassNodeTestPurePursuit(const ros::NodeHandle nh_in_) : nh_{nh_in_}
{
    topic_name__target_point_marker_ = "/target_point_marker";
    topic_name__ackerman_msg_ = "/ackermann_cmd";

    topic_name__odom_subscribed_ = "/odometry";
    topic_name__path_subscribed_ = "/path";

    topic_name__speed_mps_subscribed_ = "/expected_speed_mps";
    topic_name__look_coefficient_subscribed_ = "/look_ahead_ratio";

    controller_interval_ = 0.1;

    speed_mps_ = 0.8;
    look_ahead_ratio_ = 0.9;

    FLAG_path_msg_is_new_ = false;

    robot_odom_.header.stamp.fromSec(0);

    puber__target_point_marker_ = nh_.advertise<visualization_msgs::Marker>(topic_name__target_point_marker_, 10);
    puber__ackerman_msg_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(topic_name__ackerman_msg_, 10);

    suber__path_ = nh_.subscribe(topic_name__path_subscribed_, 1, &ClassNodeTestPurePursuit::path_callback, this);
    suber__odom_ = nh_.subscribe(topic_name__odom_subscribed_, 1, &ClassNodeTestPurePursuit::odom_callback, this);

    suber__speed_mps_ = nh_.subscribe(topic_name__speed_mps_subscribed_, 1, &ClassNodeTestPurePursuit::speed_callback, this);
    suber__look_coefficient_ = nh_.subscribe(topic_name__look_coefficient_subscribed_, 1, &ClassNodeTestPurePursuit::ratio_callback, this);

    periodic_controller_ = nh_.createTimer(ros::Duration(controller_interval_), &ClassNodeTestPurePursuit::controller_update, this);

    std::cout << "ClassPathPlanner inti Done" << std::endl;

    return;
}


ClassNodeTestPurePursuit::~ClassNodeTestPurePursuit()
{
}


/// @brief callback when receiving path msgs. Segments it and saves the result in container.
/// @param msg : nav_msgs::Path
void ClassNodeTestPurePursuit::path_callback(const nav_msgs::Path::ConstPtr &msg)
{
    if(msg->poses.size() < 2)
    {
        std::cout << "path msg size < 2.  Do not proceed." << std::endl;
        return;
    }

    mutex_path_.lock();
    int _count = 0;
    the_path_segment_.clear();
    for(auto ps : msg->poses)
    {
        double _yaw = quaternion_to_eular_yaw(ps.pose.orientation);
        ClassPose2D one_pose(ps.pose.position.x, ps.pose.position.y, _yaw);
        the_path_segment_.path_segment__original_.push_back(one_pose);
        the_path_segment_.path_segment__extended_.push_back(one_pose);
        _count ++;
    }
    FLAG_path_msg_is_new_ = true;
    mutex_path_.unlock();
    std::cout << "Put " << _count << " points in the path. Length: " << the_path_segment_.path_segment__original_.size() 
    << " " << the_path_segment_.path_segment__extended_.size() << " " << std::endl;
}


/// @brief To update the robot pose.
/// @param msg 
void ClassNodeTestPurePursuit::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // std::cout << "odom_callback." << std::endl;
    robot_odom_.child_frame_id = msg->child_frame_id;
    robot_odom_.header = msg->header;
    robot_odom_.pose = msg->pose;
    robot_odom_.twist = msg->twist;
    // std::cout << "odom_callback DONE." << std::endl;
}


/// @brief For updating the value of the target speed in mps
/// @param msg 
void ClassNodeTestPurePursuit::speed_callback(const std_msgs::Float64::ConstPtr &msg)
{
    speed_mps_ = msg->data;
}


/// @brief For updating the value of the look_ahead_ratio_
/// @param msg 
void ClassNodeTestPurePursuit::ratio_callback(const std_msgs::Float64::ConstPtr &msg)
{
    look_ahead_ratio_ = msg->data;
}


/// @brief Main function to be called by timer regualerly.
/// @param event 
void ClassNodeTestPurePursuit::controller_update(const ros::TimerEvent &event)
{
    if( ! validate_odom_msg() )
    {
        std::cout << "Invalid robot odom msg.  Do not proceed." << std::endl;
        return;
    }

    // the values for the parameters of the controller algorithm.
    controller_pp_.set_look_ahead_coefficient(look_ahead_ratio_);
    controller_pp_.set_target_linear_speed_mps(speed_mps_);
    controller_pp_.set_vehicle_wheelbase_meter(0.25);
    controller_pp_.set_robot_pose(robot_odom_.pose.pose.position.x, 
                                  robot_odom_.pose.pose.position.y,
                                  quaternion_to_eular_yaw(robot_odom_.pose.pose.orientation));
    // update the path data, when needed.
    mutex_path_.lock();
    if( FLAG_path_msg_is_new_ )
    {
        controller_pp_.set_path_segment(the_path_segment_);
        FLAG_path_msg_is_new_ = false;
    }

    bool _valid_target_point = false;
    controller_pp_.find_target_point(_valid_target_point);
    if( ! _valid_target_point )
    {
        std::cout << "Invalid target point.  Do not proceed." << std::endl;
        mutex_path_.unlock();
        return;
    }

    // get the target point position, and visulize it in RVIZ as a sphere. 
    geometry_msgs::PointStamped _target_point;
    controller_pp_.get_target_point(_target_point);

    visualization_msgs::Marker _target_point_marker;
    _target_point_marker.pose.position.x = _target_point.point.x;
    _target_point_marker.pose.position.y = _target_point.point.y;
    _target_point_marker.header.frame_id = "map";
    _target_point_marker.type = visualization_msgs::Marker::SPHERE;
    // _target_point_marker.action = visualization_msgs::Marker::ADD;
    _target_point_marker.action = visualization_msgs::Marker::MODIFY;

    _target_point_marker.scale.x = 0.1;
    _target_point_marker.scale.y = 0.1;
    _target_point_marker.scale.z = 0.1;

    _target_point_marker.color.a = 0.9; // Don't forget to set the alpha!
    _target_point_marker.color.r = 1.0;
    _target_point_marker.color.g = 0.0;
    _target_point_marker.color.b = 0.0;
    puber__target_point_marker_.publish(_target_point_marker);
    // visulization done.

    bool _success;  double _steer_needed = 0.0;
    controller_pp_.solve_for_speed_command(_success, _steer_needed);

    std::cout << _steer_needed << std::endl;

    ackermann_msgs::AckermannDriveStamped  _akm_cmd_msg;
    _akm_cmd_msg.drive.speed = speed_mps_;
    _akm_cmd_msg.drive.steering_angle = _steer_needed;
    _akm_cmd_msg.header.stamp = ros::Time::now();

    puber__ackerman_msg_.publish(_akm_cmd_msg);

    mutex_path_.unlock();
}


/// @brief 
/// @return true when valid;  false when invalid.
bool ClassNodeTestPurePursuit::validate_odom_msg()
{
    // std::cout << ros::Time::now().toSec() << std::endl;
    // std::cout << robot_odom_.header.stamp.toSec() << std::endl;
    // std::cout << ros::Time::now().toSec() - robot_odom_.header.stamp.toSec() << std::endl;
    if( ros::Time::now().toSec() - robot_odom_.header.stamp.toSec() >1.0 )
    {
        return false;
    }
    return true;
}

#endif 














