#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

/**
 * @file class_controller.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-01-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <mutex>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Path.h"
#include "tf/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/Twist.h"

#include "class_elemental_pose2d.h"
#include "class_path_segmentation.h"

using std::chrono::high_resolution_clock;

/**
 * @brief
 *
 */
class ClassCarController
{
private:
    bool FLAG_enabled_, FLAG_received_path_, FLAG_path_is_new_, FLAG_path_finished;
    nav_msgs::PathPtr path_msg_ptr_;
    tf::StampedTransform robot_tf_;
    ClassPose2D robot_pose_;
    ClassPath2D path_;
    geometry_msgs::Twist cmd_vel_msg_;
    int current_segment_index_;

    ros::Timer periodic_controller_;
    double controller_interval_;

    ros::NodeHandle nh_;
    tf::TransformListener tf_listener;
    ros::Publisher speed_puber_;

    ros::Subscriber path_suber_;
    ros::Subscriber enable_suber_;

    std::string speed_published_topic_name_;
    std::string path_subscribed_topic_name_;
    std::string enalbe_subscribed_topic_name_;

    std::mutex path_mutex_, enable_mutex_;

    std::string map_frame_, robot_frame_; //, depth_frame_, sonic_frame_;

    void path_callback(const nav_msgs::Path::ConstPtr &msg);
    void enable_callback(const std_msgs::Bool::ConstPtr &msg);
    void get_robot_pose_in_map_frame();
    void controller_update(const ros::TimerEvent &event);

    void stop_moving();

    void load_parameters();

    double mod_2pi(double angle);

    double helper_get_time();

public:
    ClassCarController(const ros::NodeHandle nh_in_);
    ~ClassCarController();
};

/**
 * @brief Construct a new Class Car Controller:: Class Car Controller object
 * 
 * @param nh_in_ 
 */
ClassCarController::ClassCarController(const ros::NodeHandle nh_in_) : nh_{nh_in_}
{
    FLAG_enabled_ = true;
    FLAG_received_path_ = false;
    FLAG_path_is_new_ = false;
    FLAG_path_finished = false;
    current_segment_index_ = 0;

    speed_puber_ = nh_.advertise<geometry_msgs::Twist>(speed_published_topic_name_, 10);

    path_suber_ = nh_.subscribe(path_subscribed_topic_name_, 1, &ClassCarController::path_callback, this);
    enable_suber_ = nh_.subscribe(enalbe_subscribed_topic_name_, 1, &ClassCarController::enable_callback, this);

    periodic_controller_ = nh_.createTimer(ros::Duration(controller_interval_), &ClassCarController::controller_update, this);

    std::cout << "ClassPathPlanner inti Done" << std::endl;
}

/**
 * @brief Destroy the Class Car Controller:: Class Car Controller object
 * 
 */
ClassCarController::~ClassCarController()
{
}

/**
 * @brief 
 * 
 */
void ClassCarController::stop_moving()
{
    cmd_vel_msg_.linear.x = 0;
    cmd_vel_msg_.linear.y = 0;
    cmd_vel_msg_.linear.z = 0;
    cmd_vel_msg_.angular.x = 0;
    cmd_vel_msg_.angular.y = 0;
    cmd_vel_msg_.angular.z = 0;
    speed_puber_.publish(cmd_vel_msg_);
}

/**
 * @brief 
 * 
 * @param event 
 */
void ClassCarController::controller_update(const ros::TimerEvent &event)
{
    enable_mutex_.lock();
    if (!FLAG_enabled_)
    {
        enable_mutex_.unlock();
        return;
    }
    enable_mutex_.unlock();

    path_mutex_.lock();
    if (!FLAG_received_path_)
    {
        path_mutex_.unlock();
        return;
    }

    if (FLAG_path_is_new_)
    {
        FLAG_path_is_new_ = false;
        FLAG_path_finished = false;
        stop_moving();
        ClassPathSegmentation segmenter();
        current_segment_index_ = 0;
    }

    path_mutex_.unlock();

    if (FLAG_path_finished)
    {
        current_segment_index_++;
    }

    if (current_segment_index_ >= path_.size())
    {
        FLAG_path_finished = true;
        stop_moving();
        return;
    }

    // decide the point to follow.

    // control algo.

    speed_puber_.publish(cmd_vel_msg_);
}

/**
 * @brief 
 * 
 */
void ClassCarController::get_robot_pose_in_map_frame()
{
    try
    {
        tf_listener.lookupTransform(map_frame_, robot_frame_, ros::Time(0), robot_tf_);
        robot_pose_.x = robot_tf_.getOrigin().x();
        robot_pose_.y = robot_tf_.getOrigin().y();

        tf::Quaternion q(robot_tf_.getRotation().x(),
                         robot_tf_.getRotation().y(),
                         robot_tf_.getRotation().z(),
                         robot_tf_.getRotation().w());

        tfScalar yaw, pitch, roll;
        tf::Matrix3x3 mat(q);
        mat.getEulerYPR(yaw, pitch, roll);
        robot_pose_.yaw = mod_2pi(yaw);
        // std::cout << start_pose_[0] << " " << start_pose_[1] << " " << start_pose_[2] << std::endl;
        // std::cout << "robot yaw  " << start_pose_[2]*180.0/M_PI << std::endl;
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("\nget_robot_pose_in_map_frame: \n%s", ex.what());
        //   ros::Duration(0.1).sleep();
    }
}


/**
 * @brief 
 * 
 * @param msg 
 */
void ClassCarController::path_callback(const nav_msgs::Path::ConstPtr &msg)
{
    path_mutex_.lock();
    path_msg_ptr_ = msg;
    FLAG_path_is_new_ = true;
    FLAG_received_path_ = true;
    path_mutex_.unlock();
}

/**
 * @brief 
 * 
 * @param msg 
 */
void ClassCarController::enable_callback(const std_msgs::Bool::ConstPtr &msg)
{
    enable_mutex_.lock();
    FLAG_enabled_ = msg->data;
    enable_mutex_.unlock();
}

/**
 * @brief 
 * 
 * @param a 
 * @return double 
 */
double ClassCarController::mod_2pi(double a)
{
    double angle = a;
    while (angle > 2 * M_PI)
        angle -= 2 * M_PI;
    while (angle < 0)
        angle += 2 * M_PI;
    return angle;
}

/**
 * @brief
 * return time in seconds, epoch time.
 */
double ClassCarController::helper_get_time()
{
    return high_resolution_clock::now().time_since_epoch().count() / 1000000000.0;
}

#endif