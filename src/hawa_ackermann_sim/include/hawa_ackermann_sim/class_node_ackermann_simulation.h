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
 * @file class_node_ackermann_simulation.h
 * @author Mingjie
 * @brief This is the ros node of running the simulated robot.
 * @version 0.3
 * @date 2023-11-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef HAWA_CLASS_NODE_ACKERMANN_SIMULATION
#define HAWA_CLASS_NODE_ACKERMANN_SIMULATION

#include <iostream>
#include <vector>
#include <string>
#include <mutex>

// #include <boost/property_tree/ptree.hpp>
// #include <boost/property_tree/json_parser.hpp>

// #include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"
// #include "tf/transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
// #include "ackermann_msgs/AckermannDriveStamped.h"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
// #include "nav_msgs/Odometry.h"
#include "nav_msgs/msg/odometry.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "class_car_body_box_marker.h"
#include "hawa_sim_tools.h"
#include "class_akm_sim_dynamic_states.h"


using namespace std::chrono_literals;
using std::placeholders::_1;

class ClassNodeAckermannSim : public rclcpp::Node
{
private:

    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_bcr_;
    
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr m_akm_drive_suber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odometry_puber_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_car_body_line_puber_;
    rclcpp::TimerBase::SharedPtr m_periodic_timer_;

    std::string m_robot_frame_;
    std::string m_akm_drive_subscribed_topic_name_; 
    std::string m_odometry_published_topic_name_;  
    std::string m_car_body_line_published_topic_name_; 

    ackermann_msgs::msg::AckermannDriveStamped m_akm_drive_msg_;
    double m_akm_drive_msg_stamp_system_;
    nav_msgs::msg::Odometry m_odometry_msg_;

    ClassCarBox2D m_car_box_;

private:
    DynamicsStates m_dyna_states_;
    double m_timestamp_last_update_;
    double m_main_loop_timer_interval_second_;
    bool FLAG_pub_tf__map2odom_;
    
private:
    void akmDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped &msg);
    void mainUpdate( );
    void loadParameters();

public:
    ClassNodeAckermannSim();
    ~ClassNodeAckermannSim();
};


// class ClassNodeAckermannSim
// {
// private:
//     ros::NodeHandle m_nodehandle_;
//     tf::TransformBroadcaster m_tf_bcr_;

//     ros::Subscriber m_akm_drive_suber_ ;
//     ros::Publisher m_odometry_puber_;
//     ros::Publisher m_car_body_line_puber_;
//     ros::Timer  m_periodic_timer_;

//     std::string m_robot_frame_;
//     std::string m_akm_drive_subscribed_topic_name_; 
//     std::string m_odometry_published_topic_name_;  
//     std::string m_car_body_line_published_topic_name_; 

//     ackermann_msgs::AckermannDriveStamped m_akm_drive_msg_;
//     double m_akm_drive_msg_stamp_system_;
//     nav_msgs::Odometry m_odometry_msg_;

//     ClassCarBox2D m_car_box_;

// private:
//     DynamicsStates m_dyna_states_;
//     double m_timestamp_last_update_;
//     double m_main_loop_timer_interval_second_;
//     bool FLAG_pub_tf__map2odom_;
    
// private:
//     void akmDriveCallback(const ackermann_msgs::AckermannDriveStampedConstPtr &msg);
//     void mainUpdate( const ros::TimerEvent &event );
//     void loadParameters();
    
// public:
//     ClassNodeAckermannSim(const ros::NodeHandle nh_in_);
//     ~ClassNodeAckermannSim();
// };


ClassNodeAckermannSim::ClassNodeAckermannSim(): Node("AckermannSim")
{
    loadParameters(); 

    m_akm_drive_msg_stamp_system_ = 0.0;

    m_tf_bcr_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    m_akm_drive_suber_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        m_akm_drive_subscribed_topic_name_, 10, std::bind(&ClassNodeAckermannSim::akmDriveCallback, this, _1)
    );

    m_odometry_puber_ = this->create_publisher<nav_msgs::msg::Odometry>(
        m_odometry_published_topic_name_, 10
    );

    m_car_body_line_puber_ = this->create_publisher<visualization_msgs::msg::Marker>(
        m_car_body_line_published_topic_name_, 10
    );

    m_periodic_timer_ = this->create_wall_timer(20ms, std::bind(&ClassNodeAckermannSim::mainUpdate, this));

    RCLCPP_INFO(this->get_logger(), "ClassNodeAckermannSim inti Done");

    // m_akm_drive_suber_ = m_nodehandle_.subscribe(m_akm_drive_subscribed_topic_name_, 
    //                                              1, &ClassNodeAckermannSim::akmDriveCallback, this);
    
    // m_odometry_puber_ = m_nodehandle_.advertise<nav_msgs::Odometry>(
    //                                                 m_odometry_published_topic_name_, 1, this);
    
    // m_car_body_line_puber_ = m_nodehandle_.advertise<visualization_msgs::Marker>(
    //                                                 m_car_body_line_published_topic_name_, 1, this);

    // m_periodic_timer_ = m_nodehandle_.createTimer(ros::Duration(m_main_loop_timer_interval_second_), 
    //                                               &ClassNodeAckermannSim::mainUpdate, this );

    // ROS_INFO_STREAM("ClassNodeAckermannSim inti Done");
}

ClassNodeAckermannSim::~ClassNodeAckermannSim()
{
}

/**
 * @brief Set the ros topic names and tf frames. In the future, these parameters will be loaded from 
 * a json file, so it will be more reconfigurable. 
*/
void ClassNodeAckermannSim::loadParameters()
{
    m_akm_drive_subscribed_topic_name_ = "/ackermann_cmd";
    m_odometry_published_topic_name_ = "/odometry";
    m_car_body_line_published_topic_name_ = "/car_body_line_marker";
    
    m_robot_frame_ = "/base_link";

    m_main_loop_timer_interval_second_ = 0.02;
    FLAG_pub_tf__map2odom_ = true; // set to true if there's no other source provides the tf bwtn map and odom.

    m_car_box_.setFrameID(m_robot_frame_);
}

/**
 * @brief Callback function to the ackermann drive message input.
*/
void ClassNodeAckermannSim::akmDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped &msg)
{
    RCLCPP_INFO(this->get_logger(), "akmDriveCallback()");
    m_akm_drive_msg_.header = msg.header;
    m_akm_drive_msg_.drive = msg.drive;
    m_akm_drive_msg_stamp_system_ = getTimeSecond();
}

/**
 * @brief The main function in class. To be called by the ros timer event. 
*/
void ClassNodeAckermannSim::mainUpdate()
{
    double _time_now_second = getTimeSecond();

    double _age_of_akm_drive_msg = _time_now_second - m_akm_drive_msg_stamp_system_ ;

    // stop the robot when the command input is out of date. 
    if (_age_of_akm_drive_msg < 0.3)
    {
        m_dyna_states_.setTargetLinearVbMps(m_akm_drive_msg_.drive.speed);
        m_dyna_states_.setTargetSteerRad(m_akm_drive_msg_.drive.steering_angle);
    }
    else
    {
        m_dyna_states_.setTargetLinearVbMps(0);
        m_dyna_states_.setTargetSteerRad(0);
    }

    double _delta_t = _time_now_second - m_timestamp_last_update_;

    m_dyna_states_.updateAllStates(_delta_t);

    m_timestamp_last_update_ = _time_now_second;

    geometry_msgs::msg::TransformStamped _transform;

    // tf::Transform _transform;
    // tf::Quaternion _tf_qua;

    

    _transform.header.stamp = this->get_clock()->now();
   

    if (FLAG_pub_tf__map2odom_)
    {
        _transform.header.frame_id = "map";
        _transform.child_frame_id = "odom";
        _transform.transform.translation.x = 0;
        _transform.transform.translation.y = 0;
        _transform.transform.translation.z = 0;
        _transform.transform.rotation.x = 0;
        _transform.transform.rotation.y = 0;
        _transform.transform.rotation.z = 0;
        _transform.transform.rotation.w = 1;
        m_tf_bcr_->sendTransform(_transform);
        // _transform.setOrigin( tf::Vector3(0, 0, 0) );
        // _tf_qua.setRPY(0, 0, 0);
        // _transform.setRotation(_tf_qua);
        // m_tf_bcr_.sendTransform(tf::StampedTransform(_transform, ros::Time::now(), "map", "odom"));
    }
    
    _transform.header.frame_id = "odom";
    _transform.child_frame_id = "base_link";
    _transform.transform.translation.x = m_dyna_states_.m_x_meter_;
    _transform.transform.translation.y = m_dyna_states_.m_y_meter_;
    _transform.transform.translation.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, m_dyna_states_.m_yaw_rad_);
    _transform.transform.rotation.x = q.x();
    _transform.transform.rotation.y = q.y();
    _transform.transform.rotation.z = q.z();
    _transform.transform.rotation.w = q.w();
    m_tf_bcr_->sendTransform(_transform);

    _transform.header.frame_id = "base_link";
    _transform.child_frame_id = "steer_link";
    _transform.transform.translation.x = m_dyna_states_.m_axle_distance_;
    _transform.transform.translation.y = 0;
    _transform.transform.translation.z = 0;

    q.setRPY(0, 0, m_dyna_states_.m_steer_rad_actual_);
    _transform.transform.rotation.x = q.x();
    _transform.transform.rotation.y = q.y();
    _transform.transform.rotation.z = q.z();
    _transform.transform.rotation.w = q.w();
    m_tf_bcr_->sendTransform(_transform);

    // _transform.setOrigin( tf::Vector3(m_dyna_states_.m_x_meter_, m_dyna_states_.m_y_meter_, 0.0) );
    // _tf_qua.setRPY(0, 0, m_dyna_states_.m_yaw_rad_);
    // _transform.setRotation(_tf_qua);
    // m_tf_bcr_.sendTransform(tf::StampedTransform(_transform, ros::Time::now(), "odom", "base_link"));

    // // publish a link that represents the front wheel (the one steering left and right)
    // _transform.setOrigin( tf::Vector3(m_dyna_states_.m_axle_distance_, 0.0, 0.0) );
    // _tf_qua.setRPY(0, 0, m_dyna_states_.m_steer_rad_actual_);
    // _transform.setRotation(_tf_qua);
    // // m_tf_bcr_.sendTransform(tf::StampedTransform(_transform, ros::Time::now(), "base_link", "steer_link"));


    visualization_msgs::msg::Marker _body_line_marker = m_car_box_.getMarkerMsg();
    m_car_body_line_puber_->publish(_body_line_marker);

    m_odometry_msg_.header.stamp = this->get_clock()->now();
    m_odometry_msg_.pose.pose.position.x = m_dyna_states_.m_x_meter_;
    m_odometry_msg_.pose.pose.position.y = m_dyna_states_.m_y_meter_;
    // _transform.setOrigin(tf::Vector3(0, 0, 0));
    q.setRPY(0, 0, m_dyna_states_.m_yaw_rad_);
    m_odometry_msg_.pose.pose.orientation.x = q.getX();
    m_odometry_msg_.pose.pose.orientation.y = q.getY();
    m_odometry_msg_.pose.pose.orientation.z = q.getZ();
    m_odometry_msg_.pose.pose.orientation.w = q.getW();
    
    m_odometry_msg_.twist.twist.linear.x = m_dyna_states_.m_linear_vb_mps_actual_;
    m_odometry_msg_.twist.twist.linear.y = 0;
    m_odometry_msg_.twist.twist.angular.z = m_dyna_states_.m_angular_wb_radps_actual_;
    
    m_odometry_puber_->publish(m_odometry_msg_);

    
}



#endif
