
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

#include "pure_pursuit/class_node_pure_pursuit.h"

namespace hawa
{   

ClassNodePurePursuit::ClassNodePurePursuit() : Node("NodePurePursuit")
{
    m_topic_name__target_point_marker_ = "/target_point_marker";
    m_topic_name__ackerman_msg_ = "/ackermann_cmd";
    m_topic_name__pause_for_searching_subscribed_ = "/planner_searching";

    m_topic_name__odom_subscribed_ = "/odometry";
    m_topic_name__path_subscribed_ = "/path";
    m_topic_name__current_path_published_ = "/current_segment";

    m_topic_name__speed_mps_subscribed_ = "/expected_speed_mps";
    m_topic_name__look_coefficient_subscribed_ = "/look_ahead_ratio";

    m_controller_interval_ = 0.1;

    m_expected_speed_mps_ = 0.6;
    m_look_ahead_ratio_ = 0.9;

    m_pause_for_searching_.data = false;

    m_robot_odom_msg_.header.stamp = this->get_clock()->now();

    m_puber__target_point_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
        m_topic_name__target_point_marker_, 10
    );

    m_puber__ackerman_msg_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        m_topic_name__ackerman_msg_, 10
    );

    m_puber__current_path_ = this->create_publisher<nav_msgs::msg::Path>(
        m_topic_name__current_path_published_, 10
    );

    m_suber__path_ = this->create_subscription<nav_msgs::msg::Path>(
        m_topic_name__path_subscribed_, 10, std::bind(&ClassNodePurePursuit::pathCallback, this, _1)
    );

    m_suber__odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        m_topic_name__odom_subscribed_, 10, std::bind(&ClassNodePurePursuit::odomCallback, this, _1)
    );

    m_suber__pause_for_searching_ = this->create_subscription<std_msgs::msg::Bool>(
        m_topic_name__pause_for_searching_subscribed_, 10, std::bind(&ClassNodePurePursuit::pauseCallback, this, _1)
    );

    m_suber__speed_mps_ = this->create_subscription<std_msgs::msg::Float64>(
        m_topic_name__speed_mps_subscribed_, 10, std::bind(&ClassNodePurePursuit::speedCallback, this, _1)
    );

    m_suber__look_coefficient_ = this->create_subscription<std_msgs::msg::Float64>(
        m_topic_name__look_coefficient_subscribed_, 10, std::bind(&ClassNodePurePursuit::ratioCallback, this, _1)
    );

    m_periodic_controller_ = this->create_wall_timer(100ms, std::bind(&ClassNodePurePursuit::controllerUpdate, this));

    ros_info("ClassNodePurePursuit inti Done");

    return;
}


ClassNodePurePursuit::~ClassNodePurePursuit()
{
}


void ClassNodePurePursuit::ros_info(const std::string& str)
{
    RCLCPP_INFO(this->get_logger(), str.c_str());
}


void ClassNodePurePursuit::ros_warn(const std::string& str, const int t)
{
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), 
                                *(this->get_clock()), 
                                std::chrono::seconds(t).count(), 
                                str.c_str());
}


/**
 * @brief callback when receiving path msgs. Pass it to the path manager module.
 * @param msg ros topic msg.
*/
void ClassNodePurePursuit::pathCallback(const nav_msgs::msg::Path &msg)
{
    if(msg.poses.size() < 2)
    {
        // RCLCPP_INFO(this->get_logger(), "ClassNodePurePursuit::pathCallback()", "Path msg size < 2.  Do not proceed.");
        ros_info("ClassNodePurePursuit::pathCallback(), Path msg size < 2.  Do not proceed.");
        return;
    }
    m_mutex_path_.lock();
    m_segment_manager_.setPath(msg);
    m_mutex_path_.unlock();
}

/**
 * @brief For updating the value of the robot pose.
 * @param msg ros topic msg.
*/
void ClassNodePurePursuit::odomCallback(const nav_msgs::msg::Odometry &msg)
{
    m_robot_odom_msg_.child_frame_id = msg.child_frame_id;
    m_robot_odom_msg_.header = msg.header;
    m_robot_odom_msg_.pose = msg.pose;
    m_robot_odom_msg_.twist = msg.twist;
}

/**
 * @brief For updating the value of the target speed in mps
 * @param msg ros topic msg.
*/
void ClassNodePurePursuit::speedCallback(const std_msgs::msg::Float64 &msg)
{
    m_expected_speed_mps_ = msg.data;
}

/**
 * @brief For updating the value of the m_look_ahead_ratio_ by ros topic.
 * @param msg ros topic msg.
*/
void ClassNodePurePursuit::ratioCallback(const std_msgs::msg::Float64 &msg)
{
    m_look_ahead_ratio_ = msg.data;
}

/**
 * @brief For updating the value of the m_pause_for_searching_ by ros topic.
*/
void ClassNodePurePursuit::pauseCallback(const std_msgs::msg::Bool &msg)
{
    m_pause_for_searching_ = msg;
}

/**
 * @brief Main function to be called by timer regualerly.
 * @param event ros timer event.
*/
void ClassNodePurePursuit::controllerUpdate()
{
    if (m_pause_for_searching_.data)
    {
        ros_warn("Pause while searching path.");
        return;
    }

    if( ! validateOdomMsg() )
    {
        // RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *(this->get_clock()), std::chrono::seconds(5).count(), "Invalid robot odom msg.  Do not proceed.");
        ros_warn("Invalid robot odom msg.  Do not proceed.");
        return;
    }

    m_mutex_path_.lock();

    // ROS_DEBUG_STREAM_THROTTLE(10 , "m_counter_current_segment_ " << m_segment_manager_.getCounter());

    m_segment_manager_.update(m_robot_odom_msg_.pose.pose.position.x, 
                              m_robot_odom_msg_.pose.pose.position.y,
                              Tools::quaternionToEularYaw(m_robot_odom_msg_.pose.pose.orientation));
    if (m_segment_manager_.doesPathExist())
    {
        m_current_path_segment_ = m_segment_manager_.getCurrentSegment();
        m_controller_purepursuit_.setPathSegment(m_current_path_segment_);
        nav_msgs::msg::Path _ros_current_path = m_segment_manager_.getCurrentSegment().toRosPath();
        _ros_current_path.header = m_segment_manager_.getOriginalPath().header;
        m_puber__current_path_->publish(_ros_current_path);
    }

    
    double _signed_speed_mps = decideSpeedMps();
    // the values for the parameters of the controller algorithm.
    m_controller_purepursuit_.setLookAheadCoefficient(m_look_ahead_ratio_);
    m_controller_purepursuit_.setTargetLinearSpeedMps(_signed_speed_mps);
    m_controller_purepursuit_.setVehicleWheelbaseMeter(0.25);
    m_controller_purepursuit_.setRobotPose(m_robot_odom_msg_.pose.pose.position.x, 
                                  m_robot_odom_msg_.pose.pose.position.y,
                                  Tools::quaternionToEularYaw(m_robot_odom_msg_.pose.pose.orientation));

    bool _valid_target_point = false;
    m_controller_purepursuit_.findTargetPoint(_valid_target_point);
    if( ! _valid_target_point )
    {
        // ros_warn("Invalid target point.  Do not proceed.", 10);
        m_mutex_path_.unlock();
        return;
    }

    visulizeTargetPoint();    

    bool _success;  double _steer_needed = 0.0;
    m_controller_purepursuit_.solveForSpeedCommand(_success, _steer_needed);

    ackermann_msgs::msg::AckermannDriveStamped  _akm_cmd_msg;
    _akm_cmd_msg.drive.speed = _signed_speed_mps;
    _akm_cmd_msg.drive.steering_angle = _steer_needed;
    _akm_cmd_msg.header.stamp = this->get_clock()->now();

    m_puber__ackerman_msg_->publish(_akm_cmd_msg);

    m_mutex_path_.unlock();
}

/**
 * @brief Check if the robot odom msg is up to date. 
 * @return true when valid;  false when invalid.
*/
bool ClassNodePurePursuit::validateOdomMsg()
{
    // auto t_now = this->get_clock()->now().nanoseconds()/1000000000.;
    // auto t_msg = m_robot_odom_msg_.header.stamp.nanosec/1000000000.;
    auto t_now = this->get_clock()->now().seconds();
    auto t_msg = m_robot_odom_msg_.header.stamp.sec;
    if( t_now - t_msg > 1.0 )
    {
        return false;
    }
    return true;
    
    // if( ros::Time::now().toSec() - m_robot_odom_msg_.header.stamp.toSec() >1.0 )
    // {
    //     return false;
    // }
    // return true;
}

/**
 * @brief When robot is approaching the end of the segment, the speed should reduce for a smoother
 * transition. This function calculates the speed based on the distance between the robot and the 
 * end of the segment. 
 * @return the speed in unit of meter per second.
*/
double ClassNodePurePursuit::calcSpeedRampping()
{
    double _dist_to_end = m_segment_manager_.getDistToEnd();
    double _dist_threshold = 1.0; // meter
    double _min_speed_mps = 0.4; // meter per second, this the mininum value of the output. 
    if (_dist_to_end >= _dist_threshold)
    {
        return m_expected_speed_mps_;
    }
    double _slope = (m_expected_speed_mps_ - _min_speed_mps) / _dist_threshold;

    return _slope * _dist_to_end + _min_speed_mps;
}

/**
 * @brief This function is for calculating the speed that the purepursuit algo should use, and the speed
 * value in the control message that will be sent out.
 * @return The speed is double type and is signed. Positive value means forward motion and negative value
 * means reversing motion. 
*/
double ClassNodePurePursuit::decideSpeedMps()
{
    double _spd_val = 0;
    if (m_current_path_segment_.isForward())
    {
        // _spd_val = std::abs(m_expected_speed_mps_);
        _spd_val = std::abs(calcSpeedRampping());
    }
    else
    {
        // _spd_val = - std::abs(m_expected_speed_mps_);
        _spd_val = - std::abs(calcSpeedRampping());
    }

    if (m_segment_manager_.didFinishAll())
    {
        _spd_val = 0;
    }
    return _spd_val;
}

/**
 * @brief Get the target point position, and visulize it in RVIZ as a sphere. 
*/
void ClassNodePurePursuit::visulizeTargetPoint()
{
    geometry_msgs::msg::PointStamped _target_point;
    m_controller_purepursuit_.getTargetPoint(_target_point);

    visualization_msgs::msg::Marker _target_point_marker;
    _target_point_marker.pose.position.x = _target_point.point.x;
    _target_point_marker.pose.position.y = _target_point.point.y;
    _target_point_marker.header.frame_id = "map";
    _target_point_marker.type = visualization_msgs::msg::Marker::SPHERE;
    _target_point_marker.action = visualization_msgs::msg::Marker::MODIFY;

    _target_point_marker.scale.x = 0.1;
    _target_point_marker.scale.y = 0.1;
    _target_point_marker.scale.z = 0.1;

    _target_point_marker.color.a = 0.9; // Don't forget to set the alpha!
    _target_point_marker.color.r = 1.0;
    _target_point_marker.color.g = 0.0;
    _target_point_marker.color.b = 0.0;
    m_puber__target_point_marker_->publish(_target_point_marker);
}


} // namespace hawa
