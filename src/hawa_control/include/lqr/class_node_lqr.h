
#ifndef CLASS_NODE_LQR_H_
#define CLASS_NODE_LQR_H_

#include <iostream>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/msg/marker.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "class_lqr_controller.h"
#include "class_lqr_ref_point_selector.h"

#include "../common/class_elemental_path2d_segment.h"
#include "../common/tools_angles.h"
#include "../common/class_multi_segment_manager.h"



using namespace std::chrono_literals;
using std::placeholders::_1;


class ClassNodeLQR : public rclcpp::Node
{
private:

    std::unique_ptr<ClassLQRController> m_lqr_controller_ = nullptr;
    ClassLqrRefPointSelector m_ref_point_selector_;

    double m_dt_;

    std::shared_ptr<ClassPath2DSegment> m_current_path_segment_;

    ClassHawaMultiSegmentManager m_segment_manager_;

    nav_msgs::msg::Odometry m_robot_odom_msg_;

    std_msgs::msg::Bool m_pause_for_searching_;

    std::mutex m_mutex_path_;

    // For algorithm
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_suber__path_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_suber__odom_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_suber__pause_for_searching_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr m_puber__ackerman_msg_;

    std::string m_topic_name__path_subscribed_;
    std::string m_topic_name__odom_subscribed_;
    std::string m_topic_name__ackerman_msg_;
    rclcpp::TimerBase::SharedPtr m_periodic_controller_;

    double m_expected_speed_mps_;
    // double m_look_ahead_ratio_;

    // For human interactivity only
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_puber__target_point_marker_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_puber__current_path_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr m_suber__speed_mps_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr m_suber__look_coefficient_;
    std::string m_topic_name__target_point_marker_;
    std::string m_topic_name__speed_mps_subscribed_;
    std::string m_topic_name__look_coefficient_subscribed_;
    std::string m_topic_name__pause_for_searching_subscribed_;
    std::string m_topic_name__current_path_published_;


public:
    
    ClassNodeLQR();

    void pathCallback(const nav_msgs::msg::Path &msg);

    void odomCallback(const nav_msgs::msg::Odometry &msg);

    void speedCallback(const std_msgs::msg::Float64 &msg);

    void ratioCallback(const std_msgs::msg::Float64 &msg);

    void pauseCallback(const std_msgs::msg::Bool &msg);

    void controllerUpdate();

    bool validateOdomMsg();

    double calcSpeedRampping();

    double decideSpeedMps();

    void visulizeTargetPoint();
    void visulizeOnePoint(const ClassPose2D& pt);

    void ros_info(const std::string& str);
    void ros_warn(const std::string& str, const int t=5);


};


ClassNodeLQR::ClassNodeLQR() : Node("node_LQR")
{
    m_dt_ = 0.1;

    m_lqr_controller_ = std::make_unique<ClassLQRController>(0.25, m_dt_);
    m_current_path_segment_ = std::make_shared<ClassPath2DSegment>();

    m_topic_name__target_point_marker_ = "/target_point_marker";
    m_topic_name__ackerman_msg_ = "/ackermann_cmd";
    m_topic_name__pause_for_searching_subscribed_ = "/planner_searching";

    m_topic_name__odom_subscribed_ = "/odometry";
    m_topic_name__path_subscribed_ = "/path";
    m_topic_name__current_path_published_ = "/current_segment";

    // m_topic_name__speed_mps_subscribed_ = "/expected_speed_mps";
    // m_topic_name__look_coefficient_subscribed_ = "/look_ahead_ratio";

    m_expected_speed_mps_ = 0.65;
    // m_look_ahead_ratio_ = 0.9;

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
        m_topic_name__path_subscribed_, 10, std::bind(&ClassNodeLQR::pathCallback, this, _1)
    );

    m_suber__odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        m_topic_name__odom_subscribed_, 10, std::bind(&ClassNodeLQR::odomCallback, this, _1)
    );

    m_suber__pause_for_searching_ = this->create_subscription<std_msgs::msg::Bool>(
        m_topic_name__pause_for_searching_subscribed_, 10, std::bind(&ClassNodeLQR::pauseCallback, this, _1)
    );

    // m_suber__speed_mps_ = this->create_subscription<std_msgs::msg::Float64>(
    //     m_topic_name__speed_mps_subscribed_, 10, std::bind(&ClassNodeLQR::speedCallback, this, _1)
    // );

    // m_suber__look_coefficient_ = this->create_subscription<std_msgs::msg::Float64>(
    //     m_topic_name__look_coefficient_subscribed_, 10, std::bind(&ClassNodeLQR::ratioCallback, this, _1)
    // );

    m_periodic_controller_ = this->create_wall_timer(100ms, std::bind(&ClassNodeLQR::controllerUpdate, this));

    ros_info("ClassNodeLQR inti Done");
    std::cout << std::setprecision(2) << std::fixed << std::endl;

    return;
}





void ClassNodeLQR::ros_info(const std::string& str)
{
    RCLCPP_INFO(this->get_logger(), str.c_str());
}


void ClassNodeLQR::ros_warn(const std::string& str, const int t)
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
void ClassNodeLQR::pathCallback(const nav_msgs::msg::Path &msg)
{
    if(msg.poses.size() < 2)
    {
        ros_info("ClassNodeLQR::pathCallback(), Path msg size < 2.  Do not proceed.");
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
void ClassNodeLQR::odomCallback(const nav_msgs::msg::Odometry &msg)
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
void ClassNodeLQR::speedCallback(const std_msgs::msg::Float64 &msg)
{
    m_expected_speed_mps_ = msg.data;
}

/**
 * @brief For updating the value of the m_pause_for_searching_ by ros topic.
*/
void ClassNodeLQR::pauseCallback(const std_msgs::msg::Bool &msg)
{
    m_pause_for_searching_ = msg;
}




/**
 * @brief Main function to be called by timer regualerly.
 * @param event ros timer event.
*/
void ClassNodeLQR::controllerUpdate()
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
                              quaternionToEularYaw(m_robot_odom_msg_.pose.pose.orientation));
    if (m_segment_manager_.doesPathExist())
    {
        auto temp = m_segment_manager_.getCurrentSegment();
        m_current_path_segment_ = std::make_shared<ClassPath2DSegment>(temp);
        m_ref_point_selector_.setPathPtr(m_current_path_segment_);
        nav_msgs::msg::Path _ros_current_path = m_segment_manager_.getCurrentSegment().toRosPath();
        _ros_current_path.header = m_segment_manager_.getOriginalPath().header;
        m_puber__current_path_->publish(_ros_current_path);
    }

    m_ref_point_selector_.setGoingForward(m_current_path_segment_->isForward());

    ClassPose2D _robot_pose(m_robot_odom_msg_.pose.pose.position.x, 
                            m_robot_odom_msg_.pose.pose.position.y, 
                            quaternionToEularYaw(m_robot_odom_msg_.pose.pose.orientation));

    m_ref_point_selector_.setRobotPose(_robot_pose);
    m_ref_point_selector_.process();
    
    double _signed_speed_mps = decideSpeedMps();

    // the values for the parameters of the controller algorithm.
    
    m_lqr_controller_->setVref(_signed_speed_mps);

    m_lqr_controller_->setSteerRef(0);

    auto _ref_pose = m_ref_point_selector_.getRefPose();
    m_lqr_controller_->setStateRef(_ref_pose); 
    m_lqr_controller_->setState(_robot_pose);
    
    m_lqr_controller_->generateQ(3, 3, 1);
    m_lqr_controller_->generateR(1, 2);
    m_lqr_controller_->solve();
    auto steer = m_lqr_controller_->getSteer();
    auto speed = m_lqr_controller_->getV();

    steer = std::min(steer, 1.0);
    steer = std::max(steer, -1.0);

    speed = std::min(speed, m_expected_speed_mps_);
    speed = std::max(speed, -m_expected_speed_mps_);

    visulizeOnePoint(_ref_pose);

    ackermann_msgs::msg::AckermannDriveStamped  _akm_cmd_msg;
    _akm_cmd_msg.drive.speed = speed;
    _akm_cmd_msg.drive.steering_angle = steer;
    _akm_cmd_msg.header.stamp = this->get_clock()->now();

    m_puber__ackerman_msg_->publish(_akm_cmd_msg);

    // std::cout << "CMD: speed: " << _akm_cmd_msg.drive.speed << "   steer: " 
    // << _akm_cmd_msg.drive.steering_angle << std::endl << std::endl;

    m_mutex_path_.unlock();
}

/**
 * @brief Check if the robot odom msg is up to date. 
 * @return true when valid;  false when invalid.
*/
bool ClassNodeLQR::validateOdomMsg()
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
double ClassNodeLQR::calcSpeedRampping()
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
double ClassNodeLQR::decideSpeedMps()
{
    double _spd_val = 0;
    if (m_current_path_segment_->isForward())
    {
        _spd_val = std::abs(calcSpeedRampping());
    }
    else
    {
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
void ClassNodeLQR::visulizeTargetPoint()
{
    auto pt = m_ref_point_selector_.getRefPose();
    
    visualization_msgs::msg::Marker _target_point_marker;
    _target_point_marker.header.frame_id = "map";
    _target_point_marker.action = visualization_msgs::msg::Marker::MODIFY;

    {
        // // draw a sphere
        // _target_point_marker.pose.position.x = pt.x;
        // _target_point_marker.pose.position.y = pt.y;
        // _target_point_marker.type = visualization_msgs::msg::Marker::SPHERE;
        // _target_point_marker.scale.x = 0.1;
        // _target_point_marker.scale.y = 0.1;
        // _target_point_marker.scale.z = 0.1;
    }
    
    {
        // draw an arrow
        geometry_msgs::msg::Point _start_point;
        _start_point.x = pt.x;
        _start_point.y = pt.y;
        _target_point_marker.points.push_back(std::move(_start_point));

        geometry_msgs::msg::Point _end_point;
        _end_point.x = pt.x + 0.5 * std::cos(pt.yaw);
        _end_point.y = pt.y + 0.5 * std::sin(pt.yaw);
        _target_point_marker.points.push_back(std::move(_end_point));

        _target_point_marker.type = visualization_msgs::msg::Marker::ARROW;
        _target_point_marker.scale.x = 0.05;
        _target_point_marker.scale.y = 0.1;
        _target_point_marker.scale.z = 0.1;
    }

    _target_point_marker.color.a = 0.7; // Don't forget to set the alpha!
    _target_point_marker.color.r = 1.0;
    _target_point_marker.color.g = 0.0;
    _target_point_marker.color.b = 0.0;
    m_puber__target_point_marker_->publish(_target_point_marker);
}

void ClassNodeLQR::visulizeOnePoint(const ClassPose2D& pt)
{
    visualization_msgs::msg::Marker _target_point_marker;
    _target_point_marker.header.frame_id = "map";
    _target_point_marker.action = visualization_msgs::msg::Marker::MODIFY;

    {
        // // draw a sphere
        // _target_point_marker.pose.position.x = pt.x;
        // _target_point_marker.pose.position.y = pt.y;
        // _target_point_marker.type = visualization_msgs::msg::Marker::SPHERE;
        // _target_point_marker.scale.x = 0.1;
        // _target_point_marker.scale.y = 0.1;
        // _target_point_marker.scale.z = 0.1;
    }
    
    {
        // draw an arrow
        geometry_msgs::msg::Point _start_point;
        _start_point.x = pt.x;
        _start_point.y = pt.y;
        _target_point_marker.points.push_back(std::move(_start_point));

        geometry_msgs::msg::Point _end_point;
        _end_point.x = pt.x + 0.5 * std::cos(pt.yaw);
        _end_point.y = pt.y + 0.5 * std::sin(pt.yaw);
        _target_point_marker.points.push_back(std::move(_end_point));

        _target_point_marker.scale.x = 0.1;
        _target_point_marker.scale.y = 0.1;
        _target_point_marker.scale.z = 0.1;
    }

    _target_point_marker.color.a = 0.7; // Don't forget to set the alpha!
    _target_point_marker.color.r = 0.0;
    _target_point_marker.color.g = 0.8;
    _target_point_marker.color.b = 0.5;
    m_puber__target_point_marker_->publish(_target_point_marker);
}



#endif