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

#ifndef CLASS_NODE_PURE_PURSUIT
#define CLASS_NODE_PURE_PURSUIT


#include <iostream>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/msg/marker.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include "class_purepursuit.h"

#include "../common/class_elemental_path2d_segment.h"
#include "../common/tools_angles.h"
#include "../common/class_multi_segment_manager.h"


namespace hawa
{

using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * @brief This class is made for utilizing the ClassPurePursuit, and let it being interactive 
 * with ROS.
*/
class ClassNodePurePursuit : public rclcpp::Node
{
    
private:
    ClassPurePursuit m_controller_purepursuit_;

    ClassPath2DSegment m_current_path_segment_;

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
    double m_controller_interval_;

    double m_expected_speed_mps_, m_look_ahead_ratio_;

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

private:
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

    void ros_info(const std::string& str);
    void ros_warn(const std::string& str, const int t=5);

public:
    ClassNodePurePursuit();
    ~ClassNodePurePursuit();
};

} // namespace hawa



#endif 

