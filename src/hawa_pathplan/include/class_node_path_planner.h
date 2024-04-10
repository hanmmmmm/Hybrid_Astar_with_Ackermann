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
 * @file class_path_planner.h
 * @author Mingjie
 * @brief This is the ros node that is the highest level of the path planning module in this system.  
 * @version 0.2
 * @date 2023-11-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#ifndef HAWA_CLASS_PATH_PLANNER
#define HAWA_CLASS_PATH_PLANNER

#include "common_includes.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"

#include "class_hybrid_astar.h"
#include "utils/class_utils__converters.h"
#include "custom_data_types.h"
#include "utils/class_utils__timestamp.h"
#include "class_path_validator.h"
#include "reedsshepp_tools.h"

// TODO:
// add a function to check if the current path is still valid:
// - robot position xy is close to it
// - robot heading is close to the path
// - path is clear from obstacles
// - 
// if it is no longer valid, then it will search new path. 



namespace hawa
{

using namespace std::chrono_literals;

class ClassPathPlanner : public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr m_periodic_timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_publisher_path_ ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_publisher_searching_ ;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_subscriber_map_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_subscriber_goal_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_subscriber_odom_;
        
    void pathPlan( );
    void mapCallback(const nav_msgs::msg::OccupancyGrid &msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped &msg);
    void odomCallback(const nav_msgs::msg::Odometry &msg);

    std::string m_topic_name_path_published_;
    std::string m_topic_name_searching_published_;
    std::string m_topic_name_map_subscribed_; 
    std::string m_topic_name_goal_subscribed_;
    std::string m_topic_name_odom_subscribed_;

    nav_msgs::msg::Path m_navmsgs_path_msg_;
    nav_msgs::msg::OccupancyGrid m_map_msg_;
    nav_msgs::msg::Odometry m_odom_msg_;

    std::mutex m_map_mutex_;
    std::mutex m_goal_mutex_;
    std::mutex m_odom_mutex_;

    std::string m_map_tf_frame_name_; 
    std::string m_robot_tf_frame_name_;

    bool m_map_received_;
    bool m_goal_received_;
    bool m_goal_solved_;

    // ClassHawaTimeStamp m_goal_stamp_;
    // double m_goal_receive_timestamp_;
    
    geometry_msgs::msg::TransformStamped m_tf_robot_to_map_;
    void getRobotPoseInMapFrame();

    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_;

    std::unique_ptr<ClassHybridAStar> m_ha_planner_;
    std::unique_ptr<ClassPathValidator> m_path_validator_;

    StructPoseReal m_start_pose_;
    StructPoseReal m_goal_pose_;

    StructSamplingProperties m_rs_sampling_properites_;

    int m_path_plan_timeout_ms_;
    double m_robot_linear_velocity_;

    void loadParameters();

    void exit_pathplan_function(const double time_start);

    void ros_info(const std::string& str);

    void checkPath();

    bool checkRobotStop();

private:

    bool FLAG_wait_before_replan_;
    double m_rs_curve_radius_metric_;

public:
    ClassPathPlanner();
    ~ClassPathPlanner(){};
};


} // namespace hawa

#endif