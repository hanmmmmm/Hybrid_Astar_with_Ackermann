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

#include <iostream>
#include <vector>
#include <string>

#include <mutex>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"

#include "../hybrid_a_star_module/class_hybrid_astar.h"
#include "../utils/hawa_conversion_tools.h"
#include "../utils/hawa_tools.h"
#include "../utils/hawa_data_containers.h"
#include "../utils/hawa_timestamp.h"
#include "class_path_validator.h"


// TODO:
// add a function to check if the current path is still valid:
// - robot position xy is close to it
// - robot heading is close to the path
// - path is clear from obstacles
// - 
// if it is no longer valid, then it will search new path. 


using namespace std::chrono_literals;
using std::placeholders::_1;



class ClassPathPlanner : public rclcpp::Node
{
private:

    ClassHybridAStar m_ha_planner_;
    StructPoseReal m_start_pose_;
    StructPoseReal m_goal_pose_;

    int m_path_plan_timeout_ms_;

    // ros::NodeHandle m_ros_nh_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_publisher_path_ ;
    rclcpp::TimerBase::SharedPtr m_periodic_timer_;
    std::string m_topic_name_path_published_;
    nav_msgs::msg::Path m_navmsgs_path_msg_;
    // double m_planer_interval_;
    void pathPlan( );

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_subscriber_map_;
    std::string m_topic_name_map_subscribed_; 
    nav_msgs::msg::OccupancyGrid m_map_msg_;
    bool m_map_received_;
    std::mutex m_map_mutex_;
    std::string m_map_tf_frame_name_; 
    void mapCallback(const nav_msgs::msg::OccupancyGrid &msg);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_subscriber_goal_;
    std::string m_topic_name_goal_subscribed_;
    bool m_goal_received_;
    bool m_goal_solved_;
    ClassHawaTimeStamp m_goal_stamp_;
    double m_goal_receive_timestamp_;
    std::mutex m_goal_mutex_;
    std::string m_robot_tf_frame_name_;
    void goalCallback(const geometry_msgs::msg::PoseStamped &msg);

    // tf2_ros::TransformListener m_tf_listener;
    // tf2_ros::StampedTransform m_tf_robot_to_map_;
    geometry_msgs::msg::TransformStamped m_tf_robot_to_map_;
    void getRobotPoseInMapFrame();

    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_;
    
    void loadParameters();

    void exit_pathplan_function(const double time_start);

    void ros_info(const std::string& str);

    void checkPath();

    ClassPathValidator m_path_validator_;


public:
    ClassPathPlanner();
    ~ClassPathPlanner();
};

/**
 * @brief The constructor function for this class. 
 * @param 
*/
ClassPathPlanner::ClassPathPlanner(): Node("path_plan_node")
{
    loadParameters(); 

    m_map_received_ = false;
    m_goal_received_ = false; 
    m_goal_solved_ = false;

    m_publisher_path_  = this->create_publisher<nav_msgs::msg::Path>(
        m_topic_name_path_published_, 10
    );

    m_subscriber_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        m_topic_name_map_subscribed_, 10, std::bind(&ClassPathPlanner::mapCallback, this, std::placeholders::_1)
    );

    m_subscriber_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        m_topic_name_goal_subscribed_, 10, std::bind(&ClassPathPlanner::goalCallback, this, std::placeholders::_1)
    );

    m_periodic_timer_ = this->create_wall_timer(200ms, std::bind(&ClassPathPlanner::pathPlan, this));

    m_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_);

    // m_ros_nh_.advertise<nav_msgs::Path>( m_topic_name_path_published_, 10);

    // m_subscriber_map_ = m_ros_nh_.subscribe( m_topic_name_map_subscribed_ , 1, &ClassPathPlanner::mapCallback,  this);

    // m_subscriber_goal_ = m_ros_nh_.subscribe( m_topic_name_goal_subscribed_ , 1, &ClassPathPlanner::goalCallback, this);

    // m_periodic_timer_ = m_ros_nh_.createTimer( ros::Duration(m_planer_interval_), &ClassPathPlanner::pathPlan, this );

    ros_info("ClassPathPlanner inti Done.");
}

/**
 * @brief The destructor function.
*/
ClassPathPlanner::~ClassPathPlanner()
{
}


/**
 * @brief Print the string to the RCLCPP_INFO. 
 * @param str The string to be printed. 
*/
void ClassPathPlanner::ros_info(const std::string& str)
{
    RCLCPP_INFO(this->get_logger(), str.c_str());
}


/**
 * @brief Load parameters from the json file. 
*/
void ClassPathPlanner::loadParameters()
{
    m_topic_name_map_subscribed_ = "/map_fusion";
    m_topic_name_goal_subscribed_ = "/goal_pose"; // "/move_base_simple/goal";
    m_topic_name_path_published_ = "/path";

    m_map_tf_frame_name_ = "/map";
    m_robot_tf_frame_name_ = "/base_link";

    m_path_plan_timeout_ms_ = 500;

    // m_planer_interval_ = 0.2;

    ros_info("ClassPathPlanner loadParameters Done.");
}

/**
 * @brief This function would be used in the function pathPlan(), when it needs 
 * to terminate the path finding. 
 * @param time_start This is the timestamp when the function pathPlan() starts.
*/
void ClassPathPlanner::exit_pathplan_function(const double time_start)
{
    m_map_mutex_.unlock();
    m_goal_mutex_.unlock(); 

    double t2 = helperGetTime(); 

    std::stringstream ss;
    ss << "Function pathPlan() used " << int((t2-time_start)*1000.0) << " ms." << std::endl;
    ros_info(ss.str());
}

/**
 * @brief This function will be executed by the ros timer, about 10 hz. 
 * @param event The ROS timer event.
*/
void ClassPathPlanner::pathPlan()
{
    if( ! m_map_received_ ) return;
    if( ! m_goal_received_ ) return;

    checkPath();

    if (m_goal_solved_) return;

    ros_info("ClassPathPlanner::path_plan() start");

    double t1 = helperGetTime();

    m_map_mutex_.lock();
    m_goal_mutex_.lock();

    getRobotPoseInMapFrame();

    bool _planner_set_map_success = m_ha_planner_.setMap(&m_map_msg_);
    bool _planner_set_pose_sucess = m_ha_planner_.setStartGoalPoses(m_start_pose_, m_goal_pose_);
    bool _planner_setup_succes = m_ha_planner_.setup(m_path_plan_timeout_ms_);

    if (! (_planner_setup_succes || _planner_set_map_success || _planner_set_pose_sucess))
    {
        exit_pathplan_function(t1);
        return;
    }

    bool found_path = m_ha_planner_.search();

    if(! found_path )
    {
        exit_pathplan_function(t1);
        return;
    }

    ClassCustomPathContainer path;
    m_ha_planner_.getFinalHybridAstarPath(path);

    m_goal_solved_ = true;

    // std::cout << "path size: " << path.size() << std::endl;

    m_navmsgs_path_msg_.header.frame_id = m_map_tf_frame_name_;

    m_navmsgs_path_msg_.poses.clear();
    geometry_msgs::msg::PoseStamped one_pose;

    for( auto point : path.getPath() ){
        // std::cout << "in for" << std::endl;
        one_pose.pose.position.x = point[0] + m_map_msg_.info.origin.position.x;
        one_pose.pose.position.y = point[1] + m_map_msg_.info.origin.position.y;
        one_pose.pose.position.z = 0.0;
        one_pose.pose.orientation = tf2quaToGeoQua(twodYawToTf2qua(point[2]));

        m_navmsgs_path_msg_.poses.push_back(one_pose);
        // std::cout << "path_ " << point[0] << " " << point[1] << " " << std::endl;
    }

    m_publisher_path_->publish( m_navmsgs_path_msg_ );

    

    // std::cout << "Hybrid-A* last point " << path_msg_.poses.back().pose.position.x << " " << path_msg_.poses.back().pose.position.y << " " << std::endl;
    // std::cout << "Goal           point " << goal_pose_[0]+map_msg_.info.origin.position.x << " " << goal_pose_[1]+map_msg_.info.origin.position.y << " " << std::endl;

    exit_pathplan_function(t1);
}

/**
 * @brief Call this function to get the latest robot pose.
*/
void ClassPathPlanner::getRobotPoseInMapFrame()
{
    try{
        // m_tf_listener.lookupTransform( m_map_tf_frame_name_, m_robot_tf_frame_name_, ros::Time(0), m_tf_robot_to_map_);

        // m_tf_robot_to_map_ = m_tf_buffer_->lookupTransform(m_map_tf_frame_name_, m_robot_tf_frame_name_, tf2::TimePointZero);
        m_tf_robot_to_map_ = m_tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

        m_start_pose_.x = m_tf_robot_to_map_.transform.translation.x - m_map_msg_.info.origin.position.x;
        m_start_pose_.y = m_tf_robot_to_map_.transform.translation.y - m_map_msg_.info.origin.position.y;
        m_start_pose_.yaw = StampedTransformToYaw(&m_tf_robot_to_map_);
        // std::cout << start_pose_[0] << " " << start_pose_[1] << " " << start_pose_[2] << std::endl;
        // std::cout << "robot yaw  " << start_pose_[2]*180.0/M_PI << std::endl;
    }
    catch (tf2::TransformException ex){
        RCLCPP_ERROR(this->get_logger(), "getRobotPoseInMapFrame: TransformException %s", ex.what());
    }
}

/**
 * @brief ROS callback frunction for the goal message. 
 * @param msg message of the rostopic
*/
void ClassPathPlanner::goalCallback(const geometry_msgs::msg::PoseStamped &msg)
{
    m_goal_mutex_.lock();

    RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 2.0, "ClassPathPlanner::goalCallback() heard msg.");

    m_goal_pose_.x = msg.pose.position.x - m_map_msg_.info.origin.position.x;
    m_goal_pose_.y = msg.pose.position.y - m_map_msg_.info.origin.position.y;
    m_goal_pose_.yaw =  geoQuaToYaw(&(msg.pose.orientation));

    m_goal_received_ = true;
    m_goal_solved_ = false;

    m_goal_stamp_.stampNow();
    
    m_goal_mutex_.unlock();
}

/**
 * @brief ROS callback frunction for the occupancy grid map message. 
 * @param msg message of the rostopic
*/
void ClassPathPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid &msg)
{
    m_map_mutex_.lock();
    RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 3.0, "ClassPathPlanner::mapCallback() heard msg.");
    
    m_map_msg_.data = msg.data;
    m_map_msg_.header = msg.header;
    m_map_msg_.info = msg.info;
    m_map_received_ = true;
    m_map_mutex_.unlock();
}


/**
 * @brief Check if the current path is still valid.
*/
void ClassPathPlanner::checkPath()
{
    if (! m_goal_solved_) return;

    m_path_validator_.setPath(m_navmsgs_path_msg_);
    m_path_validator_.setMap(&m_map_msg_);
    m_path_validator_.setRobotPose(m_tf_robot_to_map_);

    if (! m_path_validator_.validate())
    {
        m_goal_solved_ = false;
        std::cerr << "Path is no longer valid. Need to replan." << std::endl;
    }

}


#endif