#ifndef CLASS_PATH_PLANNER
#define CLASS_PATH_PLANNER

#include <iostream>
#include <vector>
#include <string>

#include <mutex>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Path.h"
#include "tf/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"

#include "hawa_msgs/car_states.h"

// #include "../car_pathplan/struct_simple_pose.h"
#include "../hybrid_a_star_module/hybrid_astar.h"
#include "../utils/hawa_conversion_tools.h"
#include "../utils/hawa_tools.h"
#include "../utils/hawa_data_containers.h"


/*
The pose of start/robot, goal, path are converted from /map frame to map-grid-space;
converted_tf = tf_in_map_frame - map_msg_origin;

*/


/*
TODO:
add a function to check if the current path is still valid:
- robot position xy is close to it
- robot heading is close to the path
- path is clear from obstacles
- 
if it's still valid, then no need to search new path. 

*/


class ClassPathPlanner
{
private:

    ClassHybridAStar m_ha_planner_;
    StructPoseReal m_start_pose_;
    StructPoseReal m_goal_pose_;

    int m_path_plan_timeout_ms_;

    ros::NodeHandle m_ros_nh_;

    ros::Publisher m_publisher_path_ ;
    ros::Timer m_periodic_timer_;
    std::string m_topic_name_path_published_;
    nav_msgs::Path m_navmsgs_path_msg_;
    double m_planer_interval_;
    void pathPlan( const ros::TimerEvent &event );

    ros::Subscriber m_subscriber_map_;
    std::string m_topic_name_map_subscribed_; 
    nav_msgs::OccupancyGrid m_map_msg_;
    bool m_map_received_;
    std::mutex m_map_mutex_;
    std::string m_map_tf_frame_name_; 
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

    ros::Subscriber m_subscriber_goal_;
    std::string m_topic_name_goal_subscribed_;
    bool m_goal_received_;
    std::mutex m_goal_mutex_;
    std::string m_robot_tf_frame_name_;
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    tf::TransformListener m_tf_listener;
    tf::StampedTransform m_tf_robot_to_map_;
    void getRobotPoseInMapFrame();
    
    void load_parameters();

    void exit_pathplan_function(const double time_start);


public:
    ClassPathPlanner(const ros::NodeHandle nh_in_);
    ~ClassPathPlanner();
};

ClassPathPlanner::ClassPathPlanner(const ros::NodeHandle nh_in_): m_ros_nh_{nh_in_}
{
    load_parameters(); 

    m_map_received_ = false;
    m_goal_received_ = false; 

    m_publisher_path_  = m_ros_nh_.advertise<nav_msgs::Path>( m_topic_name_path_published_, 10);

    m_subscriber_map_ = m_ros_nh_.subscribe( m_topic_name_map_subscribed_ , 1, &ClassPathPlanner::mapCallback,  this);

    m_subscriber_goal_ = m_ros_nh_.subscribe( m_topic_name_goal_subscribed_ , 1, &ClassPathPlanner::goalCallback, this);

    m_periodic_timer_ = m_ros_nh_.createTimer( ros::Duration(m_planer_interval_), &ClassPathPlanner::pathPlan, this );

    ROS_INFO_STREAM("ClassPathPlanner inti Done.");
}

ClassPathPlanner::~ClassPathPlanner()
{
}

void ClassPathPlanner::load_parameters()
{
    m_topic_name_map_subscribed_ = "/map_fusion";
    m_topic_name_goal_subscribed_ = "/move_base_simple/goal";
    m_topic_name_path_published_ = "/path";

    m_map_tf_frame_name_ = "/map";
    m_robot_tf_frame_name_ = "/base_link";

    m_path_plan_timeout_ms_ = 500;

    m_planer_interval_ = 0.2;

    ROS_INFO_STREAM("ClassPathPlanner load_parameters Done.");
}

void ClassPathPlanner::exit_pathplan_function(const double time_start)
{
    m_map_mutex_.unlock();
    m_goal_mutex_.unlock(); 

    double t2 = helper_get_time(); 

    ROS_INFO_STREAM("Function pathPlan() used " << int((t2-time_start)*1000.0) << " ms.");
}

void ClassPathPlanner::pathPlan( const ros::TimerEvent &event )
{
    if( ! m_map_received_ ) return;
    if( ! m_goal_received_ ) return;

    ROS_INFO("ClassPathPlanner::path_plan() start");

    double t1 = helper_get_time();

    m_map_mutex_.lock();
    m_goal_mutex_.lock();

    getRobotPoseInMapFrame();

    bool planner_setup_succes = m_ha_planner_.setup(m_path_plan_timeout_ms_, m_start_pose_, m_goal_pose_, &m_map_msg_);

    if (! planner_setup_succes)
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
    m_ha_planner_.get_path(path);

    // std::cout << "path size: " << path.size() << std::endl;

    m_navmsgs_path_msg_.header.frame_id = m_map_tf_frame_name_;

    m_navmsgs_path_msg_.poses.clear();
    geometry_msgs::PoseStamped one_pose;

    for( auto point : path.get_path() ){
        // std::cout << "in for" << std::endl;
        one_pose.pose.position.x = point[0] + m_map_msg_.info.origin.position.x;
        one_pose.pose.position.y = point[1] + m_map_msg_.info.origin.position.y;
        one_pose.pose.position.z = 0.0;
        one_pose.pose.orientation = tf2qua_to_geoQua(twoD_yaw_to_tf2qua(point[2]));

        m_navmsgs_path_msg_.poses.push_back(one_pose);
        // std::cout << "path_ " << point[0] << " " << point[1] << " " << std::endl;
    }

    m_publisher_path_.publish( m_navmsgs_path_msg_ );

    // std::cout << "Hybrid-A* last point " << path_msg_.poses.back().pose.position.x << " " << path_msg_.poses.back().pose.position.y << " " << std::endl;
    // std::cout << "Goal           point " << goal_pose_[0]+map_msg_.info.origin.position.x << " " << goal_pose_[1]+map_msg_.info.origin.position.y << " " << std::endl;

            
        
    

    // m_map_mutex_.unlock();
    // m_goal_mutex_.unlock(); 

    // double t2 = helper_get_time(); 

    // ROS_INFO("Function pathPlan() used %d ms.", int((t2-t1)*1000.0));


}

void ClassPathPlanner::getRobotPoseInMapFrame(){
    try{
        m_tf_listener.lookupTransform( m_map_tf_frame_name_, m_robot_tf_frame_name_, ros::Time(0), m_tf_robot_to_map_);

        m_start_pose_.x = m_tf_robot_to_map_.getOrigin().x() - m_map_msg_.info.origin.position.x;
        m_start_pose_.y = m_tf_robot_to_map_.getOrigin().y() - m_map_msg_.info.origin.position.y;
        m_start_pose_.yaw = tfStampedTransform_to_yaw(&m_tf_robot_to_map_);
        // std::cout << start_pose_[0] << " " << start_pose_[1] << " " << start_pose_[2] << std::endl;
        // std::cout << "robot yaw  " << start_pose_[2]*180.0/M_PI << std::endl;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("\ngetRobotPoseInMapFrame: \n%s",ex.what());
    }
}


void ClassPathPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    m_goal_mutex_.lock();
    ROS_DEBUG_STREAM_THROTTLE(2.0, "ClassPathPlanner::goalCallback() heard msg.");

    m_goal_pose_.x = msg->pose.position.x - m_map_msg_.info.origin.position.x;
    m_goal_pose_.y = msg->pose.position.y - m_map_msg_.info.origin.position.y;
    m_goal_pose_.yaw =  geoQua_to_yaw(&(msg->pose.orientation));

    m_goal_received_ = true;
    
    m_goal_mutex_.unlock();
}


void ClassPathPlanner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    m_map_mutex_.lock();
    ROS_DEBUG_STREAM_THROTTLE(3.0, "ClassPathPlanner::mapCallback() heard msg.");
    m_map_msg_.data = msg->data;
    m_map_msg_.header = msg->header;
    m_map_msg_.info = msg->info;
    m_map_received_ = true;
    m_map_mutex_.unlock();
}


#endif
