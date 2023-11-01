#ifndef CLASS_PATH_PLANNER
#define CLASS_PATH_PLANNER

#include <iostream>
#include <vector>
#include <string>
#include <chrono>
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

#include "../hybrid_a_star_module/hybrid_astar.h"
#include "conversion_tools.h"
#include "../numerical_optimization_curve/class_raw_path_sampler.h"
#include "../numerical_optimization_curve/class_corridor_generator.h"
// #include "../numerical_optimization_curve/class_path_solver.h"


using std::chrono::high_resolution_clock;

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

    struct robotPose
    {
        double x_meter, y_meter; // meter
        double robot_yaw_rad;    // radian
        int x_grid, y_grid;
        int x_grid_prev, y_grid_prev;
    };    

    ClassHybridAStar planner_;

    ClassRawPathSampler raw_sampler_;

    ClassCorridorGenerator m_corridor_generator_;

    // ClassPathSolver m_opt_path_solver_;

    int path_plan_timeout_ms_;

    std::array<double, 3> start_pose_;
    std::array<double, 3> goal_pose_;

    // std::string odom_topic_name_; 
    std::string path_published_topic_name_; 
    std::string path_sampled_published_topic_name_; 
    std::string path_reconstruct_sampled_published_topic_name_; 
    std::string corridor_vis_published_topic_name_; 
    std::string opt_path_vis_published_topic_name_; 
    std::string opt_init_path_vis_published_topic_name_; 

    std::string map_subscribed_topic_name_; 
    std::string goal_subscribed_topic_name_;

    ros::Publisher path_puber_ ;
    ros::Publisher sampled_path_puber_ ;
    ros::Publisher reconstruct_sampled_path_puber_ ;
    ros::Publisher corridor_vis_puber_ ;
    ros::Publisher opt_path_vis_puber_ ;
    ros::Publisher opt_init_path_vis_puber_ ;

    ros::Subscriber map_suber_ ;
    ros::Subscriber goal_suber_;
    ros::Subscriber state_suber_;

    bool map_received_, goal_received_; 

    nav_msgs::Path path_msg_, sampled_path_msg_, opt_path_msg_, opt_init_path_msg, reconstruct_sampled_path_msg;

    ros::Timer  periodic_path_planer_;
    double planer_interval_;

    ros::NodeHandle nh_;
    tf::TransformListener tf_listener;

    std::mutex map_mutex_, odom_mutex_, goal_mutex_;

    std::string map_frame_, robot_frame_; //, depth_frame_, sonic_frame_; 

    void load_parameters();

    double mod_2pi(double angle);

    nav_msgs::OccupancyGrid map_msg_;
    geometry_msgs::PoseStamped goal_msg_;
    hawa_msgs::car_states car_state_;

    tf::StampedTransform robot_to_map_tf_;

    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void state_callback(const hawa_msgs::car_states::ConstPtr &msg);


    void path_plan( const ros::TimerEvent &event );

    void get_robot_pose_in_map_frame();

    double helper_get_time();


public:
    ClassPathPlanner(const ros::NodeHandle nh_in_);
    ~ClassPathPlanner();
};

ClassPathPlanner::ClassPathPlanner(const ros::NodeHandle nh_in_): nh_{nh_in_}
{
    load_parameters(); 

    map_received_ = false;
    goal_received_ = false; 

    path_puber_  = nh_.advertise<nav_msgs::Path>( path_published_topic_name_, 10);

    sampled_path_puber_  = nh_.advertise<nav_msgs::Path>( path_sampled_published_topic_name_, 10);

    reconstruct_sampled_path_puber_  = nh_.advertise<nav_msgs::Path>( path_reconstruct_sampled_published_topic_name_, 10);

    corridor_vis_puber_  = nh_.advertise<visualization_msgs::MarkerArray>( corridor_vis_published_topic_name_, 10);

    opt_path_vis_puber_ = nh_.advertise<nav_msgs::Path>( opt_path_vis_published_topic_name_, 10);

    opt_init_path_vis_puber_ = nh_.advertise<nav_msgs::Path>( opt_init_path_vis_published_topic_name_, 10);

    map_suber_ = nh_.subscribe( map_subscribed_topic_name_ , 1, &ClassPathPlanner::map_callback,  this);
    goal_suber_= nh_.subscribe( goal_subscribed_topic_name_ , 1, &ClassPathPlanner::goal_callback, this);

    state_suber_ = nh_.subscribe( "/sim_ego_car_state" , 1, &ClassPathPlanner::state_callback, this);


    periodic_path_planer_ = nh_.createTimer( ros::Duration(planer_interval_), &ClassPathPlanner::path_plan, this );

    start_pose_ = {0,0,0};
    goal_pose_ = {0,0,0};
    std::cout << "ClassPathPlanner inti Done" << std::endl;

}

ClassPathPlanner::~ClassPathPlanner()
{
}




void ClassPathPlanner::load_parameters()
{
    map_subscribed_topic_name_ = "/map_fusion";
    goal_subscribed_topic_name_ = "/move_base_simple/goal";
    path_published_topic_name_ = "/path";
    path_sampled_published_topic_name_ = "/path_sampled";
    path_reconstruct_sampled_published_topic_name_ = "/path_rec";
    corridor_vis_published_topic_name_ = "/corridors";
    opt_path_vis_published_topic_name_ = "/opt_path";
    opt_init_path_vis_published_topic_name_ = "/opt_init_path";


    map_frame_ = "/map";
    robot_frame_ = "/base_link";

    planer_interval_ = 0.1;
}


void ClassPathPlanner::path_plan( const ros::TimerEvent &event )
{
    if( ! map_received_ ) return;
    if( ! goal_received_ ) return;

    std::cout << "\nClassPathPlanner::path_plan() start" << std::endl;

    double t1 = helper_get_time();

    map_mutex_.lock();
    goal_mutex_.lock();

    get_robot_pose_in_map_frame();

    bool planner_setup_success = planner_.setup(500, start_pose_, goal_pose_, map_msg_.info.width, map_msg_.info.height, map_msg_.data, map_msg_.info.resolution);
    if(planner_setup_success)
    {
        
        bool found_path = planner_.search();

        std::cout << "Search Done.  found_path:" << found_path << std::endl;

        if( found_path ){
            // std::cout << "in if" << std::endl;
            // std::deque< array<double, 3> >  path;
            ClassCustomPathContainer path;
            planner_.get_path(path);

            // std::cout << "path size: " << path.size() << std::endl;

            path_msg_.header.frame_id = map_frame_;

            path_msg_.poses.clear();
            geometry_msgs::PoseStamped one_pose;

            for( auto point : path.get_path() ){
                // std::cout << "in for" << std::endl;
                one_pose.pose.position.x = point[0] + map_msg_.info.origin.position.x;
                one_pose.pose.position.y = point[1] + map_msg_.info.origin.position.y;
                one_pose.pose.position.z = 0.0;
                one_pose.pose.orientation = tf2qua_to_geoQua(twoD_yaw_to_tf2qua(point[2]));

                path_msg_.poses.push_back(one_pose);
                // std::cout << "path_ " << point[0] << " " << point[1] << " " << std::endl;
            }

            path_puber_.publish( path_msg_ );

            // std::cout << "Hybrid-A* last point " << path_msg_.poses.back().pose.position.x << " " << path_msg_.poses.back().pose.position.y << " " << std::endl;
            // std::cout << "Goal           point " << goal_pose_[0]+map_msg_.info.origin.position.x << " " << goal_pose_[1]+map_msg_.info.origin.position.y << " " << std::endl;


            raw_sampler_.set_raw_path_ptr(path);
            StructWaypointWithTwist _robot_pose, _goal_pose;
            _robot_pose.x = start_pose_[0];
            _robot_pose.y = start_pose_[1];
            _robot_pose.yaw = start_pose_[2];
            _goal_pose.x = goal_pose_[0];
            _goal_pose.y = goal_pose_[1];
            _goal_pose.yaw = goal_pose_[2];
            raw_sampler_.set_robot_initial_state(_robot_pose);
            raw_sampler_.set_goal_state(_goal_pose);
            ClassCustomPathContainer _sampled_path;
            if (raw_sampler_.process_raw_data())
            {
                // std::cout << "sampled path." << std::endl;
                raw_sampler_.get_path(_sampled_path);
                sampled_path_msg_.poses.clear();
                sampled_path_msg_.header.frame_id = map_frame_;
                for( auto point : _sampled_path.get_path() )
                {
                    one_pose.pose.position.x = point[0] + map_msg_.info.origin.position.x;
                    one_pose.pose.position.y = point[1] + map_msg_.info.origin.position.y;
                    one_pose.pose.position.z = 0.0;
                    one_pose.pose.orientation = tf2qua_to_geoQua(twoD_yaw_to_tf2qua(point[2]));
                    sampled_path_msg_.poses.push_back(one_pose);
                }
                sampled_path_puber_.publish(sampled_path_msg_);
                // cout << "path_msg_  size  " << path_msg_.poses.size() << endl;
                // cout << "sampled_path_msg_  size  " << sampled_path_msg_.poses.size() << endl;

                raw_sampler_.m_reconstructed_path_.header = sampled_path_msg_.header;
                for (auto& ppppt : raw_sampler_.m_reconstructed_path_.poses)
                {
                    ppppt.pose.position.x += map_msg_.info.origin.position.x;
                    ppppt.pose.position.y += map_msg_.info.origin.position.y;
                }

                reconstruct_sampled_path_puber_.publish(raw_sampler_.m_reconstructed_path_);

                m_corridor_generator_.set_map_data(map_msg_.info.width, map_msg_.info.height, map_msg_.data, map_msg_.info.resolution, 88);
                m_corridor_generator_.set_points_data(_sampled_path);
                m_corridor_generator_.generate_bboxs();
                ClassCorridor _temp_corridor;
                m_corridor_generator_.get_result(_temp_corridor);
                visualization_msgs::MarkerArray _temp_corridor_vis_msg;
                
                _temp_corridor.get_ros_marker_format(map_msg_.header.frame_id, _temp_corridor_vis_msg, map_msg_.info.origin.position.x, map_msg_.info.origin.position.y);
                corridor_vis_puber_.publish(_temp_corridor_vis_msg);

                // StructModelStates _opt_init_state, _opt_goal_state;
                // _opt_init_state.x = _robot_pose.x;
                // _opt_init_state.y = _robot_pose.y;
                // _opt_init_state.yaw = _robot_pose.yaw;
                // _opt_init_state.v = car_state_.body_frame_velocity.twist.linear.x;
                // _opt_init_state.a = car_state_.body_frame_accel.accel.linear.x;
                // _opt_init_state.str_angl = car_state_.steer_radius;
                // _opt_init_state.str_rate = 0;

                // _opt_goal_state.x = _goal_pose.x;
                // _opt_goal_state.y = _goal_pose.y;
                // _opt_goal_state.yaw = _goal_pose.yaw;
                // _opt_goal_state.v = 0;
                // _opt_goal_state.a = 0;
                // _opt_goal_state.str_angl = 0;
                // _opt_goal_state.str_rate = 0;
                // // _opt_goal_state.str_angl = ;

                

                // m_opt_path_solver_.solve(_opt_init_state, _opt_goal_state, _sampled_path.number_of_points()-1, 1.0);

                // vector<vector<double>> _opt_init_path;
                // m_opt_path_solver_.get_initial_path(_opt_init_path);
                // if (_opt_init_path.size() > 1)
                // {
                //     opt_init_path_msg.header = sampled_path_msg_.header;
                //     for (int i=0; i<_opt_init_path.size(); i++)
                //     {
                //         geometry_msgs::PoseStamped _p;
                //         _p.pose.position.x = _opt_init_path[i][0] + map_msg_.info.origin.position.x;
                //         _p.pose.position.y = _opt_init_path[i][1] + map_msg_.info.origin.position.y;
                //         opt_init_path_msg.poses.push_back( _p );
                //     }
                //     opt_init_path_vis_puber_.publish(opt_init_path_msg);
                //     opt_init_path_msg.poses.clear();
                // }


                // vector<vector<double>> _opt_path;
                // m_opt_path_solver_.get_result_path(_opt_path);
                // if (_opt_path.size() > 1)
                // {
                //     opt_path_msg_.header = sampled_path_msg_.header;
                //     for (int i=0; i<_opt_path.size(); i++)
                //     {
                //         geometry_msgs::PoseStamped _p;
                //         _p.pose.position.x = _opt_path[i][0] + map_msg_.info.origin.position.x;
                //         _p.pose.position.y = _opt_path[i][1] + map_msg_.info.origin.position.y;
                //         opt_path_msg_.poses.push_back( _p );
                //     }
                //     opt_path_vis_puber_.publish(opt_path_msg_);
                //     opt_path_msg_.poses.clear();
                // }
                

            }
        }

        // std::cout << "after if" << std::endl;

    }

    map_mutex_.unlock();
    goal_mutex_.unlock(); 

    double t2 = helper_get_time(); 

    std::cout << "function path_plan " << int((t2-t1)*1000.0) << " ms" << std::endl;

    // exit(0);

}





void ClassPathPlanner::get_robot_pose_in_map_frame(){
    try{
        tf_listener.lookupTransform( map_frame_, robot_frame_, ros::Time(0), robot_to_map_tf_);
        start_pose_[0] = robot_to_map_tf_.getOrigin().x() - map_msg_.info.origin.position.x;
        start_pose_[1] = robot_to_map_tf_.getOrigin().y() - map_msg_.info.origin.position.y;
        // start_pose_[2] = robot_to_map_tf_.getRotation().

        tf::Quaternion q(robot_to_map_tf_.getRotation().x(),
                         robot_to_map_tf_.getRotation().y(),
                         robot_to_map_tf_.getRotation().z(),
                         robot_to_map_tf_.getRotation().w()  );

        tfScalar yaw, pitch, roll;
        tf::Matrix3x3 mat(q);
        mat.getEulerYPR(yaw, pitch, roll);
        start_pose_[2] = mod_2pi(yaw);
        // std::cout << start_pose_[0] << " " << start_pose_[1] << " " << start_pose_[2] << std::endl;
        // std::cout << "robot yaw  " << start_pose_[2]*180.0/M_PI << std::endl;

    }
    catch (tf::TransformException ex){
      ROS_ERROR("\nget_robot_pose_in_map_frame: \n%s",ex.what());
    //   ros::Duration(0.1).sleep();
    }
}


void ClassPathPlanner::state_callback(const hawa_msgs::car_states::ConstPtr &msg)
{
    car_state_.body_frame_velocity = msg->body_frame_velocity;
    car_state_.body_frame_accel = msg->body_frame_accel;
    car_state_.steer_radius = msg->steer_radius;
}


void ClassPathPlanner::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map_mutex_.lock();
    // std::cout << "map_callback start" << std::endl;
    map_msg_.data = msg->data;
    map_msg_.header = msg->header;
    map_msg_.info = msg->info;
    map_received_ = true;
    // std::cout << "map_callback end" << std::endl;
    map_mutex_.unlock();
}


void ClassPathPlanner::goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goal_mutex_.lock();

    goal_msg_ = *msg;

    goal_pose_[0] = msg->pose.position.x - map_msg_.info.origin.position.x;
    goal_pose_[1] = msg->pose.position.y - map_msg_.info.origin.position.y;
    tf::Quaternion q(msg->pose.orientation.x,
                     msg->pose.orientation.y,
                     msg->pose.orientation.z,
                     msg->pose.orientation.w  );
    tfScalar yaw, pitch, roll;
    tf::Matrix3x3 mat(q);
    mat.getEulerYPR(yaw, pitch, roll);

    goal_pose_[2] =  mod_2pi(yaw);

    goal_received_ = true;
    
    goal_mutex_.unlock();

}


double ClassPathPlanner::mod_2pi( double a)
{
    double angle = a;
    while (angle > 2*M_PI){
        angle -= 2*M_PI;
    }
    while (angle < 0){
        angle += 2*M_PI;
    }
    return angle;
}





/**
 * @brief 
 * return time in seconds, epoch time. 
 */
double ClassPathPlanner::helper_get_time()
{
    return high_resolution_clock::now().time_since_epoch().count()/1000000000.0; 
}












#endif
