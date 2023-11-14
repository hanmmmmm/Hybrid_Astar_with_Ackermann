#ifndef CLASS_NODE_ACKERMANN_SIMULATION
#define CLASS_NODE_ACKERMANN_SIMULATION

#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <mutex>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
// #include "tf/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "nav_msgs/Odometry.h"

// #include "hawa_msgs/car_states.h"

#include "visualization_msgs/Marker.h"
#include "class_car_body_box_marker.h"


using std::chrono::high_resolution_clock;

/*
The pose of start/robot, goal, path are converted from /map frame to map-grid-space;
converted_tf = tf_in_map_frame - map_msg_origin;

*/


/*
TODO:

*/


double mod_2pi( double a)
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
double helper_get_time_second()
{
    return high_resolution_clock::now().time_since_epoch().count()/1000000000.0; 
}


struct DynamicsLimits
{
    double max_linear_v_mps = 1.7;
    double min_linear_v_mps = -1.7;
    double linear_a_mpss = 0.8;

    double max_steer_rad = 0.5;
    double min_steer_rad = -0.5;
    double steer_rate_radps = 1.0;

    // double max_angular_w_radps = 1.0;
    // double min_angular_w_radps = -1.0;
};


class DynamicsStates
{
public:
    double x_meter_ = 0;
    double y_meter_ = 0;
    double yaw_rad_ = 0;

    double steer_rad_actual_ = 0;
    double steer_rad_target_ = 0;

    double linear_vb_mps_actual_ = 0;
    double linear_vb_mps_target_ = 0;

    double angular_wb_radps_actual_ = 0;
    // double angular_wb_radps_target = 0;

    double axle_distance_ = 0.25;

    DynamicsLimits limits;

    double get_vx_mps();
    double get_vy_mps();

    void set_target_linear_vb_mps(double _target_mps);
    void set_target_steer_rad(double _target_rad);

    void calc_actual_linear_v_mps(double _dt);
    void calc_actual_steer_rad(double _dt);

    void update_all_states(double _dt);
};

double DynamicsStates::get_vx_mps()
{
    // std::cout << std::fixed << std::setprecision(2) << "get_vx_mps " << linear_vb_mps_actual_ * cos(yaw_rad_) << std::endl;
    return linear_vb_mps_actual_ * cos(yaw_rad_);
}

double DynamicsStates::get_vy_mps()
{
    return linear_vb_mps_actual_ * sin(yaw_rad_);
}

void DynamicsStates::set_target_linear_vb_mps(double _target_mps)
{
    // std::cout << "set_target_linear_vb_mps  " << target_mps;
    _target_mps = std::min(_target_mps, limits.max_linear_v_mps);
    _target_mps = std::max(_target_mps, limits.min_linear_v_mps);
    linear_vb_mps_target_ = _target_mps;
    // std::cout << std::fixed << std::setprecision(2) << "linear_vb_mps_target " << linear_vb_mps_target << std::endl;
}

void DynamicsStates::set_target_steer_rad(double _target_rad)
{
    _target_rad = std::min(_target_rad, limits.max_steer_rad);
    _target_rad = std::max(_target_rad, limits.min_steer_rad);
    steer_rad_target_ = _target_rad;
}

void DynamicsStates::calc_actual_linear_v_mps(double _dt)
{
    // std::cout << "calc_actual_linear_v_mps" << std::endl;
    if (linear_vb_mps_actual_ < linear_vb_mps_target_)
    {
        linear_vb_mps_actual_ = std::min(linear_vb_mps_target_, linear_vb_mps_actual_ + limits.linear_a_mpss * _dt);
    }
    else if (linear_vb_mps_actual_ > linear_vb_mps_target_)
    {
        linear_vb_mps_actual_ = std::max(linear_vb_mps_target_, linear_vb_mps_actual_ - limits.linear_a_mpss * _dt);
    }
    // std::cout << std::fixed << std::setprecision(2) << "target_mps " << target_mps << "   linear_vb_mps_actual_  " << linear_vb_mps_actual_ << std::endl;
}

void DynamicsStates::calc_actual_steer_rad(double _dt)
{
    if (steer_rad_actual_ < steer_rad_target_)
    {
        steer_rad_actual_ = std::min(steer_rad_target_, steer_rad_actual_ + limits.steer_rate_radps * _dt);
    }
    else if (steer_rad_actual_ > steer_rad_target_)
    {
        steer_rad_actual_ = std::max(steer_rad_target_, steer_rad_actual_ - limits.steer_rate_radps * _dt);
    }
}

void DynamicsStates::update_all_states(double _dt)
{
    // std::cout << "update_all_states" << std::endl;
    calc_actual_linear_v_mps(_dt);
    calc_actual_steer_rad(_dt);
    // std::cout << std::fixed << std::setprecision(2) << "**********linear_vb_mps_actual_ " << linear_vb_mps_actual_ << std::endl;
    x_meter_ += get_vx_mps() * _dt;
    y_meter_ += get_vy_mps() * _dt;

    angular_wb_radps_actual_ = linear_vb_mps_actual_ * tan(steer_rad_actual_) / axle_distance_;
    // std::cout << std::fixed << std::setprecision(2) << "-**********--angular_wb_radps_actual_ " << angular_wb_radps_actual_ << std::endl;
    yaw_rad_ += mod_2pi( angular_wb_radps_actual_ * _dt );
    yaw_rad_ = mod_2pi(yaw_rad_);
}





class ClassNodeAckermannSim
{
private:
    ros::NodeHandle nh_;
    tf::TransformBroadcaster br_;
    ros::Subscriber akm_drive_suber_ ;
    ros::Publisher odometry_puber_;
    ros::Publisher states_puber_;
    ros::Publisher car_body_line_puber_;
    ros::Timer  periodic_timer_;
    std::string map_frame_, robot_frame_;
    std::string akm_drive_subscribed_topic_name_; 
    std::string odometry_published_topic_name_; 
    std::string states_published_topic_name_; 
    std::string car_body_line_published_topic_name_; 
    ackermann_msgs::AckermannDriveStamped akm_drive_msg_;
    double akm_drive_msg_stamp_system_;
    nav_msgs::Odometry odometry_msg_;
    // hawa_msgs::car_states states_msg_;

    ClassCarBox2D car_box_;

private:
    DynamicsStates dyna_states_;
    double timestamp_last_update_;
    double main_loop_timer_interval_second_;
    bool FLAG_pub_tf__map2odom_;

    double linear_velocity_previous_;

    
private:
    void akm_callback(const ackermann_msgs::AckermannDriveStampedConstPtr &msg);
    void main_update( const ros::TimerEvent &event );
    void load_parameters();
    
    
public:
    ClassNodeAckermannSim(const ros::NodeHandle nh_in_);
    ~ClassNodeAckermannSim();
};

ClassNodeAckermannSim::ClassNodeAckermannSim(const ros::NodeHandle nh_in_): nh_{nh_in_}
{
    load_parameters(); 

    akm_drive_msg_stamp_system_ = 0.0;
    linear_velocity_previous_ = 0.0;

    akm_drive_suber_ = nh_.subscribe( akm_drive_subscribed_topic_name_, 1, &ClassNodeAckermannSim::akm_callback, this);
    odometry_puber_ = nh_.advertise<nav_msgs::Odometry>(odometry_published_topic_name_, 1, this);
    // states_puber_ = nh_.advertise<hawa_msgs::car_states>(states_published_topic_name_, 1, this);
    car_body_line_puber_ = nh_.advertise<visualization_msgs::Marker>(car_body_line_published_topic_name_, 1, this);

    periodic_timer_ = nh_.createTimer( ros::Duration(main_loop_timer_interval_second_), &ClassNodeAckermannSim::main_update, this );

    std::cout << "ClassNodeAckermannSim inti Done" << std::endl;
}

ClassNodeAckermannSim::~ClassNodeAckermannSim()
{
}


void ClassNodeAckermannSim::load_parameters()
{
    // cmdvel_subscribed_topic_name_ = "/cmd_vel";
    akm_drive_subscribed_topic_name_ = "/ackermann_cmd";
    odometry_published_topic_name_ = "/odometry";
    car_body_line_published_topic_name_ = "/car_body_line_marker";
    states_published_topic_name_ = "/sim_ego_car_state";
    
    map_frame_ = "/map";
    robot_frame_ = "/base_link";

    main_loop_timer_interval_second_ = 0.01;
    FLAG_pub_tf__map2odom_ = true;

    car_box_.set_frame_id(robot_frame_);
}



// void ClassNodeAckermannSim::cmdvel_callback(const geometry_msgs::TwistConstPtr &msg)
// {
//     // std::cout << "cmdvel_callback" << std::endl;
//     // cmdvel_msg_.cmdvel_msg_ptr = msg;
//     cmdvel_msg_.cmdvel_msg_.linear.x = msg->linear.x;
//     cmdvel_msg_.cmdvel_msg_.angular.z = msg->angular.z;
//     cmdvel_msg_.timestamp = helper_get_time_second();
// }

void ClassNodeAckermannSim::akm_callback(const ackermann_msgs::AckermannDriveStampedConstPtr &msg)
{
    akm_drive_msg_.header = msg->header;
    akm_drive_msg_.drive = msg->drive;
    akm_drive_msg_stamp_system_ = helper_get_time_second();
}

void ClassNodeAckermannSim::main_update( const ros::TimerEvent &event )
{
    // std::cout << "main_update" << std::endl;
    double time_now_second = helper_get_time_second();

    // std::cout << cmdvel_msg_.timestamp << " " << time_now_second << std::endl;
    // std::cout << cmdvel_msg_.cmdvel_msg_.linear.x << " " << cmdvel_msg_.cmdvel_msg_.angular.z << std::endl;
    
    double age_of_akm_drive_msg = time_now_second - akm_drive_msg_stamp_system_ ;

    if (age_of_akm_drive_msg < 0.3)
    {
        dyna_states_.set_target_linear_vb_mps(akm_drive_msg_.drive.speed);
        dyna_states_.set_target_steer_rad(akm_drive_msg_.drive.steering_angle);
    }
    else
    {
        dyna_states_.set_target_linear_vb_mps(0);
        dyna_states_.set_target_steer_rad(0);
    }

    double dt = time_now_second - timestamp_last_update_;

    dyna_states_.update_all_states( dt );

    timestamp_last_update_ = time_now_second;

    double linear_acc = (dyna_states_.linear_vb_mps_actual_ - linear_velocity_previous_) / dt;
    linear_velocity_previous_ = dyna_states_.linear_vb_mps_actual_;

    // publish robot states
    // std::cout << std::fixed << std::setprecision(2) << dt << "  " << dyna_states_.x_meter_ << " " << dyna_states_.y_meter_ << " " << dyna_states_.yaw_rad_ << " ";
    // std::cout << dyna_states_.linear_vb_mps_actual_<< "  " << dyna_states_.steer_rad_actual_ << std::endl;

    tf::Transform transform;
    tf::Quaternion q;

    if (FLAG_pub_tf__map2odom_)
    {
        transform.setOrigin( tf::Vector3(0, 0, 0) );
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));
    }
    
    transform.setOrigin( tf::Vector3(dyna_states_.x_meter_, dyna_states_.y_meter_, 0.0) );
    q.setRPY(0, 0, dyna_states_.yaw_rad_);
    transform.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

    transform.setOrigin( tf::Vector3(dyna_states_.axle_distance_, 0.0, 0.0) );
    q.setRPY(0, 0, dyna_states_.steer_rad_actual_);
    transform.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "steer_link"));

    visualization_msgs::Marker _body_line_marker = car_box_.get_marker_msg();
    car_body_line_puber_.publish(_body_line_marker);

    odometry_msg_.header.stamp = ros::Time::now();
    odometry_msg_.pose.pose.position.x = dyna_states_.x_meter_;
    odometry_msg_.pose.pose.position.y = dyna_states_.y_meter_;
    transform.setOrigin( tf::Vector3(0, 0, 0) );
    q.setRPY(0, 0, dyna_states_.yaw_rad_);
    odometry_msg_.pose.pose.orientation.x = q.getX();
    odometry_msg_.pose.pose.orientation.y = q.getY();
    odometry_msg_.pose.pose.orientation.z = q.getZ();
    odometry_msg_.pose.pose.orientation.w = q.getW();
    
    odometry_msg_.twist.twist.linear.x = dyna_states_.linear_vb_mps_actual_;
    odometry_msg_.twist.twist.linear.y = 0;
    odometry_msg_.twist.twist.angular.z = dyna_states_.angular_wb_radps_actual_;
    
    odometry_puber_.publish(odometry_msg_);

    // states_msg_.header.stamp = ros::Time::now();
    // states_msg_.car_id = "ego";
    // states_msg_.body_frame_velocity.twist.linear.x = dyna_states_.linear_vb_mps_actual_;
    // states_msg_.body_frame_velocity.twist.angular.z = dyna_states_.angular_wb_radps_actual_;
    // states_msg_.body_frame_accel.accel.linear.x = linear_acc;
    // states_msg_.steer_radius = dyna_states_.steer_rad_actual_;
    // states_msg_.pose.pose.position.x = dyna_states_.x_meter_;
    // states_msg_.pose.pose.position.y = dyna_states_.y_meter_;
    // states_msg_.heading_yaw_radius = dyna_states_.yaw_rad_;

    // states_puber_.publish(states_msg_);


}



#endif
