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



#include "hawa_ackermann_sim/class_node_ackermann_simulation.h"



namespace hawa
{


ClassNodeAckermannSim::ClassNodeAckermannSim(): Node("AckermannSim")
{
    loadParameters(); 

    m_akm_drive_msg_stamp_system_ = 0.0;

    m_tf_bcr_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    m_sub_ptr_akm_drive_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        m_topic_name_akm_drive_, 10, std::bind(&ClassNodeAckermannSim::akmDriveCallback, this, _1)
    );

    m_pub_ptr_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>(
        m_topic_name_odometry_, 10
    );

    m_pub_ptr_car_body_vis_ = this->create_publisher<visualization_msgs::msg::Marker>(
        m_topic_name_car_body_vis_, 10
    );

    m_periodic_timer_ = this->create_wall_timer(std::chrono::milliseconds(m_main_loop_timer_interval_millisecond_), 
                                                std::bind(&ClassNodeAckermannSim::mainUpdate, this));

    m_noise_generator_ = std::make_unique<ClassRandomSinNoiseGenerator>();

    RCLCPP_INFO(this->get_logger(), "ClassNodeAckermannSim inti Done");
}


/**
 * @brief Default Destructor
*/
ClassNodeAckermannSim::~ClassNodeAckermannSim()
{
}


/**
 * @brief Set the ros topic names and tf frames. In the future, these parameters will be loaded from 
 * a json file, so it will be more reconfigurable. 
*/
void ClassNodeAckermannSim::loadParameters()
{
    typedef boost::property_tree::ptree ptree;

    std::string cfg_path = "/home/jf/Hybrid_Astar_with_Ackermann/hawa_cfg.json";

    std::cout << "cfg path: >" << cfg_path << "< " << std::endl;

    ptree root;
    boost::property_tree::read_json(cfg_path, root);

    ptree topics = root.get_child("ros_topics");

    m_topic_name_akm_drive_ = topics.get<std::string>("ackerman_cmd_topic");            //"/ackermann_cmd";
    m_topic_name_odometry_ = topics.get<std::string>("odometry_topic");                 //"/odometry";
    m_topic_name_car_body_vis_ = topics.get<std::string>("robot_body_visualization");   //"/car_body_line_marker";
    
    ptree frames = root.get_child("frames");

    m_robot_frame_ = frames.get<std::string>("base_frame");        //"/base_link";
    m_steer_link_frame_ = frames.get<std::string>("steer_frame"); 
    m_map_frame_ = frames.get<std::string>("map_frame"); 
    m_odom_frame_ = frames.get<std::string>("odom_frame"); 

    ptree simulation = root.get_child("simulation");

    m_main_loop_timer_interval_millisecond_ = simulation.get<int>("loop_time_ms_INT");  //20;
    m_cmd_timeout_sec_ = simulation.get<double>("cmd_timeout_sec_FLOAT");               //0.3;
    FLAG_use_noise_ = simulation.get<bool>("use_noise_BOOL");                           //true;
    FLAG_pub_tf__map2odom_ = simulation.get<bool>("pub_tf_map2odom_BOOL");              //true;

    m_car_box_.setFrameID(m_robot_frame_);

    ptree robot_specs = root.get_child("robot_specs");
    double axle_distance_metric = robot_specs.get<double>("axle_distance_metric");
    double max_speed_mps = robot_specs.get<double>("max_speed_mps");
    double max_steer_angle = robot_specs.get<double>("max_steer_angle");
    double max_acceleration_mps2 = robot_specs.get<double>("max_acceleration_mps2");
    double max_steer_rate_radps = robot_specs.get<double>("max_steer_rate_radps");

    m_dyna_states_ = std::make_unique<DynamicsStates>(axle_distance_metric, max_speed_mps, max_acceleration_mps2
                                            , max_steer_angle, max_steer_rate_radps);
}

/**
 * @brief Callback function to the ackermann drive message input.
*/
void ClassNodeAckermannSim::akmDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped &msg)
{
    // RCLCPP_INFO(this->get_logger(), "akmDriveCallback()");
    m_akm_drive_msg_.header = msg.header;
    m_akm_drive_msg_.drive = msg.drive;
    
    // auto tools = ClassHawaSimTools();
    m_akm_drive_msg_stamp_system_ = ClassHawaSimTools::getTimeSecond();
}

/**
 * @brief The main function in class. To be called by the ros timer event. 
*/
void ClassNodeAckermannSim::mainUpdate()
{
    double _time_now_second = ClassHawaSimTools::getTimeSecond();

    double _age_of_akm_drive_msg = _time_now_second - m_akm_drive_msg_stamp_system_ ;

    // stop the robot when the command input is out of date. 
    if (_age_of_akm_drive_msg < m_cmd_timeout_sec_)
    {
        m_dyna_states_->setTargetLinearVbMps(m_akm_drive_msg_.drive.speed);
        m_dyna_states_->setTargetSteerRad(m_akm_drive_msg_.drive.steering_angle);
    }
    else
    {
        m_dyna_states_->setTargetLinearVbMps(0);
        m_dyna_states_->setTargetSteerRad(0);
    }

    double _delta_t = _time_now_second - m_timestamp_last_update_;

    m_dyna_states_->updateAllStates(_delta_t);

    m_timestamp_last_update_ = _time_now_second;

    auto state_x = m_dyna_states_->m_x_meter_;
    auto state_y = m_dyna_states_->m_y_meter_;
    auto state_yaw = m_dyna_states_->m_yaw_rad_;
    // apply the noise to the states
    if (FLAG_use_noise_)
    {
        double _x_noise, _y_noise, _yaw_noise;
        m_noise_generator_->getNoise(_x_noise, _y_noise, _yaw_noise);
        state_x += _x_noise;
        state_y += _y_noise;
        state_yaw += _yaw_noise;
    }
    // done applying noise

    // prepare and publish the tf and odometry message

    geometry_msgs::msg::TransformStamped _transform;

    _transform.header.stamp = this->get_clock()->now();
   
    if (FLAG_pub_tf__map2odom_)
    {
        _transform.header.frame_id = m_map_frame_;
        _transform.child_frame_id = m_odom_frame_;
        _transform.transform.translation.x = 0;
        _transform.transform.translation.y = 0;
        _transform.transform.translation.z = 0;
        _transform.transform.rotation.x = 0;
        _transform.transform.rotation.y = 0;
        _transform.transform.rotation.z = 0;
        _transform.transform.rotation.w = 1;
        m_tf_bcr_->sendTransform(_transform);
    }
    
    _transform.header.frame_id = m_odom_frame_;
    _transform.child_frame_id = m_robot_frame_;
    _transform.transform.translation.x = state_x;
    _transform.transform.translation.y = state_y;
    _transform.transform.translation.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, state_yaw);
    _transform.transform.rotation.x = q.x();
    _transform.transform.rotation.y = q.y();
    _transform.transform.rotation.z = q.z();
    _transform.transform.rotation.w = q.w();
    m_tf_bcr_->sendTransform(_transform);

    _transform.header.frame_id = m_robot_frame_;
    _transform.child_frame_id = m_steer_link_frame_;
    _transform.transform.translation.x = m_dyna_states_->m_axle_distance_;
    _transform.transform.translation.y = 0;
    _transform.transform.translation.z = 0;

    q.setRPY(0, 0, m_dyna_states_->m_steer_rad_actual_);
    _transform.transform.rotation.x = q.x();
    _transform.transform.rotation.y = q.y();
    _transform.transform.rotation.z = q.z();
    _transform.transform.rotation.w = q.w();
    m_tf_bcr_->sendTransform(_transform);

    visualization_msgs::msg::Marker _body_line_marker = m_car_box_.getMarkerMsg();
    m_pub_ptr_car_body_vis_->publish(_body_line_marker);

    m_odometry_msg_.header.stamp = this->get_clock()->now();
    m_odometry_msg_.pose.pose.position.x = state_x;
    m_odometry_msg_.pose.pose.position.y = state_y;
    // _transform.setOrigin(tf::Vector3(0, 0, 0));
    q.setRPY(0, 0, state_yaw);
    m_odometry_msg_.pose.pose.orientation.x = q.getX();
    m_odometry_msg_.pose.pose.orientation.y = q.getY();
    m_odometry_msg_.pose.pose.orientation.z = q.getZ();
    m_odometry_msg_.pose.pose.orientation.w = q.getW();
    
    m_odometry_msg_.twist.twist.linear.x = m_dyna_states_->m_linear_vb_mps_actual_;
    m_odometry_msg_.twist.twist.linear.y = 0;
    m_odometry_msg_.twist.twist.angular.z = m_dyna_states_->m_angular_wb_radps_actual_;
    
    m_pub_ptr_odometry_->publish(m_odometry_msg_);

    
}


} // namespace hawa
