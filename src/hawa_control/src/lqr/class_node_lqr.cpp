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


#include "lqr/class_node_lqr.h"


namespace hawa
{

/**
 * @brief Constructor of the class.
*/
ClassNodeLQR::ClassNodeLQR() : Node("node_LQR")
{
    loadParameters();

    m_lqr_controller_ = std::make_unique<ClassLQRController>(m_axle_distance_, m_dt_, m_ricatti_max_iter_);

    m_ref_point_selector_ = std::make_unique<ClassLqrRefPointSelector>();

    m_segment_manager_ = std::make_unique<ClassHawaMultiSegmentManager>();

    m_current_path_segment_ = std::make_shared<ClassPath2DSegment>();

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
        m_topic_name__path_subscribed_, 10, std::bind(&ClassNodeLQR::pathCallback, this, std::placeholders::_1)
    );

    m_suber__odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        m_topic_name__odom_subscribed_, 10, std::bind(&ClassNodeLQR::odomCallback, this, std::placeholders::_1)
    );

    m_suber__pause_for_searching_ = this->create_subscription<std_msgs::msg::Bool>(
        m_topic_name__pause_for_searching_subscribed_, 10, std::bind(&ClassNodeLQR::pauseCallback, this, std::placeholders::_1)
    );

    m_periodic_controller_ = this->create_wall_timer(std::chrono::milliseconds(m_loop_time_ms_), 
                                                     std::bind(&ClassNodeLQR::controllerUpdate, this)
    );

    rosInfo("ClassNodeLQR inti Done");
    std::cout << std::setprecision(2) << std::fixed << std::endl;

    stopMotors();

    return;
}

/**
 * @breif Load parameters from the json file.
*/
void ClassNodeLQR::loadParameters()
{
    typedef boost::property_tree::ptree ptree;

    std::string cfg_path = "/home/jf/Hybrid_Astar_with_Ackermann/hawa_cfg.json";

    std::cout << "cfg path: >" << cfg_path << "< " << std::endl;

    ptree root;
    boost::property_tree::read_json(cfg_path, root);

    ptree topics = root.get_child("ros_topics");

    m_topic_name__ackerman_msg_ = topics.get<std::string>("ackerman_cmd_topic");                 //"/ackermann_cmd";
    m_topic_name__odom_subscribed_ = topics.get<std::string>("odometry_topic");                  //"/odometry";
    m_topic_name__path_subscribed_ = topics.get<std::string>("whole_path");                      //"/path";
    m_topic_name__pause_for_searching_subscribed_ = topics.get<std::string>("planner_searching");  
    m_topic_name__target_point_marker_ = topics.get<std::string>("controller_ref_point_vis");
    m_topic_name__current_path_published_ = topics.get<std::string>("controller_curr_path_vis");

    ptree motion_controller = root.get_child("motion_controller");

    m_pause_for_searching_.data = motion_controller.get<bool>("stop_when_in_obstacle_BOOL");
    
    ptree LQR = motion_controller.get_child("LQR");

    m_loop_time_ms_ = LQR.get<int>("loop_time_ms_INT");
    m_dt_ = LQR.get<double>("time_step_sec_FLOAT");
    m_ref_point_ahead_ = LQR.get<int>("ref_point_proximity_INT");
    m_ricatti_max_iter_ = LQR.get<int>("ricatti_max_iter_INT");

    ptree qmatrix = LQR.get_child("Q_FLOAT");

    m_Q_.at(0) = qmatrix.get<double>("q1");
    m_Q_.at(1) = qmatrix.get<double>("q2");
    m_Q_.at(2) = qmatrix.get<double>("q3");

    ptree rmatrix = LQR.get_child("R_FLOAT");

    m_R_.at(0) = rmatrix.get<double>("r1");
    m_R_.at(1) = rmatrix.get<double>("r2");

    ptree robot_specs = root.get_child("robot_specs");

    m_expected_speed_mps_ = robot_specs.get<double>("max_speed_mps");
    m_axle_distance_ = robot_specs.get<double>("axle_distance_metric");
    m_max_steering_angle_ = robot_specs.get<double>("max_steer_angle");

    m_map_received_ = false;
}

/**
 * @brief Print out the info message.
 * @param str the message to be printed.
*/
void ClassNodeLQR::rosInfo(const std::string& str)
{
    RCLCPP_INFO(this->get_logger(), str.c_str());
}

/**
 * @brief Print out the warning message.
*/
void ClassNodeLQR::rosWarn(const std::string& str, const int t)
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
        rosInfo("ClassNodeLQR::pathCallback(), Path msg size < 2.  Do not proceed.");
        return;
    }
    m_mutex_path_.lock();
    m_segment_manager_->setPath(msg);
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
 * @brief For updating the value of the grid map.
*/
void ClassNodeLQR::gridMapCallback(const nav_msgs::msg::OccupancyGrid &msg)
{
    m_mutex_map_.lock();
    m_map_msg_.data = msg.data;
    m_map_msg_.header = msg.header;
    m_map_msg_.info = msg.info;
    m_map_received_ = true;
    m_mutex_map_.unlock();
}

/**
 * @brief Main function to be called by timer regualerly.
 * @param event ros timer event.
*/
void ClassNodeLQR::controllerUpdate()
{
    if (m_pause_for_searching_.data)
    {
        rosWarn("Pause while searching path.");
        return;
    }

    if( ! validateOdomMsg() )
    {
        rosWarn("Invalid robot odom msg.  Do not proceed.");
        return;
    }

    m_mutex_path_.lock();

    // ------------------------- first, update the path manager -------------------------

    m_segment_manager_->update(m_robot_odom_msg_.pose.pose.position.x, 
                              m_robot_odom_msg_.pose.pose.position.y,
                              Tools::quaternionToEularYaw(m_robot_odom_msg_.pose.pose.orientation));
    if (m_segment_manager_->doesPathExist())
    {
        auto temp = m_segment_manager_->getCurrentSegment();
        m_current_path_segment_ = std::make_shared<ClassPath2DSegment>(temp);
        m_ref_point_selector_->setPathPtr(m_current_path_segment_);
        nav_msgs::msg::Path _ros_current_path = m_segment_manager_->getCurrentSegment().toRosPath();
        _ros_current_path.header = m_segment_manager_->getOriginalPath().header;
        m_puber__current_path_->publish(_ros_current_path);

        
    }

    if (m_segment_manager_->didFinishAll())
    {
        std::cout << "m_segment_manager_  Finished all segments." << std::endl;
        stopMotors();
        m_mutex_path_.unlock();
        return;
    }


    // ------------------------- second, find the reference point -------------------------

    m_ref_point_selector_->setGoingForward(m_current_path_segment_->isForward());

    ClassPose2D _robot_pose(m_robot_odom_msg_.pose.pose.position.x, 
                            m_robot_odom_msg_.pose.pose.position.y, 
                            Tools::quaternionToEularYaw(m_robot_odom_msg_.pose.pose.orientation));

    m_ref_point_selector_->setRobotPose(_robot_pose);
    m_ref_point_selector_->process();
    
    double _signed_speed_mps = decideSpeedMps();

    // ------------------------- third, solve the LQR controller -------------------------
    
    m_lqr_controller_->setVref(_signed_speed_mps);
    m_lqr_controller_->setSteerRef(0);

    m_lqr_controller_->setStateRef(m_ref_point_selector_->getRefPose()); 
    m_lqr_controller_->setState(_robot_pose);
    
    m_lqr_controller_->generateQ(m_Q_.at(0), m_Q_.at(1), m_Q_.at(2));
    m_lqr_controller_->generateR(m_R_.at(0), m_R_.at(1));

    m_lqr_controller_->solve();

    auto steer = m_lqr_controller_->getSteer();
    auto speed = m_lqr_controller_->getV();

    // ---------------- fourth, post-process and publish the control command -------------------------

    steer = std::min(steer, m_max_steering_angle_);
    steer = std::max(steer, -m_max_steering_angle_);

    speed = std::min(speed, m_expected_speed_mps_);
    speed = std::max(speed, -m_expected_speed_mps_);

    ackermann_msgs::msg::AckermannDriveStamped  _akm_cmd_msg;
    _akm_cmd_msg.drive.speed = speed;
    _akm_cmd_msg.drive.steering_angle = steer;
    _akm_cmd_msg.header.stamp = this->get_clock()->now();

    m_puber__ackerman_msg_->publish(_akm_cmd_msg);

    visulizeOnePoint(m_ref_point_selector_->getRefPose());

    m_mutex_path_.unlock();
}

/**
 * @brief Check if the robot odom msg is up to date. 
 * @return true when valid;  false when invalid.
*/
bool ClassNodeLQR::validateOdomMsg()
{
    auto t_now = this->get_clock()->now().seconds();
    auto t_msg = m_robot_odom_msg_.header.stamp.sec;
    if( t_now - t_msg > 1.0 )
    {
        return false;
    }
    return true;
}

/**
 * @brief When robot is approaching the end of the segment, the speed should reduce for a smoother
 * transition. This function calculates the speed based on the distance between the robot and the 
 * end of the segment. 
 * @return the speed in unit of meter per second.
*/
double ClassNodeLQR::calcSpeedRampping()
{
    double _dist_to_end = m_segment_manager_->getDistToEnd();
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

    if (m_segment_manager_->didFinishAll())
    {
        _spd_val = 0;
    }
    return _spd_val;
}

/**
 * @brief Visualize a point on the map.
*/
void ClassNodeLQR::visulizeOnePoint(const ClassPose2D& pt, const bool draw_arrow, const bool draw_sphere)
{
    double a = 0.7; // Don't forget to set the alpha!
    double r = 0.0;
    double g = 0.8;
    double b = 0.5;

    if (draw_sphere)
    {
        visualization_msgs::msg::Marker _target_point_marker;
        _target_point_marker.header.frame_id = "map";
        _target_point_marker.action = visualization_msgs::msg::Marker::MODIFY;
        
        _target_point_marker.pose.position.x = pt.x;
        _target_point_marker.pose.position.y = pt.y;
        _target_point_marker.type = visualization_msgs::msg::Marker::SPHERE;
        _target_point_marker.scale.x = 0.1;
        _target_point_marker.scale.y = 0.1;
        _target_point_marker.scale.z = 0.1;

        _target_point_marker.color.a = a;
        _target_point_marker.color.r = r;
        _target_point_marker.color.g = g;
        _target_point_marker.color.b = b;
        m_puber__target_point_marker_->publish(_target_point_marker);
    }
    
    if (draw_arrow)
    {
        visualization_msgs::msg::Marker _target_point_marker;
        _target_point_marker.header.frame_id = "map";
        _target_point_marker.action = visualization_msgs::msg::Marker::MODIFY;

        geometry_msgs::msg::Point _start_point;
        _start_point.x = pt.x;
        _start_point.y = pt.y;
        _target_point_marker.points.push_back(std::move(_start_point));

        geometry_msgs::msg::Point _end_point;
        _end_point.x = pt.x + 0.5 * std::cos(pt.yaw);
        _end_point.y = pt.y + 0.5 * std::sin(pt.yaw);
        _target_point_marker.points.push_back(std::move(_end_point));

        _target_point_marker.scale.x = 0.03;
        _target_point_marker.scale.y = 0.1;
        _target_point_marker.scale.z = 0.1;

        _target_point_marker.color.a = a;
        _target_point_marker.color.r = r;
        _target_point_marker.color.g = g;
        _target_point_marker.color.b = b;
        m_puber__target_point_marker_->publish(_target_point_marker);
    }
}

/**
 * @brief Stop the motors.
*/
void ClassNodeLQR::stopMotors()
{
    ackermann_msgs::msg::AckermannDriveStamped  _akm_cmd_msg;
    _akm_cmd_msg.drive.speed = 0;
    _akm_cmd_msg.drive.steering_angle = 0;
    _akm_cmd_msg.header.stamp = this->get_clock()->now();
    m_puber__ackerman_msg_->publish(_akm_cmd_msg);
}

/**
 * @brief Check if the robot is in the obstacle.
 * @return true when in obstacle; false when not in obstacle.
 * TODO: implement this function.
*/
bool ClassNodeLQR::checkRobotInObstacle()
{
    return false;
}

} // namespace hawa
