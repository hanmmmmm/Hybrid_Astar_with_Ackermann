
#include "class_node_path_planner.h"


namespace hawa
{

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

    m_publisher_searching_  = this->create_publisher<std_msgs::msg::Bool>(
        m_topic_name_searching_published_, 10
    );

    m_subscriber_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        m_topic_name_map_subscribed_, 10, std::bind(&ClassPathPlanner::mapCallback, this, std::placeholders::_1)
    );

    m_subscriber_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        m_topic_name_goal_subscribed_, 10, std::bind(&ClassPathPlanner::goalCallback, this, std::placeholders::_1)
    );

    m_subscriber_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        m_topic_name_odom_subscribed_, 10, std::bind(&ClassPathPlanner::odomCallback, this, std::placeholders::_1)
    );

    m_periodic_timer_ = this->create_wall_timer(200ms, std::bind(&ClassPathPlanner::pathPlan, this));

    m_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_);

    m_ha_planner_ = std::make_unique<ClassHybridAStar>();
    m_path_validator_ = std::make_unique<ClassPathValidator>();

    m_path_validator_->setMapHandler(m_ha_planner_->m_gridmap_handler_ptr_);

    m_ha_planner_->m_RS_curve_finder_.setSamplingProperties(m_rs_sampling_properites_);

    ros_info("ClassPathPlanner inti Done.");
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
    typedef boost::property_tree::ptree ptree;

    std::string cfg_path = "/home/jf/Hybrid_Astar_with_Ackermann/hawa_cfg.json";

    std::cout << "cfg path: >" << cfg_path << "< " << std::endl;

    ptree root;
    boost::property_tree::read_json(cfg_path, root);

    ptree topics = root.get_child("ros_topics");

    m_topic_name_map_subscribed_ = topics.get<std::string>("map_from_map_processor");      //"/map_fusion";
    m_topic_name_goal_subscribed_ = topics.get<std::string>("goal_topic");                 //"/goal_pose";
    m_topic_name_odom_subscribed_ = topics.get<std::string>("odometry_topic");             //"/odometry";
    m_topic_name_path_published_ = topics.get<std::string>("whole_path");                  //"/path";
    m_topic_name_searching_published_ = topics.get<std::string>("planner_searching");      //"/planner_searching";

    ptree frames = root.get_child("frames");

    m_map_tf_frame_name_ = frames.get<std::string>("map_frame");     // "/map";
    m_robot_tf_frame_name_ = frames.get<std::string>("base_frame");  // "/base_link";

    ptree planner = root.get_child("path_planer");

    m_path_plan_timeout_ms_ = planner.get<int>("timeout_ms_INT");
    FLAG_wait_before_replan_ = planner.get<bool>("wait_for_stop_before_replan_BOOL");

    ptree rs_curves = planner.get_child("ReedsShepp");

    m_rs_sampling_properites_.turning_radius = rs_curves.get<double>("turning_radius_metric");
    m_rs_sampling_properites_.angular_step_size = rs_curves.get<double>("angle_step");
    m_rs_sampling_properites_.linear_step_size = rs_curves.get<double>("linear_step");

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

    double t2 = ClassHawaTimer::getTimeSecs();

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
    std_msgs::msg::Bool msg;
    msg.data = false;
    m_publisher_searching_->publish(msg);

    // --------------- Check if everything is ready ----------------

    if( ! m_map_received_ ) return;
    if( ! m_goal_received_ ) return;

    checkPath();

    if (m_goal_solved_) return;

    msg.data = true;
    m_publisher_searching_->publish(msg);

    if (! checkRobotStop()) return;

    // --------------- Start path planning ----------------

    ros_info("ClassPathPlanner::path_plan() start");

    double t1 = ClassHawaTimer::getTimeSecs();

    m_map_mutex_.lock();
    m_goal_mutex_.lock();

    getRobotPoseInMapFrame();

    bool _planner_set_map_success = m_ha_planner_->setMap(&m_map_msg_);
    bool _planner_set_pose_sucess = m_ha_planner_->setStartGoalPoses(m_start_pose_, m_goal_pose_);
    bool _planner_setup_succes = m_ha_planner_->setup();

    if (! (_planner_setup_succes || _planner_set_map_success || _planner_set_pose_sucess))
    {
        exit_pathplan_function(t1);
        return;
    }

    bool found_path = m_ha_planner_->search();

    if(! found_path )
    {
        exit_pathplan_function(t1);
        return;
    }

    // --------------- Done searching. Get the path ----------------

    ClassCustomPathContainer path;
    m_ha_planner_->getFinalHybridAstarPath(path);

    m_goal_solved_ = true;

    m_navmsgs_path_msg_.header.frame_id = m_map_tf_frame_name_;

    m_navmsgs_path_msg_.poses.clear();
    geometry_msgs::msg::PoseStamped one_pose;

    for( auto point : path.getPath() )
    {
        one_pose.pose.position.x = point[0] + m_map_msg_.info.origin.position.x;
        one_pose.pose.position.y = point[1] + m_map_msg_.info.origin.position.y;
        one_pose.pose.position.z = 0.0;
        one_pose.pose.orientation = ClassUtilsConverters::tf2quaToGeoQua(ClassUtilsConverters::twodYawToTf2qua(point[2]));

        m_navmsgs_path_msg_.poses.push_back(one_pose);
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
        m_tf_robot_to_map_ = m_tf_buffer_->lookupTransform(m_map_tf_frame_name_, 
                                                           m_robot_tf_frame_name_, 
                                                           tf2::TimePointZero);

        m_start_pose_.x = m_tf_robot_to_map_.transform.translation.x - m_map_msg_.info.origin.position.x;
        m_start_pose_.y = m_tf_robot_to_map_.transform.translation.y - m_map_msg_.info.origin.position.y;
        m_start_pose_.yaw = ClassUtilsConverters::StampedTransformToYaw(&m_tf_robot_to_map_);
    }
    catch (tf2::TransformException const& ex){
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
    m_goal_pose_.yaw = ClassUtilsConverters::geoQuaToYaw(&(msg.pose.orientation));

    m_goal_received_ = true;
    m_goal_solved_ = false;

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
 * @brief ROS callback frunction for the odometry message. 
 * @param msg message of the rostopic
*/
void ClassPathPlanner::odomCallback(const nav_msgs::msg::Odometry &msg)
{
    m_odom_mutex_.lock();
    m_odom_msg_ = msg;
    m_robot_linear_velocity_ = m_odom_msg_.twist.twist.linear.x;
    m_odom_mutex_.unlock();
}

bool ClassPathPlanner::checkRobotStop()
{
    return std::abs(m_robot_linear_velocity_) < 0.1;
}


/**
 * @brief Check if the current path is still valid.
*/
void ClassPathPlanner::checkPath()
{
    if (! m_goal_solved_) return;

    m_path_validator_->setPath(m_navmsgs_path_msg_);
    m_path_validator_->setRobotPose(m_tf_robot_to_map_);

    if (! m_path_validator_->validate())
    {
        m_goal_solved_ = false;
        std::cerr << "Path is no longer valid. Need to replan." << std::endl;
    }

}




}


