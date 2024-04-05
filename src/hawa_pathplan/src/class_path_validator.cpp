
#include "class_path_validator.h"


namespace hawa
{

const double ClassPathValidator::k_pi2_ = M_PI * 2.0;

/**
 * @brief Default constructor.
*/
ClassPathValidator::ClassPathValidator()
{
    m_distance_tolerance_ = 0.05;
    m_angle_tolerance_ = M_PI / 6.0;

    m_map_occ_threshold_.plan = 60;
    m_map_occ_threshold_.vali = 50;

    m_yaw_angle_bin_size_ = 0.1745329;
}

/**
 * @brief Default destructor.
*/
ClassPathValidator::~ClassPathValidator()
{
}

/**
 * @brief Set path pointer.
 * @param path 
*/
void ClassPathValidator::setPath(const nav_msgs::msg::Path& path)
{
    m_path_ptr_ = std::make_shared<nav_msgs::msg::Path>(path);
}

/**
 * @brief Set the Map object.
*/
void ClassPathValidator::setMap(nav_msgs::msg::OccupancyGrid * ptr_map)
{
    m_gridmap_handler_.setGridMapPtr(&(ptr_map->data));
    m_gridmap_handler_.setGridWidthHeight(ptr_map->info.width, ptr_map->info.height);
    m_gridmap_handler_.setPlanningObstacleThreshold(m_map_occ_threshold_.plan);
    m_gridmap_handler_.setValidateObstacleThreshold(m_map_occ_threshold_.vali);
    m_gridmap_handler_.setGridMeterRatio(ptr_map->info.resolution, m_yaw_angle_bin_size_);
}


/**
 * @brief Set the Robot Pose object.
*/
void ClassPathValidator::setRobotPose(const geometry_msgs::msg::TransformStamped& robot_pose)
{
    m_robot_pose_.x = robot_pose.transform.translation.x;
    m_robot_pose_.y = robot_pose.transform.translation.y;
    m_robot_pose_.yaw = quaternionToYaw(robot_pose.transform.rotation);
}

/**
 * @brief Check if the path is still valid.
 * @return true 
 * @return false 
*/
bool ClassPathValidator::validate()
{
    if (m_path_ptr_ == nullptr)
    {
        std::cerr << "Path is not set. nullptr." << std::endl;
        return false;
    }

    if (m_path_ptr_->poses.size() <= 1)
    {
        std::cerr << "Path too short: " << m_path_ptr_->poses.size() << std::endl;
        return false;
    }

    // if (! checkRobotNearPath())
    // {
    //     return false;
    // }

    if (! checkPathIsClear())
    {
        std::cerr << "Path collides." << std::endl;
        return false;
    }

    return true;
}

/**
 * @brief Check if the path is clear.
 * @return true 
*/
bool ClassPathValidator::checkPathIsClear()
{
    StructPoseGrid _pose_grid;
    StructPoseReal _pose_real;
    for (auto ps : m_path_ptr_->poses)
    {
        _pose_real.x = ps.pose.position.x;
        _pose_real.y = ps.pose.position.y;
        _pose_real.yaw = quaternionToYaw(ps.pose.orientation);

        m_gridmap_handler_.convertFinePoseToGrid(_pose_real, _pose_grid);

        if (m_gridmap_handler_.checkGridWithinMap(_pose_grid.x, _pose_grid.y))
            continue;
        
        bool step_is_clear = m_gridmap_handler_.checkGridClear(_pose_grid.x, _pose_grid.y, ClassGridMapHandler::EnumMode::plan);
        if (! step_is_clear)
        {
            return false;
        }
    }
    return true;
}


// bool ClassPathValidator::checkRobotNearPath()
// {
//     // vector< array<double, 3> > points_near_robot;
//     for (auto pt : *path_ptr_)
//     {
//         double distance = euclidean_distance(pt[0], pt[1], robot_x_, robot_y_);
//         if (distance <= distance_tolerance_)
//         {
//             if (min_angle_diffrence(robot_yaw_, pt[2]) < angle_tolerance_)
//             {
//                 // points_near_robot.push_back( pt );
//                 return true;
//             }
//         }
//     }
//     return false;
// }


inline double ClassPathValidator::euclidean_distance(double x1, double y1, double x2, double y2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}


double ClassPathValidator::min_angle_diffrence(double a1, double a2)
{
    a1 = mod_2pi(a1);
    a2 = mod_2pi(a2);
    double amax, amin, diff;
    if (a1 > a2)
    {
        amax = a1;
        amin = a2;
    }
    else if (a2 > a1)
    {
        amax = a2;
        amin = a1;
    }
    else
    {
        return 0.0;
    }

    diff = amax - amin;
    if (diff > M_PI)
    {
        diff = k_pi2_ - diff;
    }

    return diff;
}


double ClassPathValidator::mod_2pi(double angle)
{
    double a_out = std::remainder(angle, k_pi2_);
    if (a_out > 0.0)
    {
        return a_out;
    }
    else
    {
        return a_out + k_pi2_;
    }
}


double ClassPathValidator::quaternionToYaw(const geometry_msgs::msg::Quaternion& q_msg) 
{
    Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    return euler[2];
}



} // namespace hawa

