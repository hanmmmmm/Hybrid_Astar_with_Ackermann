#ifndef CLASS_PUREPURSUIT
#define CLASS_PUREPURSUIT


/**
 * @file class_purepursuit.h
 * @author Mingjie
 * @brief This is the implementation of the pure pursuit algorithm.
            The other parts like testing and ros-node are wrappers of this class.
 * @version 0.1
 * @date 2023-04-04
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#include <iostream>
#include <vector>
#include <array>
#include <math.h>

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point.h"
#include "../common/class_elemental_path2d_segment.h"
#include "../common/class_elemental_pose2d.h"
#include "../common/tools_angles.h"

/// @brief compute the Euclidean distance between 2 points. 
/// @param x1 
/// @param y1 
/// @param x2 
/// @param y2 
/// @return distance in meter.
inline double compute_distance_meter(double x1, double y1, double x2, double y2)
{
    return sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) );
}


/// @brief For hodling the parameters for this algorithm.
struct StructParameters
{
    double target_longitude_speed_mps_;  // meter per second
    double look_ahead_distance_meter_;
    double look_ahead_coefficient_;

    double k_max_target_speed_mps_;
    double k_max_look_ahead_coefficient_;
    double k_min_wheelbase_meter_;
    double k_max_wheelbase_meter_;

    double wheelbase_meter_;  // wheelbase is the distance between the centers of the front and rear tires on a vehicle.
};


/// @brief For holding the values of robot pose.
struct StructPose
{
    double x_meter;
    double y_meter;
    double yaw_rad;
};


/// @brief The main componenet of the pure pursuit algorithm.
class ClassPurePursuit
{
private:
    StructParameters parameters_;

    std::vector<std::array<geometry_msgs::Point,2>> all_possible_target_points_;
    geometry_msgs::PointStamped target_points_;

    StructPose robot_pose_;

    ClassPath2DSegment path_;

    // bool m_finish_;

private:
    void compute_look_ahead_distnace();
    
    void choose_the_proper_target_point();


public:
    ClassPurePursuit();
    ~ClassPurePursuit();

    bool set_vehicle_wheelbase_meter(double val);  

    bool set_target_linear_speed_mps(double val);

    bool set_look_ahead_coefficient(double val);

    void set_path_segment(ClassPath2DSegment& path);

    void set_robot_pose(double x_meter, double y_meter, double yaw_rad);

    void find_target_point(bool& success);

    void get_target_point(geometry_msgs::PointStamped &target);

    void solve_for_speed_command(bool &success, double &steer_rad);

    void find_precise_target_point(std::array<geometry_msgs::Point, 2> two_pathpoints, geometry_msgs::PointStamped &result_point);

    bool isFinish();
};


ClassPurePursuit::ClassPurePursuit()
{
    parameters_.k_max_target_speed_mps_ = 3.0;
    parameters_.k_max_look_ahead_coefficient_ = 30.0;
    parameters_.k_min_wheelbase_meter_ = 0.05;
    parameters_.k_max_wheelbase_meter_ = 0.5;
    // m_finish_ = false;
}

ClassPurePursuit::~ClassPurePursuit()
{
}


/// @brief External use. To set the path_segment
/// @param path reference of a ClassPath2DSegment
void ClassPurePursuit::set_path_segment(ClassPath2DSegment& path)
{
    std::cout << "PurePursuit get new path." << std::endl;
    path_ = path;
    // m_finish_ = false;

    // ROS_INFO_STREAM("set_path_segment  m_finish_ " << m_finish_);
}


/// @brief External use. To set the value of the robot_pose
/// @param x_meter 
/// @param y_meter 
/// @param yaw_rad 
void ClassPurePursuit::set_robot_pose(double x_meter, double y_meter, double yaw_rad)
{
    robot_pose_.x_meter = x_meter;
    robot_pose_.y_meter = y_meter;
    robot_pose_.yaw_rad = mod_2pi(yaw_rad);
}


/// @brief External use. To set the value of the target_linear_speed_mps
/// @param val double in meter per second
/// @return bool. False when the input value exceeds the limits, and the value will not be updated.
bool ClassPurePursuit::set_target_linear_speed_mps(double val)
{
    if( std::abs(val) < parameters_.k_max_target_speed_mps_ )
    {
        parameters_.target_longitude_speed_mps_ = val;
        return true;
    }
    return false;
}


/// @brief External use. To set the value of the look_ahead_coefficient
/// @param val : positive double 
/// @return bool. False when the input value exceeds the limits, and the value will not be updated.
bool ClassPurePursuit::set_look_ahead_coefficient(double val)
{
    if( (0 < val) && (val < parameters_.k_max_look_ahead_coefficient_) )
    {
        parameters_.look_ahead_coefficient_ = val;
        return true;
    }
    return false;
}


/// @brief External use. To set the value of the wheel base. It's the distance between the axles of the 
/// front wheel and rear wheel.
/// @param val : double in meter.
/// @return bool. False when the input value exceeds the limits, and the value will not be updated.
bool ClassPurePursuit::set_vehicle_wheelbase_meter(double val)
{
    if( (val > parameters_.k_min_wheelbase_meter_) && (val < parameters_.k_max_wheelbase_meter_) )
    {
        parameters_.wheelbase_meter_ = val;
        return true;
    }
    return false;
}


/// @brief After the linear speed and look_ahead_coefficient are set, call this func to compute the L_d.
void ClassPurePursuit::compute_look_ahead_distnace()
{
    parameters_.look_ahead_distance_meter_ = std::abs(parameters_.look_ahead_coefficient_ * parameters_.target_longitude_speed_mps_);
    // std::cout << "look_ahead_coefficient_ = " << parameters_.look_ahead_coefficient_ << std::endl;
    // std::cout << "target_longitude_speed_mps_ = " << parameters_.target_longitude_speed_mps_ << std::endl;
    // std::cout << "look_ahead_distance_meter_ = " << parameters_.look_ahead_distance_meter_ << std::endl;
}


/// @brief One of the main functions to be called. After all parameters are updated, call this func to 
/// find the target_point from the path. This function is possible to fail finding the target_point when
/// robot is significantly far away from the path.
/// @param success : bool
void ClassPurePursuit::find_target_point(bool& success)
{
    // std::cout << "find_target_point()" << std::endl;
    // ROS_INFO_STREAM("find_target_point()1  m_finish_ " << m_finish_ );

    int _num_points = path_.path_segment__extended_.size();
    if(_num_points <= 2)
    {
        std::cout << "find_target_point()  Path too short: " << _num_points << std::endl;
        success = false;
        return;
    }
    std::cout << "find_target_point()  Path length okay. " << _num_points << std::endl;

    compute_look_ahead_distnace();
    all_possible_target_points_.clear();

    for(int i=0; i<_num_points-1; i++)
    {
        ClassPose2D _pose_1 = path_.path_segment__extended_.at(i);
        ClassPose2D _pose_2 = path_.path_segment__extended_.at(i+1);
        double dist_1 = compute_distance_meter(_pose_1.x, _pose_1.y, robot_pose_.x_meter, robot_pose_.y_meter);
        double dist_2 = compute_distance_meter(_pose_2.x, _pose_2.y, robot_pose_.x_meter, robot_pose_.y_meter);

        geometry_msgs::Point _p1;
        _p1.x = _pose_1.x;
        _p1.y = _pose_1.y;
        
        geometry_msgs::Point _p2;
        _p2.x = _pose_2.x;
        _p2.y = _pose_2.y;

        if( (dist_1 < parameters_.look_ahead_distance_meter_) && (dist_2 > parameters_.look_ahead_distance_meter_) )
        {
            all_possible_target_points_.push_back(std::array<geometry_msgs::Point, 2> {_p1, _p2});
        }

    }

    if(all_possible_target_points_.size() == 0)
    {
        std::cout << "No Target point found." << std::endl;
        /*
        TODO:
        expand searching area and try again.
        */
    }
    else if(all_possible_target_points_.size() >= 2)
    {
        std::cout << " >= 2 Target points found." << std::endl;
        /*
        TODO:
        select the best one.
        */
    }
    else if(all_possible_target_points_.size() == 1)
    {
        std::cout << " == 1 Target point found." << std::endl;
        success = true;
        find_precise_target_point(all_possible_target_points_[0], target_points_);
        return;
    }
    success = false;

    // double _dist_to_end = compute_distance_meter(path_.path_segment__original_.back().x, 
    //                                              path_.path_segment__original_.back().y, 
    //                                              robot_pose_.x_meter, 
    //                                              robot_pose_.y_meter);
    // ROS_INFO_STREAM("_dist_to_end " << _dist_to_end << "  look_ahead " << parameters_.look_ahead_distance_meter_);
    // if (_dist_to_end <= parameters_.look_ahead_distance_meter_ * 1.0)
    // {
    //     m_finish_ = true;
    // }
    // ROS_INFO_STREAM("find_target_point()2  m_finish_ " << m_finish_ );
}


/// @brief For calling from external only. To get the value of the target_point at the moment.
/// @param target : result.  geometry_msgs::PointStamped
void ClassPurePursuit::get_target_point(geometry_msgs::PointStamped &target)
{
    target = target_points_;
}


/// @brief One of the main functions to be called. After all parameters are updated, and the target_point has
/// been found, then call this func to compute the steer angle that can move the robot to the target_point.
/// @param success : bool. 
/// @param steer_rad : the result to be found.
void ClassPurePursuit::solve_for_speed_command(bool &success, double &steer_rad)
{
    // first, compute the angle difference between the robot_heading and the line from robot 
    // to the target_point. 
    double _angle_robot_to_target_point = compute_yaw_of_2_points(robot_pose_.x_meter, 
                                                                  robot_pose_.y_meter,
                                                                  target_points_.point.x,
                                                                  target_points_.point.y);
    _angle_robot_to_target_point = mod_2pi(_angle_robot_to_target_point);

    // std::cout << "_angle_robot_to_target_point     " << _angle_robot_to_target_point << std::endl;
    // std::cout << "robot_pose_.yaw_rad     " << robot_pose_.yaw_rad << std::endl;

    double _large_angle = std::max<double>(_angle_robot_to_target_point, robot_pose_.yaw_rad);
    double _small_angle = std::min<double>(_angle_robot_to_target_point, robot_pose_.yaw_rad);
    double _angle_difference_rad =  _large_angle - _small_angle;
    if( _angle_difference_rad > M_PI)
    {
        _angle_difference_rad = 2*M_PI - _angle_difference_rad;
    }

    // the value above has only magnitude but no direction. 
    // So now, check the target_point locates on which side of the robot_heading. Due to the 
    // method I use, an extra point in front of robot is required. 
    double _virtual_point_x = robot_pose_.x_meter + 1.0 * cos(robot_pose_.yaw_rad);
    double _virtual_point_y = robot_pose_.y_meter + 1.0 * sin(robot_pose_.yaw_rad);

    bool on_left_side = check_if_point_on_left_of_line(robot_pose_.x_meter, 
                                                        robot_pose_.y_meter, 
                                                        _virtual_point_x, 
                                                        _virtual_point_y, 
                                                        target_points_.point.x, 
                                                        target_points_.point.y);

    if( ! on_left_side)
    {
        _angle_difference_rad *= -1.0;  // using regular meth/physics coordinate. CCW == positive.
    }

    // the main formula to compute the requied steer angle to move to the taret_point.
    steer_rad = std::atan((2 * parameters_.wheelbase_meter_ * std::sin(_angle_difference_rad)) / parameters_.look_ahead_distance_meter_);
    success = true;
    
    return;
}


/// @brief After found the pair of points that has one point in L_d region and one outside L_d region, use this funciton to find a more
/// precise point between these two point, which is closer to the look_ahead_distance circle.
/// @param two_pathpoints : the two points mentioned in brief.
/// @param result_point : the precise point to be found.
void ClassPurePursuit::find_precise_target_point(std::array<geometry_msgs::Point, 2> two_pathpoints, geometry_msgs::PointStamped &result_point)
{
    geometry_msgs::Point p1 = two_pathpoints[0];
    geometry_msgs::Point p2 = two_pathpoints[1];
    geometry_msgs::Point pm;  // the point between p1 and p2. 

    double _critical_distance_meter = 0.01; 

    // The method is like binary search. Get the middle point bwtn p1 p2, and check if this point is close enough to the 
    // look_ahead_distance circle.  If close enough or for loop ends, then use this point as result. Otherwise, keep slicing
    // into halves.
    // Limit the number of iterations to be tried, in the worst case. 
    for(int ct=0; ct<10; ct++)
    {
        pm.x = (p1.x + p2.x) /2.0;
        pm.y = (p1.y + p2.y) /2.0;
        
        double _distance_pm_to_robot = compute_distance_meter(pm.x, pm.y, robot_pose_.x_meter, robot_pose_.y_meter);
        if( std::abs(_distance_pm_to_robot - parameters_.look_ahead_distance_meter_) < _critical_distance_meter )
        {
            // std::cout << "found the estimated target point." << std::endl;
            result_point.point.x = pm.x;
            result_point.point.y = pm.y;
            return;
        }
        else
        {
            if(_distance_pm_to_robot > parameters_.look_ahead_distance_meter_)
            {
                p2 = pm;
            }
            else
            {
                p1 = pm;
            }
        }
    }
    // std::cout << "reached max binary search iterations." << std::endl;
    result_point.point.x = pm.x;
    result_point.point.y = pm.y;
}

// bool ClassPurePursuit::isFinish()
// {
//     return m_finish_;
// }

#endif