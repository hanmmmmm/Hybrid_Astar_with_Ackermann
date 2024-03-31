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
 * @file class_purepursuit.h
 * @author Mingjie
 * @brief This is the implementation of the pure pursuit algorithm.
            The other parts like testing and ros-node are wrappers of this class.
 * @version 0.2
 * @date 2023-11-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#ifndef HAWA_CLASS_PUREPURSUIT_H
#define HAWA_CLASS_PUREPURSUIT_H


#include <iostream>
#include <vector>
#include <array>
#include <math.h>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "../common/class_elemental_path2d_segment.h"
#include "../common/class_elemental_pose2d.h"
#include "../common/tools_angles.h"
#include "tools_purepursuit.h"


/**
 * @brief The my implementation of the pure pursuit algorithm. This class has only the algorithm 
 * things, it does not have ros things. 
*/
class ClassPurePursuit
{
private:
    StructParameters m_parameters_;

    std::vector<std::array<geometry_msgs::msg::Point, 2>> m_vector_target_points_;

    geometry_msgs::msg::PointStamped m_actual_target_points_;

    StructPose m_robot_pose_;

    ClassPath2DSegment m_the_segment_;

private:
    void computeLookAheadDistnace();

public:
    ClassPurePursuit();
    ~ClassPurePursuit();

    bool setVehicleWheelbaseMeter(double val);  

    bool setTargetLinearSpeedMps(double val);

    bool setLookAheadCoefficient(double val);

    void setPathSegment(ClassPath2DSegment& r_segment);

    void setRobotPose(double x_meter, double y_meter, double yaw_rad);

    void findTargetPoint(bool& r_success);

    void getTargetPoint(geometry_msgs::msg::PointStamped& r_target);

    void solveForSpeedCommand(bool& r_success, double& r_steer_rad);

    void findPreciseTargetPoint(std::array<geometry_msgs::msg::Point, 2> two_pathpoints, 
                                geometry_msgs::msg::PointStamped& r_result_point);

};


ClassPurePursuit::ClassPurePursuit()
{
    m_parameters_.k_max_target_speed_mps_ = 3.0;
    m_parameters_.k_max_look_ahead_coefficient_ = 30.0;
    m_parameters_.k_min_wheelbase_meter_ = 0.05;
    m_parameters_.k_max_wheelbase_meter_ = 0.5;
}

ClassPurePursuit::~ClassPurePursuit()
{
}

/**
 * @brief External use. To set the path_segment.
 * @param path reference of a ClassPath2DSegment
*/
void ClassPurePursuit::setPathSegment(ClassPath2DSegment& r_segment)
{
    m_the_segment_ = r_segment;
}

/**
 * @brief External use. To set the value of the robot_pose.
 * @param x_meter 
 * @param y_meter 
 * @param yaw_rad 
*/
void ClassPurePursuit::setRobotPose(double x_meter, double y_meter, double yaw_rad)
{
    m_robot_pose_.x_meter = x_meter;
    m_robot_pose_.y_meter = y_meter;
    m_robot_pose_.yaw_rad = mod2pi(yaw_rad);
}

/**
 * @brief External use. To set the value of the target_linear_speed_mps.
 * @param val double in meter per second.
 * @return bool. False when the input value exceeds the limits, and the value will not be updated.
*/
bool ClassPurePursuit::setTargetLinearSpeedMps(double val)
{
    if( std::abs(val) < m_parameters_.k_max_target_speed_mps_ )
    {
        m_parameters_.target_longitude_speed_mps_ = val;
        return true;
    }
    return false;
}

/**
 * @brief External use. To set the value of the look_ahead_coefficient.
 * @param val : positive double value. unitless.
 * @return bool. False when the input value exceeds the limits, and the value will not be updated.
*/
bool ClassPurePursuit::setLookAheadCoefficient(double val)
{
    if( (0 < val) && (val < m_parameters_.k_max_look_ahead_coefficient_) )
    {
        m_parameters_.look_ahead_coefficient_ = val;
        return true;
    }
    return false;
}

/**
 * @brief External use. To set the value of the wheel base. It's the distance between the axles of the 
 * front wheel and rear wheel.
 * @param val : double in meter.
 * @return bool. False when the input value exceeds the limits, and the value will not be updated.
*/
bool ClassPurePursuit::setVehicleWheelbaseMeter(double val)
{
    if( (val > m_parameters_.k_min_wheelbase_meter_) && (val < m_parameters_.k_max_wheelbase_meter_) )
    {
        m_parameters_.wheelbase_meter_ = val;
        return true;
    }
    return false;
}

/**
 * @brief After the linear speed and look_ahead_coefficient are set, call this func to compute the L_d.
*/
void ClassPurePursuit::computeLookAheadDistnace()
{
    m_parameters_.look_ahead_distance_meter_ = std::abs(
                    m_parameters_.look_ahead_coefficient_ * m_parameters_.target_longitude_speed_mps_);
}

/**
 * @brief One of the main functions to be called. After all parameters are updated, call this func to 
 * find the target_point from the path. This function is possible to fail finding the target_point when
 * robot is significantly far away from the path.
 * @param r_success bool
*/
void ClassPurePursuit::findTargetPoint(bool& r_success)
{
    int _num_points = m_the_segment_.m_path_extended_.size();
    if(_num_points <= 2)
    {
        // ROS_WARN_STREAM_THROTTLE(10 , "findTargetPoint()  Path too short: " << _num_points);
        r_success = false;
        return;
    }

    computeLookAheadDistnace();
    m_vector_target_points_.clear();

    for(int i=0; i<_num_points-1; i++)
    {
        ClassPose2D _pose_1 = m_the_segment_.m_path_extended_.at(i);
        ClassPose2D _pose_2 = m_the_segment_.m_path_extended_.at(i+1);
        double _dist_1 = computeDistanceMeter(_pose_1.x, _pose_1.y, m_robot_pose_.x_meter, m_robot_pose_.y_meter);
        double _dist_2 = computeDistanceMeter(_pose_2.x, _pose_2.y, m_robot_pose_.x_meter, m_robot_pose_.y_meter);

        if( (_dist_1 < m_parameters_.look_ahead_distance_meter_) 
            && (_dist_2 > m_parameters_.look_ahead_distance_meter_) )
        {
            geometry_msgs::msg::Point _p1;
            _p1.x = _pose_1.x;
            _p1.y = _pose_1.y;
            
            geometry_msgs::msg::Point _p2;
            _p2.x = _pose_2.x;
            _p2.y = _pose_2.y;

            m_vector_target_points_.push_back(std::array<geometry_msgs::msg::Point, 2> {_p1, _p2});
        }

    }

    if(m_vector_target_points_.size() == 0)
    {
        // ROS_DEBUG_STREAM_THROTTLE(10, "No Target point found.");
        /*
        TODO:
        expand searching area and try again.
        */
    }
    else if(m_vector_target_points_.size() >= 2)
    {
        // ROS_DEBUG_STREAM_THROTTLE(10, "Found >= 2 Target points.");
        /*
        TODO:
        select the best one.
        */
       findPreciseTargetPoint(m_vector_target_points_[0], m_actual_target_points_);
        return;
    }
    else if(m_vector_target_points_.size() == 1)
    {
        // ROS_DEBUG_STREAM_THROTTLE(10, "Found 1 Target point.");
        r_success = true;
        findPreciseTargetPoint(m_vector_target_points_[0], m_actual_target_points_);
        return;
    }
    r_success = false;
}

/**
 * @brief For calling from external only. To get the value of the target_point at the moment.
 * @param r_target : result.  geometry_msgs::PointStamped
*/
void ClassPurePursuit::getTargetPoint(geometry_msgs::msg::PointStamped& r_target)
{
    r_target = m_actual_target_points_;
}

/**
 * @brief One of the main functions to be called. After all parameters are updated, and the target_point has
 * been found, then call this func to compute the steer angle that can move the robot to the target_point.
 * @param r_success : bool. 
 * @param r_steer_rad : the result to be found.
*/
void ClassPurePursuit::solveForSpeedCommand(bool& r_success, double& r_steer_rad)
{
    // first, compute the angle difference between the robot_heading and the line from robot 
    // to the target_point. 
    double _angle_robot_to_target_point = computeYawOf2Points(m_robot_pose_.x_meter, 
                                                                  m_robot_pose_.y_meter,
                                                                  m_actual_target_points_.point.x,
                                                                  m_actual_target_points_.point.y);
    _angle_robot_to_target_point = mod2pi(_angle_robot_to_target_point);

    double _large_angle = std::max<double>(_angle_robot_to_target_point, m_robot_pose_.yaw_rad);
    double _small_angle = std::min<double>(_angle_robot_to_target_point, m_robot_pose_.yaw_rad);
    double _angle_difference_rad =  _large_angle - _small_angle;
    if( _angle_difference_rad > M_PI)
    {
        _angle_difference_rad = 2*M_PI - _angle_difference_rad;
    }

    // the value above has only magnitude but no direction. 
    // So now, check the target_point locates on which side of the robot_heading. Due to the 
    // method I use, an extra virtual point in front of robot is required. 
    double _virtual_point_x = m_robot_pose_.x_meter + 1.0 * cos(m_robot_pose_.yaw_rad);
    double _virtual_point_y = m_robot_pose_.y_meter + 1.0 * sin(m_robot_pose_.yaw_rad);

    bool on_left_side = checkIfPointOnLeftOfLine(m_robot_pose_.x_meter, 
                                                 m_robot_pose_.y_meter, 
                                                 _virtual_point_x, 
                                                 _virtual_point_y, 
                                                 m_actual_target_points_.point.x, 
                                                 m_actual_target_points_.point.y);

    if( ! on_left_side)
    {
        _angle_difference_rad *= -1.0;  // using regular meth/physics coordinate. CCW == positive.
    }

    // the main formula to compute the requied steer angle to move to the taret_point.
    r_steer_rad = std::atan((2 * m_parameters_.wheelbase_meter_ * std::sin(_angle_difference_rad)) 
                                    / m_parameters_.look_ahead_distance_meter_);
    r_success = true;
    
    return;
}

/**
 * @brief After found the pair of points that has one point in L_d region and one outside L_d region, 
 * use this funciton to find a more precise point between these two point, which is closer to the 
 * look_ahead_distance circle.
 * @param two_pathpoints : the two points mentioned in brief.
 * @param r_result_point : the precise point to be found.
*/
void ClassPurePursuit::findPreciseTargetPoint(std::array<geometry_msgs::msg::Point, 2> two_pathpoints, 
                                              geometry_msgs::msg::PointStamped& r_result_point)
{
    geometry_msgs::msg::Point p1 = two_pathpoints[0];
    geometry_msgs::msg::Point p2 = two_pathpoints[1];
    geometry_msgs::msg::Point pm;  // the point between p1 and p2. 

    double _critical_distance_meter = 0.01; 

    // The method is like binary search. Get the middle point bwtn p1 p2, and check if this point is close enough 
    // to the look_ahead_distance circle.  If close enough or for loop ends, then use this point as result. 
    // Otherwise, keep slicing into halves.
    // Limit the number of iterations to be tried, in the worst case. 
    for(int ct=0; ct<10; ct++)
    {
        pm.x = (p1.x + p2.x) /2.0;
        pm.y = (p1.y + p2.y) /2.0;
        
        double _distance_pm_to_robot = computeDistanceMeter(pm.x, pm.y, m_robot_pose_.x_meter, m_robot_pose_.y_meter);
        if( std::abs(_distance_pm_to_robot - m_parameters_.look_ahead_distance_meter_) < _critical_distance_meter )
        {
            r_result_point.point.x = pm.x;
            r_result_point.point.y = pm.y;
            return;
        }
        else
        {
            if(_distance_pm_to_robot > m_parameters_.look_ahead_distance_meter_)
            {
                p2 = pm;
            }
            else
            {
                p1 = pm;
            }
        }
    }
    // ROS_DEBUG_STREAM_THROTTLE(4, "Reached max TargetPoint search iterations.");
    r_result_point.point.x = pm.x;
    r_result_point.point.y = pm.y;
}

#endif