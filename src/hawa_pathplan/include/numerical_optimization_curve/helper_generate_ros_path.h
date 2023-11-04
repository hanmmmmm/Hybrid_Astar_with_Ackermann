#ifndef GENERATE_ROS_PATH_FROM_SPEED_STEER
#define GENERATE_ROS_PATH_FROM_SPEED_STEER

// Using 

#include <iostream>

#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

#include "vector"

#include "../planner/conversion_tools.h"


double calc_turning_radius(const double axle, const double steer)
{
    // return std::abs( axle / std::tan(steer) );
    return axle / std::tan(steer) ;
}


void generate_rospath(const double init_x, 
                        const double init_y,
                        const double init_yaw,
                        const double axle_dist,
                        const double timestep_sec,
                        const bool verbose,
                        const std::vector<double> speeds,
                        const std::vector<double> steers,
                        nav_msgs::Path& r_path)
{
    if (speeds.size() != steers.size())
    {
        std::cerr << "Check the size of inputs !!! " << "speed:" << speeds.size() << "  steer:" << steers.size() << std::endl;
    }

    r_path.poses.clear();
    geometry_msgs::PoseStamped _temp_pose;
    double _temp_x, _temp_y, _temp_yaw;

    _temp_x = init_x;
    _temp_y = init_y;
    _temp_yaw = init_yaw;
    _temp_pose.pose.position.x = _temp_x;
    _temp_pose.pose.position.y = _temp_y;
    _temp_pose.pose.orientation = tf2qua_to_geoQua(twoD_yaw_to_tf2qua(_temp_yaw));
    r_path.poses.push_back(_temp_pose);

    for (int i=0; i<steers.size(); i++)
    {
        double _steer = steers.at(i);
        // if (std::abs(_steer) < 0.001)

        double _radius = calc_turning_radius(axle_dist, _steer);
        double _arc_length = speeds.at(i) * timestep_sec;
        double _angle_change = _arc_length / _radius;
        
        double _dx_body = _radius * std::sin(_angle_change);
        double _dy_body = _radius - _radius * std::cos(_angle_change);
        double _dyaw = _angle_change;

        if (std::abs(_steer) < 0.1)
        {
            _dx_body = _arc_length;
            _dy_body = 0;
        }
        
        if (speeds.at(i) >= 0)
            {_dx_body = std::abs(_dx_body);}
        else
            {_dx_body = - std::abs(_dx_body);}
        
        if (_steer >= 0)
            {_dy_body = std::abs(_dy_body);}
        else
            {_dy_body = - std::abs(_dy_body);}

    
        double _dx_world = std::cos(_temp_yaw) * _dx_body - std::sin(_temp_yaw) * _dy_body;
        double _dy_world = std::sin(_temp_yaw) * _dx_body + std::cos(_temp_yaw) * _dy_body;

        if (verbose)
        {
            std::cout << "speed: " << speeds.at(i) << "  steer " << _steer << "  radius " << _radius << std::endl;
            std::cout << "body: " << _dx_body << " " << _dy_body << " " << _dyaw << std::endl;
            std::cout << "world: " << _dx_world << " " << _dy_world << std::endl;
            std::cout  << std::endl;
        }


        _temp_x += _dx_world;
        _temp_y += _dy_world;
        _temp_yaw += _dyaw;

        _temp_pose.pose.position.x = _temp_x;
        _temp_pose.pose.position.y = _temp_y;
        _temp_pose.pose.orientation = tf2qua_to_geoQua(twoD_yaw_to_tf2qua(_temp_yaw));
        r_path.poses.push_back(_temp_pose);
    
    }


}

















#endif