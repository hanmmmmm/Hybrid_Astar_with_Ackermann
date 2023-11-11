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
 * This file is 
 */


#ifndef CLASS_HAWA_REEDSSHEPP_TOOL_H
#define CLASS_HAWA_REEDSSHEPP_TOOL_H

#include <math.h>

#include "../utils/hawa_conversion_tools.h"
#include "hawa_reeds_shepp_curves/reedshepp_path.h"


struct StructReedsSheppFormulaResult
{
    double t = 0.0;
    double u = 0.0;
    double v = 0.0;
    double L = 0.0;
    bool valid = false;
};

/**
 * 
*/
struct StructRSPathResult
{
    std::string path_word;
    std::vector< std::array<double, 3> > path_steps;
    double path_length_unitless;
    bool valid = false; 
    bool active = true;

    void setName(std::string name)
    {
        path_word = name;
    }

    void setValid(const bool val)
    {
        valid = val;
    }

    void reset()
    {
        path_length_unitless = 0;
        path_steps.clear();
        valid = false;
    }

    void setActive()
    {
        active = true;
    }
    
    void setInactive()
    {
        active = false;
    }

    std::string getName()
    {
        return path_word;
    }
};


struct StructSamplingProperties
{
    double linear_step_size;
    double angular_step_size;
    double turning_radius;
};


struct StructDubinsCurveCCCvalcollection
{
    double u;
    double t;
    double v;
    bool valid;
};



/**
 * 
*/
void convertCartesianToPolar(const double x, const double y, double &radius_ro, double &theta)
{
    radius_ro = std::sqrt(x*x + y*y);
    theta = std::atan2(y, x);
}



void tauOmega(const double u, const double v, const double xi, const double eta, const double phi,
                               double &tau, double &omega)
{
    double _delta = mod2pi(u - v);
    double _A = std::sin(u) - std::sin(_delta);
    double _B = std::cos(u) - std::cos(_delta) - 1.0;
    double _t1 = std::atan2(eta * _A - xi * _B, xi * _A + eta * _B);
    double _t2 = 2.0 * (std::cos(_delta) - std::cos(v) - std::cos(u)) + 3;
    if (_t2 < 0)
    {
        tau = mod2pi(_t1 + M_PI);
    }
    else
    {
        tau = mod2pi(_t1);
    }
    omega = mod2pi(tau - u + v - phi);
}






/**
 * 
*/
void sampleOnLeftCurve( const double target_angle_change, 
                        double& robot_yaw, double& robotx, double& roboty, 
                        StructRSPathResult* ptr_path_result,
                        StructSamplingProperties* r_properties)
{
    double init_yaw = robot_yaw;
    double init_x = robotx;
    double init_y = roboty;
    double theta_change = 0;
    // double speed = 0.0;
    double ang_step_size_local = 0.0;
    if( target_angle_change >= 0.0 ){
        // speed = r_properties->linear_step_size;
        ang_step_size_local = r_properties->angular_step_size;
    }
    else {
        // speed = -1.0 * r_properties->linear_step_size;
        ang_step_size_local = -1.0 * r_properties->angular_step_size;
    }

    while ( std::abs( theta_change )  < std::abs( target_angle_change ) ){
        if ( std::abs( target_angle_change ) - std::abs( theta_change ) < std::abs( ang_step_size_local ) )
        {
            ang_step_size_local = std::abs( target_angle_change ) - std::abs( theta_change );
            if( target_angle_change < 0.0 )
            {
                ang_step_size_local *= -1.0;
            }
        }

        double _body_frame_dx = r_properties->turning_radius * std::sin(ang_step_size_local);
        double _body_frame_dy = r_properties->turning_radius 
                                    - r_properties->turning_radius * std::cos(ang_step_size_local);

        double _fix_frame_dx = _body_frame_dx * cos(robot_yaw) - _body_frame_dy * cos(M_PI/2.0 - robot_yaw);
        double _fix_frame_dy = _body_frame_dx * cos(M_PI/2.0 - robot_yaw) + _body_frame_dy * cos(robot_yaw);

        robotx += _fix_frame_dx;
        roboty += _fix_frame_dy;
        theta_change += ang_step_size_local;
        robot_yaw += ang_step_size_local ;
        
        // std::array<double,3> pose = {robotx, roboty, robot_yaw};
        // ptr_path_result->path_steps.push_back(pose);
        ptr_path_result->path_steps.push_back(std::array<double,3>{robotx, roboty, robot_yaw});
    }

    double _body_frame_dx = r_properties->turning_radius * sin(target_angle_change);
    double _body_frame_dy = r_properties->turning_radius - r_properties->turning_radius * cos(target_angle_change);
    double _fix_frame_dx = _body_frame_dx * cos(init_yaw) - _body_frame_dy * cos(M_PI/2.0 - init_yaw);
    double _fix_frame_dy = _body_frame_dx * cos(M_PI/2.0 - init_yaw) + _body_frame_dy * cos(init_yaw);

    robotx = init_x + _fix_frame_dx;
    roboty = init_y + _fix_frame_dy;
    robot_yaw = mod2pi(init_yaw + target_angle_change);
}




void sampleOnRightCurve(const double target_angle_change, 
                        double& robot_yaw, double& robotx, double& roboty, 
                        StructRSPathResult* ptr_path_result,
                        StructSamplingProperties* r_properties)
{
    double init_yaw = robot_yaw;
    double init_x = robotx;
    double init_y = roboty;
    double theta_change = 0;
    // double speed = 0.0;
    double ang_step_size_local = 0.0;
    if( target_angle_change >= 0.0 )
    {
        // speed = v_l;
        ang_step_size_local = -1.0 * r_properties->angular_step_size;
    }
    else 
    {
        // speed = -1.0 * v_l;
        ang_step_size_local = r_properties->angular_step_size;
    }
    while ( std::abs( theta_change )  < std::abs( target_angle_change ) )
    {
        if ( std::abs( target_angle_change ) - std::abs( theta_change ) < std::abs( ang_step_size_local ) )
        {
            ang_step_size_local = std::abs( target_angle_change ) - std::abs( theta_change );
            if( target_angle_change > 0.0 )
            {
                ang_step_size_local *= -1.0;
            }
        }

        double _body_frame_dx = r_properties->turning_radius * std::sin(-ang_step_size_local);
        double _body_frame_dy = -r_properties->turning_radius 
                                    + r_properties->turning_radius * std::cos(-ang_step_size_local);
        
        double _fix_frame_dx = _body_frame_dx * std::cos(robot_yaw) - _body_frame_dy * std::cos(M_PI/2.0 - robot_yaw);
        double _fix_frame_dy = _body_frame_dx * std::cos(M_PI/2.0 - robot_yaw) + _body_frame_dy * std::cos(robot_yaw);

        robotx += _fix_frame_dx;
        roboty += _fix_frame_dy;

        theta_change += ang_step_size_local;
        robot_yaw += ang_step_size_local ;

        // std::array<double,3> pose = {robotx, roboty, robot_yaw};
        // result_container.path_steps.push_back(pose);
        ptr_path_result->path_steps.push_back(std::array<double,3>{robotx, roboty, robot_yaw});
    }
    
    double _body_frame_dx = r_properties->turning_radius * std::sin(target_angle_change);
    double _body_frame_dy = -r_properties->turning_radius + r_properties->turning_radius * cos(target_angle_change);
    double _fix_frame_dx = _body_frame_dx * std::cos(init_yaw) - _body_frame_dy * std::cos(M_PI/2.0 - init_yaw);
    double _fix_frame_dy = _body_frame_dx * std::cos(M_PI/2.0 - init_yaw) + _body_frame_dy * std::cos(init_yaw);

    robotx = init_x + _fix_frame_dx;
    roboty = init_y + _fix_frame_dy;
    robot_yaw = mod2pi(init_yaw - target_angle_change);

}



void sampleOnStraightLine(const double target_angle_change, 
                          double robot_yaw, double& robotx, double& roboty, 
                          StructRSPathResult* ptr_path_result,
                          StructSamplingProperties* r_properties)
{
    double init_x = robotx;
    double init_y = roboty;
    double theta_change = 0;
    double linear_step_size_local = 0.0;
    if( target_angle_change >= 0.0 ){
        linear_step_size_local = r_properties->linear_step_size;
    }
    else {
        linear_step_size_local = -1.0 * r_properties->linear_step_size;
    }

    double _target_distance = target_angle_change * r_properties->turning_radius;
    bool _finish = false;

    while ( std::abs( theta_change )  < std::abs( _target_distance ) )
    {
        if ( std::abs( _target_distance ) - std::abs( theta_change ) < std::abs( linear_step_size_local ) )
        {
            linear_step_size_local = std::abs( _target_distance ) - std::abs( theta_change );
            if( target_angle_change < 0.0 )
            {
                linear_step_size_local *= -1.0;
            }
            _finish = true;
        }

        double dx = linear_step_size_local * cos( robot_yaw );
        double dy = linear_step_size_local * sin( robot_yaw );
        robotx = robotx + dx;
        roboty = roboty + dy;
        theta_change += linear_step_size_local;
        
        // std::array<double,3> pose = {robotx, roboty, robot_yaw};
        // result_container.path_steps.push_back(pose);
        ptr_path_result->path_steps.push_back(std::array<double,3>{robotx, roboty, robot_yaw});

        if (_finish)
            break;
        
    }
    robotx = init_x + target_angle_change * r_properties->turning_radius * std::cos( robot_yaw );
    roboty = init_y + target_angle_change * r_properties->turning_radius * std::sin( robot_yaw );
}



// // void ReedsSheppClass::add_sort_path( ClassReedSheppPath::pathResult& path )
// void add_sort_path(const StructRSPathResult& r_path, std::vector<StructRSPathResult>& r_path_collection )
// {
//     if( r_path_collection.size() <= 0){
//         r_path_collection.push_back(r_path);
//     }
//     else{
//         double length = r_path.path_length_unitless;
//         int count = 0;
//         for(auto p : r_path_collection){
//             if( p.path_length_unitless > length ){
//                 break;
//             }
//             else{
//                 count ++;
//             }
//         }
//         r_path_collection.insert(  r_path_collection.begin()+count, r_path);
//     }
// }



void add_sort_path(StructRSPathResult* ptr_path, std::vector<StructRSPathResult>& r_path_collection )
{
    if (! ptr_path->valid)
    {
        return;
    }

    if( r_path_collection.size() <= 0){
        r_path_collection.push_back(*ptr_path);
        return;
    }
    
    int _count = 0;
    for(auto p : r_path_collection){
        if( p.path_length_unitless > ptr_path->path_length_unitless ){
            break;
        }
        else{
            _count ++;
        }
    }
    r_path_collection.insert( r_path_collection.begin() + _count, *ptr_path);
}




#endif