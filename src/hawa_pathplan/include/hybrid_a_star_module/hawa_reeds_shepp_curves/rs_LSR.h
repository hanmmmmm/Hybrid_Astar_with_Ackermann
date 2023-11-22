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
// SOFTWARE
/**
 * @file rs_LSR.h
 * @author Mingjie
 * @brief Solve the LSR type curves. 
 * @version 0.1
 * @date 2023-11-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef HAWA_REEDS_SHEPP_LSR_H
#define HAWA_REEDS_SHEPP_LSR_H

#include <math.h>

#include "../../utils/hawa_conversion_tools.h"
#include "../../utils/hawa_data_containers.h"
#include "../reedsshepp_tools.h"


StructReedsSheppFormulaResult LpSpRp_base(StructPoseReal* ptr_pose)
{
    StructReedsSheppFormulaResult _rs_result;

    double t1, u1;

    convertCartesianToPolar(ptr_pose->x + std::sin(ptr_pose->yaw), 
                            ptr_pose->y - std::cos(ptr_pose->yaw) - 1, 
                            u1,
                            t1);
    if(u1 * u1 < 4.0){
        _rs_result.valid = false;
        return _rs_result;
    }
    _rs_result.u = std::sqrt(u1 * u1 - 4.0);
    double _theta = std::atan2(2.0, _rs_result.u);
    _rs_result.t = mod2pi(t1 + _theta);
    _rs_result.v = mod2pi(_rs_result.t - ptr_pose->yaw);
    _rs_result.L = std::abs( _rs_result.t) + std::abs( _rs_result.u) + std::abs( _rs_result.v) ;

    if( _rs_result.t > 0.0 && _rs_result.v > 0.0){
        _rs_result.valid = true;
    }
    else{
        _rs_result.valid = false;
    }

    if (std::abs(_rs_result.t) > 1.5 * M_PI || std::abs(_rs_result.v) > 1.5 * M_PI) _rs_result.valid = false;
    
    
    return _rs_result;
}


void solveLpSpRp(StructPoseReal goal_pose, 
                 StructPoseReal start_pose, 
                 StructRSPathResult* ptr_path, 
                 StructSamplingProperties* ptr_sample )
{
    StructReedsSheppFormulaResult _res = LpSpRp_base(&goal_pose);

    ptr_path->setValid(_res.valid);

    if (! _res.valid)
        return;
    
    ptr_path->path_length_unitless = _res.L;

    sampleOnLeftCurve(_res.t, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnStraightLine(_res.u, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnRightCurve(_res.v, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
}

void solveLmSmRm(StructPoseReal goal_pose, 
                 StructPoseReal start_pose, 
                 StructRSPathResult* ptr_path, 
                 StructSamplingProperties* ptr_sample )
{
    goal_pose.x *= -1;
    goal_pose.yaw *= -1;

    StructReedsSheppFormulaResult _res = LpSpRp_base(&goal_pose);

    ptr_path->setValid(_res.valid);

    if (! _res.valid)
        return;
    
    ptr_path->path_length_unitless = _res.L;

    sampleOnLeftCurve(-_res.t, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnStraightLine(-_res.u, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnRightCurve(-_res.v, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);

}

// void LmSmLm(){
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpSpLp_base(-x,y,-phi,t,u,v,L,valid);
//     all_possible_paths_.LmSmLm.path_word = "LmSmLm";
//     all_possible_paths_.LmSmLm.valid = valid;
//     if( valid ){
//         all_possible_paths_.LmSmLm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( -t, robot_theta, robot_x, robot_y, all_possible_paths_.LmSmLm);
//         get_samples_S( -u, robot_theta, robot_x, robot_y, all_possible_paths_.LmSmLm);
//         get_samples_L( -v, robot_theta, robot_x, robot_y, all_possible_paths_.LmSmLm);
//         all_possible_paths_.LmSmLm.path_length_unitless = calc_weighted_length(std::vector<double>{-t, -u, -v});
//         add_sort_path( all_possible_paths_.LmSmLm );
//     }
// }


void solveRpSpLp(StructPoseReal goal_pose, 
                 StructPoseReal start_pose, 
                 StructRSPathResult* ptr_path, 
                 StructSamplingProperties* ptr_sample )
{
    goal_pose.y *= -1;
    goal_pose.yaw *= -1;

    StructReedsSheppFormulaResult _res = LpSpRp_base(&goal_pose);

    ptr_path->setValid(_res.valid);

    if (! _res.valid)
        return;
    
    ptr_path->path_length_unitless = _res.L;

    sampleOnRightCurve(_res.t, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnStraightLine(_res.u, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnLeftCurve(_res.v, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
}

// void RpSpRp( ){
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpSpLp_base(x,-y,-phi,t,u,v,L,valid);
//     all_possible_paths_.RpSpRp.path_word = "RpSpRp";
//     all_possible_paths_.RpSpRp.valid = valid;
//     if( valid ){
//         all_possible_paths_.RpSpRp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( t, robot_theta, robot_x, robot_y, all_possible_paths_.RpSpRp);
//         get_samples_S( u, robot_theta, robot_x, robot_y, all_possible_paths_.RpSpRp);
//         get_samples_R( v, robot_theta, robot_x, robot_y, all_possible_paths_.RpSpRp);
//         all_possible_paths_.RpSpRp.path_length_unitless = calc_weighted_length(std::vector<double>{t, u, v});
//         add_sort_path( all_possible_paths_.RpSpRp );
//     }
// }



void solveRmSmLm(StructPoseReal goal_pose, 
                 StructPoseReal start_pose, 
                 StructRSPathResult* ptr_path, 
                 StructSamplingProperties* ptr_sample )
{
    goal_pose.x *= -1;
    goal_pose.y *= -1;

    StructReedsSheppFormulaResult _res = LpSpRp_base(&goal_pose);

    ptr_path->setValid(_res.valid);

    if (! _res.valid)
        return;
    
    ptr_path->path_length_unitless = _res.L;

    sampleOnRightCurve(-_res.t, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnStraightLine(-_res.u, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnLeftCurve(-_res.v, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
}

// void RmSmRm( ){
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpSpLp_base(-x,-y, phi,t,u,v,L,valid);
//     all_possible_paths_.RmSmRm.path_word = "RmSmRm";
//     all_possible_paths_.RmSmRm.valid = valid;
//     if( valid ){
//         all_possible_paths_.RmSmRm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( -t, robot_theta, robot_x, robot_y, all_possible_paths_.RmSmRm);
//         get_samples_S( -u, robot_theta, robot_x, robot_y, all_possible_paths_.RmSmRm);
//         get_samples_R( -v, robot_theta, robot_x, robot_y, all_possible_paths_.RmSmRm);
//         all_possible_paths_.RmSmRm.path_length_unitless = calc_weighted_length(std::vector<double>{-t, -u, -v});
//         add_sort_path( all_possible_paths_.RmSmRm );
//     }
// }



#endif