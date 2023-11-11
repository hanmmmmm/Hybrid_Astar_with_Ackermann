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


#ifndef HAWA_REEDS_SHEPP_LSL_H
#define HAWA_REEDS_SHEPP_LSL_H

#include <math.h>

#include "../../utils/hawa_conversion_tools.h"
#include "../../utils/hawa_data_containers.h"
#include "../reedsshepp_tools.h"
// #include "reedshepp_path.h"

///////////////////////////   CSC 
// 8.1 in paper 


// void LpSpLp_base(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid )
// {
//     valid = false;

//     convertCartesianToPolar( x-sin(phi), y-1+cos(phi), u, t );
//     v = mod2pi(phi - t);
//     L = std::abs(t) + std::abs(u) + std::abs(v) ;
//     if( t > 0.0 && v > 0.0){
//         valid = true;
//     }
//     else{
//         valid = false;
//     }
// }

// void LpSpLp_base(StructPoseReal* ptr_pose, StructReedsSheppFormulaResult* rs_result )
// {
//     rs_result->valid = false;

//     convertCartesianToPolar(ptr_pose->x - std::sin(ptr_pose->yaw), 
//                             ptr_pose->y + std::cos(ptr_pose->yaw) - 1, 
//                             rs_result->u,
//                             rs_result->t);
//     rs_result->v = mod2pi(ptr_pose->yaw - rs_result->t);
//     rs_result->L = std::abs(rs_result->t) + std::abs(rs_result->u) + std::abs(rs_result->v) ;
//     if( rs_result->t > 0.0 && rs_result->v > 0.0){
//         rs_result->valid = true;
//     }
//     else{
//         rs_result->valid = false;
//     }
// }

StructReedsSheppFormulaResult LpSpLp_base(StructPoseReal* ptr_pose)
{
    StructReedsSheppFormulaResult _rs_result;

    convertCartesianToPolar(ptr_pose->x - std::sin(ptr_pose->yaw), 
                            ptr_pose->y + std::cos(ptr_pose->yaw) - 1, 
                            _rs_result.u,
                            _rs_result.t);
    _rs_result.v = mod2pi(ptr_pose->yaw - _rs_result.t);
    _rs_result.L = std::abs(_rs_result.t) + std::abs(_rs_result.u) + std::abs(_rs_result.v) ;
    if( _rs_result.t > 0.0 && _rs_result.v > 0.0){
        _rs_result.valid = true;
    }
    else{
        _rs_result.valid = false;
    }
    return _rs_result;
}


void solveLpSpLp(StructPoseReal goal_pose, 
                 StructPoseReal start_pose, 
                 StructRSPathResult* ptr_path, 
                 StructSamplingProperties* ptr_sample )
{
    StructReedsSheppFormulaResult _res = LpSpLp_base(&goal_pose);

    ptr_path->setValid(_res.valid);

    if (! _res.valid)
        return;
    
    ptr_path->path_length_unitless = _res.L;

    sampleOnLeftCurve(_res.t, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnStraightLine(_res.u, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnLeftCurve(_res.v, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
}

void solveLmSmLm(StructPoseReal goal_pose, 
                 StructPoseReal start_pose, 
                 StructRSPathResult* ptr_path, 
                 StructSamplingProperties* ptr_sample )
{
    goal_pose.x *= -1;
    goal_pose.yaw *= -1;

    StructReedsSheppFormulaResult _res = LpSpLp_base(&goal_pose);

    ptr_path->setValid(_res.valid);

    if (! _res.valid)
        return;
    
    ptr_path->path_length_unitless = _res.L;

    sampleOnLeftCurve(-_res.t, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnStraightLine(-_res.u, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnLeftCurve(-_res.v, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);

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


void solveRpSpRp(StructPoseReal goal_pose, 
                 StructPoseReal start_pose, 
                 StructRSPathResult* ptr_path, 
                 StructSamplingProperties* ptr_sample )
{
    goal_pose.y *= -1;
    goal_pose.yaw *= -1;

    StructReedsSheppFormulaResult _res = LpSpLp_base(&goal_pose);

    ptr_path->setValid(_res.valid);

    if (! _res.valid)
        return;
    
    ptr_path->path_length_unitless = _res.L;

    sampleOnLeftCurve(_res.t, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
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



void solveRmSmRm(StructPoseReal goal_pose, 
                 StructPoseReal start_pose, 
                 StructRSPathResult* ptr_path, 
                 StructSamplingProperties* ptr_sample )
{
    goal_pose.x *= -1;
    goal_pose.y *= -1;

    StructReedsSheppFormulaResult _res = LpSpLp_base(&goal_pose);

    ptr_path->setValid(_res.valid);

    if (! _res.valid)
        return;
    
    ptr_path->path_length_unitless = _res.L;

    sampleOnLeftCurve(-_res.t, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
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