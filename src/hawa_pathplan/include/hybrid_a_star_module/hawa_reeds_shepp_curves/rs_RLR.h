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
 * @file rs_RLR.h
 * @author Mingjie
 * @brief Solve the RLR type curves. 
 * @version 0.1
 * @date 2023-11-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef HAWA_REEDS_SHEPP_RLR_H
#define HAWA_REEDS_SHEPP_RLR_H

#include <math.h>

#include "../../utils/hawa_conversion_tools.h"
#include "../../utils/hawa_data_containers.h"
#include "../reedsshepp_tools.h"


/**
 * @brief Call this before run any solveXXX function below, so the important values will be 
 * calculated first.
*/
StructDubinsCurveCCCvalcollection prepareCCCRLR(StructPoseReal goal_pose, 
                                                StructPoseReal start_pose, 
                                                double turning_radius)
{
    double _dyy = goal_pose.y - start_pose.y;
    double _dxx = goal_pose.x - start_pose.x;
    double _theta = std::atan2(_dyy, _dxx);
    double _D = std::sqrt( _dxx*_dxx + _dyy*_dyy );
    double _ccc_d_ = _D/turning_radius;
    double _ccc_alpha_ = mod2pi( start_pose.yaw - _theta );
    double _ccc_beta_  = mod2pi( goal_pose.yaw - _theta );

    double u = mod2pi(2*M_PI - std::acos(( 6-_ccc_d_*_ccc_d_ + 2*std::cos(_ccc_alpha_-_ccc_beta_) 
                                        + 2*_ccc_d_*(std::sin(_ccc_alpha_) - std::sin(_ccc_beta_)) )/8.0 ) ) ;
    double t = mod2pi( _ccc_alpha_ - std::atan( ( std::cos(_ccc_alpha_) - std::cos(_ccc_beta_)) 
                                        /(_ccc_d_ - std::sin(_ccc_alpha_) + std::sin(_ccc_beta_))) + mod2pi(u/2.0) ) ;
    double v = mod2pi( -_ccc_beta_ + _ccc_alpha_ -t +  mod2pi(u));

    StructDubinsCurveCCCvalcollection _result;
    _result.t = t;
    _result.u = u;
    _result.v = v;

    _result.valid = (t>=0.0 && u>=0.0 && v>=0.0);

    return _result;
}

/**
 * 
*/
void solveRpLpRp(StructPoseReal goal_pose, 
                 StructPoseReal start_pose, 
                 StructDubinsCurveCCCvalcollection* ptr_dubins_para, 
                 StructRSPathResult* ptr_path, 
                 StructSamplingProperties* ptr_sample )
{
    ptr_path->setValid(ptr_dubins_para->valid);
    if (! ptr_dubins_para->valid) return;
    double new_t = ptr_dubins_para->t;
    double new_u = ptr_dubins_para->u;
    double new_v = ptr_dubins_para->v;
    ptr_path->path_length_unitless = std::abs(new_t) + std::abs(new_u) + std::abs(new_v) ;

    double _limit = 1.5 * M_PI;
    if (std::abs(new_t) > _limit || std::abs(new_u) > _limit || std::abs(new_v) > _limit)
    {
        ptr_dubins_para->valid = false;
        return;
    }
    
    sampleOnRightCurve(new_t, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnLeftCurve(new_u, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnRightCurve(new_v, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
}

/**
 * 
*/
void solveRpLpRm(StructPoseReal goal_pose, 
                 StructPoseReal start_pose, 
                 StructDubinsCurveCCCvalcollection* ptr_dubins_para, 
                 StructRSPathResult* ptr_path, 
                 StructSamplingProperties* ptr_sample )
{
    ptr_path->setValid(ptr_dubins_para->valid);
    if (! ptr_dubins_para->valid) return;
    double new_t = ptr_dubins_para->t;
    double new_u = ptr_dubins_para->u;
    double new_v = ptr_dubins_para->v - M_PI*2.0;
    ptr_path->path_length_unitless = std::abs(new_t) + std::abs(new_u) + std::abs(new_v) ;

    double _limit = 1.5 * M_PI;
    if (std::abs(new_t) > _limit || std::abs(new_u) > _limit || std::abs(new_v) > _limit)
    {
        ptr_dubins_para->valid = false;
        return;
    }
    sampleOnRightCurve(new_t, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnLeftCurve(new_u, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnRightCurve(new_v, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
}


void solveRpLmRp(StructPoseReal goal_pose, 
                 StructPoseReal start_pose, 
                 StructDubinsCurveCCCvalcollection* ptr_dubins_para, 
                 StructRSPathResult* ptr_path, 
                 StructSamplingProperties* ptr_sample )
{
    ptr_path->setValid(ptr_dubins_para->valid);
    if (! ptr_dubins_para->valid) return;
    double new_t = ptr_dubins_para->t;
    double new_u = ptr_dubins_para->u-M_PI*2.0;
    double new_v = ptr_dubins_para->v;
    ptr_path->path_length_unitless = std::abs(new_t) + std::abs(new_u) + std::abs(new_v) ;

    double _limit = 1.5 * M_PI;
    if (std::abs(new_t) > _limit || std::abs(new_u) > _limit || std::abs(new_v) > _limit)
    {
        ptr_dubins_para->valid = false;
        return;
    }
    sampleOnRightCurve(new_t, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnLeftCurve(new_u, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnRightCurve(new_v, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
}



void solveRpLmRm(StructPoseReal goal_pose, 
                 StructPoseReal start_pose, 
                 StructDubinsCurveCCCvalcollection* ptr_dubins_para, 
                 StructRSPathResult* ptr_path, 
                 StructSamplingProperties* ptr_sample )
{
    ptr_path->setValid(ptr_dubins_para->valid);
    if (! ptr_dubins_para->valid) return;
    double new_t = ptr_dubins_para->t;
    double new_u = ptr_dubins_para->u-M_PI*2.0;
    double new_v = ptr_dubins_para->v-M_PI*2.0;
    ptr_path->path_length_unitless = std::abs(new_t) + std::abs(new_u) + std::abs(new_v) ;

    double _limit = 1.5 * M_PI;
    if (std::abs(new_t) > _limit || std::abs(new_u) > _limit || std::abs(new_v) > _limit)
    {
        ptr_dubins_para->valid = false;
        return;
    }
    sampleOnRightCurve(new_t, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnLeftCurve(new_u, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnRightCurve(new_v, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
}



void solveRmLpRp(StructPoseReal goal_pose, 
                 StructPoseReal start_pose, 
                 StructDubinsCurveCCCvalcollection* ptr_dubins_para, 
                 StructRSPathResult* ptr_path, 
                 StructSamplingProperties* ptr_sample )
{
    ptr_path->setValid(ptr_dubins_para->valid);
    if (! ptr_dubins_para->valid) return;
    double new_t = ptr_dubins_para->t-M_PI*2.0;
    double new_u = ptr_dubins_para->u;
    double new_v = ptr_dubins_para->v;
    ptr_path->path_length_unitless = std::abs(new_t) + std::abs(new_u) + std::abs(new_v) ;

    double _limit = 1.5 * M_PI;
    if (std::abs(new_t) > _limit || std::abs(new_u) > _limit || std::abs(new_v) > _limit)
    {
        ptr_dubins_para->valid = false;
        return;
    }
    sampleOnRightCurve(new_t, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnLeftCurve(new_u, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnRightCurve(new_v, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
}



void solveRmLpRm(StructPoseReal goal_pose, 
                 StructPoseReal start_pose, 
                 StructDubinsCurveCCCvalcollection* ptr_dubins_para, 
                 StructRSPathResult* ptr_path, 
                 StructSamplingProperties* ptr_sample )
{
    ptr_path->setValid(ptr_dubins_para->valid);
    if (! ptr_dubins_para->valid) return;
    double new_t = ptr_dubins_para->t-M_PI*2.0;
    double new_u = ptr_dubins_para->u;
    double new_v = ptr_dubins_para->v-M_PI*2.0;
    ptr_path->path_length_unitless = std::abs(new_t) + std::abs(new_u) + std::abs(new_v) ;

    double _limit = 1.5 * M_PI;
    if (std::abs(new_t) > _limit || std::abs(new_u) > _limit || std::abs(new_v) > _limit)
    {
        ptr_dubins_para->valid = false;
        return;
    }
    sampleOnRightCurve(new_t, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnLeftCurve(new_u, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnRightCurve(new_v, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
}



void solveRmLmRp(StructPoseReal goal_pose, 
                 StructPoseReal start_pose, 
                 StructDubinsCurveCCCvalcollection* ptr_dubins_para, 
                 StructRSPathResult* ptr_path, 
                 StructSamplingProperties* ptr_sample )
{
    ptr_path->setValid(ptr_dubins_para->valid);
    if (! ptr_dubins_para->valid) return;
    double new_t = ptr_dubins_para->t-M_PI*2.0;
    double new_u = ptr_dubins_para->u-M_PI*2.0;
    double new_v = ptr_dubins_para->v;
    ptr_path->path_length_unitless = std::abs(new_t) + std::abs(new_u) + std::abs(new_v) ;

    double _limit = 1.5 * M_PI;
    if (std::abs(new_t) > _limit || std::abs(new_u) > _limit || std::abs(new_v) > _limit)
    {
        ptr_dubins_para->valid = false;
        return;
    }
    sampleOnRightCurve(new_t, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnLeftCurve(new_u, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnRightCurve(new_v, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
}



void solveRmLmRm(StructPoseReal goal_pose, 
                 StructPoseReal start_pose, 
                 StructDubinsCurveCCCvalcollection* ptr_dubins_para, 
                 StructRSPathResult* ptr_path, 
                 StructSamplingProperties* ptr_sample )
{
    ptr_path->setValid(ptr_dubins_para->valid);
    if (! ptr_dubins_para->valid) return;
    double new_t = ptr_dubins_para->t-M_PI*2.0;
    double new_u = ptr_dubins_para->u-M_PI*2.0;
    double new_v = ptr_dubins_para->v-M_PI*2.0;
    ptr_path->path_length_unitless = std::abs(new_t) + std::abs(new_u) + std::abs(new_v) ;

    double _limit = 1.5 * M_PI;
    if (std::abs(new_t) > _limit || std::abs(new_u) > _limit || std::abs(new_v) > _limit)
    {
        ptr_dubins_para->valid = false;
        return;
    }
    sampleOnRightCurve(new_t, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnLeftCurve(new_u, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
    sampleOnRightCurve(new_v, start_pose.yaw, start_pose.x, start_pose.y, ptr_path, ptr_sample);
}


#endif