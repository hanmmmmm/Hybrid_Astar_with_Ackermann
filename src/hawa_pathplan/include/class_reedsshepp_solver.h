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
 * @file class_reedsshepp_solver.h
 * @author Mingjie
 * @brief This class is the entry to the RS module.
 * @version 0.2
 * @date 2023-11-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */


/**
 * @note Only the CSC and CCC curves are completed. All the rest types were working but were made for the previous
 * version of code before refactoring. So they still need to be refactored in order to match with the current 
 * version. 
*/

#ifndef CLASS_HAWA_REEDSSHEPP_FINDER_H
#define CLASS_HAWA_REEDSSHEPP_FINDER_H

#include "common_includes.h"

#include "hawa_reeds_shepp_curves/reedshepp_path.h"
#include "reedsshepp_tools.h"

#include "utils/class_utils__converters.h"

#include "hawa_reeds_shepp_curves/rs_LSL.h"
#include "hawa_reeds_shepp_curves/rs_LSR.h"
#include "hawa_reeds_shepp_curves/rs_LRL.h"
#include "hawa_reeds_shepp_curves/rs_RLR.h"


namespace hawa
{

/*
This is my implementation of Reeds-Sheep curve. 
Most parts are following the paper. 
One exceptation is that L+R+L+ L-R-L- R+L+R+ R-L-R- are also included;
so here are 52=48+4 types of curves are being considered;
And the CCC types are computed using Dubins Curve formula. 

The result is a vector of my custom path type, defined in another header file. 
Each 'path' contains the type-word, unitless length, sampled waypoints (x,y,yaw), valid (bool).
Sorted in length increasing order. 
*/

class ClassReedsSheppSolver
{
private:

    StructPoseReal m_start_pose_;
    StructPoseReal m_goal_pose_;
    StructPoseReal m_goal_pose_processed_;

    StructSamplingProperties m_sampling_properites_;

    double calc_weighted_length(std::vector<double> list_in);

public:
    ClassReedsSheppSolver(){};

    ~ClassReedsSheppSolver(){};

    ClassReedSheppPath::PathCollection m_all_curves_;

    std::vector<StructRSPathResult> m_vector_path_results_;  

    void setup( StructPoseReal start_pose, StructPoseReal goal_pose );

    void search( );

    // void get_path( std::array<double,3> start_pose, std::array<double,3> goal_pose , std::string path_type, std::vector<double>& path_angle, bool& valid);

    std::array<double, 2> calcFixFrameDxDy(const double angle_change, const double pose_yaw );

};


}



#endif