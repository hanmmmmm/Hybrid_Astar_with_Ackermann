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
 * @file class_akm_sim_dynamic_states.h
 * @author Mingjie
 * @brief This is a class managing the dynamics model parameters.
 * @version 0.3
 * @date 2023-11-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#ifndef HAWA_CLASS_ALM_SIM_DYNAMIC_STATES
#define HAWA_CLASS_ALM_SIM_DYNAMIC_STATES

#include <iostream>
#include <math.h>

#include "hawa_sim_tools.h"

namespace hawa
{

struct DynamicsLimits
{
    const double max_linear_v_mps = 1.7;
    const double min_linear_v_mps = -1.7;
    const double linear_a_mpss = 0.8;

    const double max_steer_rad = 0.5;
    const double min_steer_rad = -0.5;
    const double steer_rate_radps = 1.0;
};

// Everything in this class is set to be public because I think this class is quite simple and 
// the parameters will be used frequently in the simulation node, so it's not really nesscasry to
// make them private and add the set/get function for every parameter. 
class DynamicsStates
{
public:
    double m_x_meter_ = 0;
    double m_y_meter_ = 0;
    double m_yaw_rad_ = 0;

    double m_steer_rad_actual_ = 0;
    double m_steer_rad_target_ = 0;

    double m_linear_vb_mps_actual_ = 0;
    double m_linear_vb_mps_target_ = 0;

    double m_angular_wb_radps_actual_ = 0;
    // double angular_wb_radps_target = 0;

    double m_axle_distance_ = 0.25;

    DynamicsLimits m_limits_;

    double getVxMps();
    double getVyMps();

    void setTargetLinearVbMps(double target_mps);
    void setTargetSteerRad(double target_rad);

    void calcActualLinearMps(const double dt);
    void calcActualSteerRad(const double dt);

    void updateAllStates(const double dt);
};


}





#endif