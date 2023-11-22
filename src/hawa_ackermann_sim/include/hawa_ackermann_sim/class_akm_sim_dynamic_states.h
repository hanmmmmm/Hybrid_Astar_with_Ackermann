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

/**
 * @brief Get the x-component of the linear speed, in world coordinate.
 * @return The value in meter per second.
*/
double DynamicsStates::getVxMps()
{
    return m_linear_vb_mps_actual_ * std::cos(m_yaw_rad_);
}

/**
 * @brief Get the y-component of the linear speed, in world coordinate.
 * @return The value in meter per second.
*/
double DynamicsStates::getVyMps()
{
    return m_linear_vb_mps_actual_ * std::sin(m_yaw_rad_);
}

/**
 * @brief Call this function to set the expected linear speed, in meter per second. If the value is out of
 * the range of limits, then it will be trauncated at the limits.
 * @param target_mps The expected value.
*/
void DynamicsStates::setTargetLinearVbMps(double target_mps)
{
    target_mps = std::min(target_mps, m_limits_.max_linear_v_mps);
    target_mps = std::max(target_mps, m_limits_.min_linear_v_mps);
    m_linear_vb_mps_target_ = target_mps;
}

/**
 * @brief Call this function to set the expected steer angle, in radians. If the value is out of
 * the range of limits, then it will be trauncated at the limits.
 * @param target_rad The expected value.
*/
void DynamicsStates::setTargetSteerRad(double target_rad)
{
    target_rad = std::min(target_rad, m_limits_.max_steer_rad);
    target_rad = std::max(target_rad, m_limits_.min_steer_rad);
    m_steer_rad_target_ = target_rad;
}

/**
 * @brief Call this function to calculate the actual value of the linear speed. The actual speed might be
 * different from the expected value because the acceleration is also considerred. 
 * @param dt The time duration since the previous update. In seconds.
*/
void DynamicsStates::calcActualLinearMps(const double dt)
{
    if (m_linear_vb_mps_actual_ < m_linear_vb_mps_target_)
    {
        m_linear_vb_mps_actual_ = std::min(m_linear_vb_mps_target_, 
                                           m_linear_vb_mps_actual_ + m_limits_.linear_a_mpss * dt);
    }
    // else if (m_linear_vb_mps_actual_ > m_linear_vb_mps_target_)
    else
    {
        m_linear_vb_mps_actual_ = std::max(m_linear_vb_mps_target_, 
                                           m_linear_vb_mps_actual_ - m_limits_.linear_a_mpss * dt);
    }
}

/**
 * @brief Call this function to calculate the actual value of the steer angle. The actual value might be
 * different from the expected value because the rate of steer change is limited. 
 * @param dt The time duration since the previous update. In seconds.
*/
void DynamicsStates::calcActualSteerRad(const double dt)
{
    if (m_steer_rad_actual_ < m_steer_rad_target_)
    {
        m_steer_rad_actual_ = std::min(m_steer_rad_target_, 
                                       m_steer_rad_actual_ + m_limits_.steer_rate_radps * dt);
    }
    else if (m_steer_rad_actual_ > m_steer_rad_target_)
    {
        m_steer_rad_actual_ = std::max(m_steer_rad_target_, 
                                       m_steer_rad_actual_ - m_limits_.steer_rate_radps * dt);
    }
}

/**
 * @brief This function should be called after the expected values are set. 
 * This will update the velocity and pose.
 * @param dt The time duration since the previous update. In seconds.
*/
void DynamicsStates::updateAllStates(const double dt)
{
    calcActualLinearMps(dt);
    calcActualSteerRad(dt);
    
    m_x_meter_ += getVxMps() * dt;
    m_y_meter_ += getVyMps() * dt;

    m_angular_wb_radps_actual_ = m_linear_vb_mps_actual_ * std::tan(m_steer_rad_actual_) / m_axle_distance_;
    
    m_yaw_rad_ += mod2pi( m_angular_wb_radps_actual_ * dt );
    m_yaw_rad_ = mod2pi(m_yaw_rad_);
}





#endif