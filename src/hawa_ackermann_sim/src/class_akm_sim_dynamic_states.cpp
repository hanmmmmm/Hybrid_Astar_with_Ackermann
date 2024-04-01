
#include "hawa_ackermann_sim/class_akm_sim_dynamic_states.h"

namespace hawa
{

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
    
    // m_yaw_rad_ += mod2pi( m_angular_wb_radps_actual_ * dt );
    // m_yaw_rad_ = mod2pi(m_yaw_rad_);

    // auto tools = ClassHawaSimTools();
    // m_yaw_rad_ += tools.mod2pi( m_angular_wb_radps_actual_ * dt );
    // m_yaw_rad_ = tools.mod2pi(m_yaw_rad_);

    m_yaw_rad_ += ClassHawaSimTools::mod2pi( m_angular_wb_radps_actual_ * dt );
    m_yaw_rad_ = ClassHawaSimTools::mod2pi(m_yaw_rad_);
}




}