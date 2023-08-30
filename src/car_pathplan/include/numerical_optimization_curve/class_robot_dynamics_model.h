#ifndef ROBOT_DYNAMICS_MODEL_H
#define ROBOT_DYNAMICS_MODEL_H


#include <iostream>
#include <cmath>


const static double limit1 = 0.5;



class ClassRobotDynamicsModel
{
private:
    struct StructModelStates
    {
        double x;
        double y;
        double yaw;

        double v;  // linear velocity, meter per second;
        double a;  // if using a as independent variable, then ignore j in calculation.
        double j;  // if using j as independent variable, then a becomes a dependent variable.
        
        double w;  // angular velocity, rad per second;
        double steer_angle;  // the angle of virtual center steering wheel;
        double steer_rate;  // the rate of change of steer wheel angle.
    };

private:
    StructModelStates m_states_;

    double m_axle_distance_;

public:
    ClassRobotDynamicsModel();
    ~ClassRobotDynamicsModel();

    double m_velocity_forward_limit_;
    double m_velocity_reverse_limit_;

    double m_acc_positive_limit_;
    double m_acc_negative_limit_;

    double m_jerk_positive_limit_;
    double m_jerk_negative_limit_;

    double m_angular_positive_limit_;
    double m_angular_negative_limit_;

    double m_steer_angle_positive_limit_;
    double m_steer_angle_negative_limit_;

    double m_steer_rate_positive_limit_;
    double m_steer_rate_negative_limit_;



    void update_states(const double acc, const double steerRate, const double duration_sec);

};

ClassRobotDynamicsModel::ClassRobotDynamicsModel()
{
    m_velocity_forward_limit_ = 0.2;
    m_velocity_reverse_limit_ = -0.2;

    m_acc_positive_limit_ = 0.2;
    m_acc_negative_limit_ = 0.2;

    m_jerk_positive_limit_ = 0.1;
    m_jerk_negative_limit_ = 0.1;

    m_angular_positive_limit_ = 0.5;
    m_angular_negative_limit_ = -0.5;

    m_steer_angle_positive_limit_ = 0.5;
    m_steer_angle_negative_limit_ = -0.5;

    m_steer_rate_positive_limit_ = 0.5;
    m_steer_rate_negative_limit_ = -0.5;

    m_axle_distance_ = 0.25;
}

ClassRobotDynamicsModel::~ClassRobotDynamicsModel()
{
}

void ClassRobotDynamicsModel::update_states(const double acc, const double steerRate, const double duration_sec)
{
    double _new_acc = acc;
    _new_acc = std::min(_new_acc, m_acc_positive_limit_);
    _new_acc = std::max(_new_acc, m_acc_negative_limit_);

    double _new_v = m_states_.v + _new_acc*duration_sec;
    _new_v = std::min(_new_v, m_velocity_forward_limit_);
    _new_v = std::max(_new_v, m_velocity_reverse_limit_);
    // if (_new_v > m_velocity_forward_limit_)
    // {
    //     double _actual_acc_duration = (m_velocity_forward_limit_ - m_states_.v)/_temp_acc;
    // }

    double _new_steer_rate = steerRate;
    _new_steer_rate = std::min(_new_steer_rate, m_steer_rate_positive_limit_);
    _new_steer_rate = std::max(_new_steer_rate, m_steer_rate_negative_limit_);

    double _new_steer_angle = m_states_.steer_angle + _new_steer_rate * duration_sec;
    _new_steer_angle = std::min(_new_steer_angle, m_steer_angle_positive_limit_);
    _new_steer_angle = std::max(_new_steer_angle, m_steer_angle_negative_limit_);

    double _new_angular_velo = _new_v * tan(_new_steer_angle) / m_axle_distance_;
    _new_angular_velo = std::min(_new_angular_velo, m_angular_positive_limit_);
    _new_angular_velo = std::max(_new_angular_velo, m_angular_negative_limit_);

    double _d_yaw = _new_angular_velo * duration_sec;

    double _new_yaw = m_states_.yaw + _d_yaw;

}







#endif