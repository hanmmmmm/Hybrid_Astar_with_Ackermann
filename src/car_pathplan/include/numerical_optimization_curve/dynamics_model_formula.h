#ifndef DYNAMICS_MODEL_FORMULA_H
#define DYNAMICS_MODEL_FORMULA_H

#include <iostream>
#include <vector>
#include <cmath>
#include <assert.h>

#include <cppad/cppad.hpp>

#include "class_model_limits.h"
#include "helper_state_indices.h"


using CppAD::AD;

typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
typedef CPPAD_TESTVECTOR(double) Dvector;


ADvector update_robot_model(const ADvector model_t0, const double duration_sec)
{
    assert(model_t0.size() == model_states_total_number);

    AD<double> _t0_x = model_t0[index_pose_x];
    AD<double> _t0_y = model_t0[index_pose_y];
    AD<double> _t0_yaw = model_t0[index_pose_yaw];

    std::cout << "_t0 x y yaw  " << _t0_x << " " << _t0_y << " " << _t0_yaw << std::endl;

    AD<double> _t0_v = model_t0[index_linear_v];
    AD<double> _t0_a = model_t0[index_linear_a];
    // AD<double> _t0_j = model_t0[linear_j];

    // AD<double> _t0_w = model_t0[aglr_w];
    AD<double> _t0_s = model_t0[index_aglr_str_angle];
    AD<double> _t0_sr = model_t0[index_aglr_str_rate];


    AD<double> _new_acc = _t0_a;
    // if (_new_acc > )
    _new_acc = std::min(_new_acc, acc_positive_limit_);
    _new_acc = std::max(_new_acc, acc_negative_limit_);

    AD<double> _new_v = _t0_v + _new_acc*duration_sec;
    _new_v = std::min(_new_v, velocity_forward_limit_);
    _new_v = std::max(_new_v, velocity_reverse_limit_);

    // AD<double> _new_jerk = _t0_j;

    AD<double> _new_steer_rate = _t0_sr;
    _new_steer_rate = std::min(_new_steer_rate, steer_rate_positive_limit_);
    _new_steer_rate = std::max(_new_steer_rate, steer_rate_negative_limit_);

    AD<double> _new_steer_angle = _t0_s + _new_steer_rate * duration_sec;
    _new_steer_angle = std::min(_new_steer_angle, steer_angle_positive_limit_);
    _new_steer_angle = std::max(_new_steer_angle, steer_angle_negative_limit_);


    // AD<double> _new_angular_velo = _new_v * tan(_new_steer_angle) / axle_distance_;
    AD<double> _new_angular_velo = _t0_v * CppAD::tan(_t0_s) / axle_distance_;
    // _new_angular_velo = std::min(_new_angular_velo, angular_positive_limit_);
    // _new_angular_velo = std::max(_new_angular_velo, angular_negative_limit_);

    AD<double> _d_yaw = _new_angular_velo * duration_sec;
    AD<double> _new_yaw = _t0_yaw + _d_yaw;

    AD<double> _turn_radius_meter, _body_frame_dx, _body_frame_dy ;

    if (CppAD::abs(_t0_s) > 0.01)
    {
        _turn_radius_meter = axle_distance_ / CppAD::tan(_t0_s);
        _body_frame_dx = _turn_radius_meter * CppAD::sin(_d_yaw);
        _body_frame_dy = _turn_radius_meter - _turn_radius_meter * CppAD::cos(_d_yaw);
    }
    else
    {
        _body_frame_dx = _t0_v * duration_sec;
        _body_frame_dy = 0.0;
    }
    // AD<double> _turn_radius_meter = axle_distance_ / CppAD::tan(_t0_s);
    // AD<double> _body_frame_dx = _turn_radius_meter * CppAD::sin(_d_yaw);
    // AD<double> _body_frame_dy = _turn_radius_meter - _turn_radius_meter * CppAD::cos(_d_yaw);

    AD<double> _fix_frame_dx = _body_frame_dx * CppAD::cos(_t0_yaw) - _body_frame_dy * CppAD::sin(_t0_yaw);
    AD<double> _fix_frame_dy = _body_frame_dx * CppAD::sin(_t0_yaw) + _body_frame_dy * CppAD::cos(_t0_yaw);

    // std::cout << "fix_frame_d x y  " << _fix_frame_dx << " " << _fix_frame_dy << std::endl;

    // AD<double> _avg_v = (_new_v + _t0_v)/ 2.0;
    // AD<double> _new_x = _t0_x + _avg_v * duration_sec * cos(_t0_yaw);
    // AD<double> _new_y = _t0_y + _avg_v * duration_sec * sin(_t0_yaw);

    AD<double> _new_x = _t0_x + _fix_frame_dx;
    AD<double> _new_y = _t0_y + _fix_frame_dy;

    // std::cout << "_t0 x y yaw  " << _t0_x << " " << _t0_y << " " << _t0_yaw << std::endl;

    ADvector _result(model_states_total_number);
    _result[index_pose_x] = _new_x;
    _result[index_pose_y] = _new_y;
    _result[index_pose_yaw] = _new_yaw;
    _result[index_linear_v] = _new_v;
    _result[index_linear_a] = _new_acc;
    _result[index_aglr_str_angle] = _new_steer_angle;
    _result[index_aglr_str_rate] = _new_steer_rate;

    return _result;
}


struct StructModelStates
{
    double x;
    double y;
    double yaw;

    double v;
    double a;

    double str_angl;
    double str_rate;
};




// std::vector<double> update_robot_model(const std::vector<double> model_t0, const double duration_sec)
// {
//     assert(model_t0.size() == model_states_total_number);

//     double _t0_x = model_t0[index_pose_x];
//     double _t0_y = model_t0[index_pose_y];
//     double _t0_yaw = model_t0[index_pose_yaw];

//     double _t0_v = model_t0[index_linear_v];
//     double _t0_a = model_t0[index_linear_a];
//     // double _t0_j = model_t0[linear_j];

//     // double _t0_w = model_t0[aglr_w];
//     double _t0_s = model_t0[index_aglr_str_angle];
//     double _t0_sr = model_t0[index_aglr_str_rate];

//     double _new_acc = _t0_a;
//     _new_acc = std::min(_new_acc, acc_positive_limit_);
//     _new_acc = std::max(_new_acc, acc_negative_limit_);

//     double _new_v = _t0_v + _new_acc*duration_sec;
//     _new_v = std::min(_new_v, velocity_forward_limit_);
//     _new_v = std::max(_new_v, velocity_reverse_limit_);

//     // double _new_jerk = _t0_j;

//     double _new_steer_rate = _t0_sr;
//     _new_steer_rate = std::min(_new_steer_rate, steer_rate_positive_limit_);
//     _new_steer_rate = std::max(_new_steer_rate, steer_rate_negative_limit_);

//     double _new_steer_angle = _t0_s + _new_steer_rate * duration_sec;
//     _new_steer_angle = std::min(_new_steer_angle, steer_angle_positive_limit_);
//     _new_steer_angle = std::max(_new_steer_angle, steer_angle_negative_limit_);

//     double _new_angular_velo = _new_v * tan(_new_steer_angle) / axle_distance_;
//     // _new_angular_velo = std::min(_new_angular_velo, angular_positive_limit_);
//     // _new_angular_velo = std::max(_new_angular_velo, angular_negative_limit_);

//     double _d_yaw = _new_angular_velo * duration_sec;
//     double _new_yaw = _t0_yaw + _d_yaw;

//     double _avg_v = (_new_v + _t0_v)/ 2.0;

//     double _new_x = _t0_x + _avg_v * duration_sec * cos(_t0_yaw);
//     double _new_y = _t0_y + _avg_v * duration_sec * sin(_t0_yaw);

//     std::vector<double> _result(model_states_total_number);
//     _result[index_pose_x] = _new_x;
//     _result[index_pose_y] = _new_y;
//     _result[index_pose_yaw] = _new_x;
//     _result[index_linear_v] = _new_v;
//     _result[index_linear_a] = _new_acc;
//     _result[index_aglr_str_angle] = _new_steer_angle;
//     _result[index_aglr_str_rate] = _new_steer_rate;

//     return _result;
// }




#endif