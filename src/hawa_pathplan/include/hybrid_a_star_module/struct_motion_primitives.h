#ifndef STRUCT_MOTION_PRIMITIVES_H
#define STRUCT_MOTION_PRIMITIVES_H


#include <vector>
#include <math.h>

#include "hybrid_astar_tools.h"

struct MotionModelParameters
{
    double step_length_forward;
    double step_length_backward;
    double turning_radius;
    double turning_angle;
};


enum EnumForwardBackward
{
    forward,
    backward
};

struct MotionPrimitive
{
    double dx;
    double dy;
    double dyaw;
    double edge_cost;
    EnumForwardBackward direction;
    EnumHAMotionType motion_type;
};

struct CostMultipler
{
    double forward = 1.0;
    double backward = 1.0;
    double mild_steer = 1.0;
    double fast_steer = 1.0;
};

struct StructMotionModel
{
    std::vector<MotionPrimitive> motions;
    MotionModelParameters parameters;    
    CostMultipler cost_multipler;

    void prepare_model();
};

void StructMotionModel::prepare_model()
{
    MotionPrimitive _temp_mp;
    double angle, curve_dx, curve_dy;

    angle = std::abs(parameters.step_length_forward / parameters.turning_radius);

    curve_dx = parameters.turning_radius * std::sin(angle);
    curve_dy = parameters.turning_radius * (1.0 - std::cos(angle));

    // first 3 options are moving forward
    _temp_mp.edge_cost = parameters.step_length_forward * cost_multipler.forward;
    _temp_mp.direction = EnumForwardBackward::forward;

    // to front left
    _temp_mp.dx = curve_dx;
    _temp_mp.dy = curve_dy;
    _temp_mp.dyaw = angle;
    _temp_mp.motion_type = EnumHAMotionType::F_L;
    motions.push_back(_temp_mp);

    // to front center
    _temp_mp.dx = parameters.step_length_forward;
    _temp_mp.dy = 0;
    _temp_mp.dyaw = 0;
    _temp_mp.motion_type = EnumHAMotionType::F_S;
    motions.push_back(_temp_mp);

    // to front right
    _temp_mp.dx = curve_dx;
    _temp_mp.dy = -curve_dy;
    _temp_mp.dyaw = -angle;
    _temp_mp.motion_type = EnumHAMotionType::F_R;
    motions.push_back(_temp_mp);

    angle = std::abs(parameters.step_length_backward / parameters.turning_radius);

    curve_dx = parameters.turning_radius * sin(angle);
    curve_dy = parameters.turning_radius * (1.0 - cos(angle));

    // next 3 options are moving backward
    _temp_mp.edge_cost = parameters.step_length_backward * cost_multipler.backward;
    _temp_mp.direction = EnumForwardBackward::backward;

    // to back right
    _temp_mp.dx = -curve_dx;
    _temp_mp.dy = -curve_dy;
    _temp_mp.dyaw = angle;
    _temp_mp.motion_type = EnumHAMotionType::R_R;
    motions.push_back(_temp_mp);

    // to back center
    _temp_mp.dx = -parameters.step_length_backward;
    _temp_mp.dy = 0;
    _temp_mp.dyaw = 0;
    _temp_mp.motion_type = EnumHAMotionType::R_S;
    motions.push_back(_temp_mp);

    // to back left
    _temp_mp.dx = -curve_dx;
    _temp_mp.dy = curve_dy;
    _temp_mp.dyaw = -angle;
    _temp_mp.motion_type = EnumHAMotionType::R_L;
    motions.push_back(_temp_mp);
}










#endif