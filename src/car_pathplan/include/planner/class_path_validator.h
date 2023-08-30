#ifndef CLASS_PATH_VALIDATOR
#define CLASS_PATH_VALIDATOR

/**
 * @file class_path_validator.h
 * @author Mingjie
 * @brief
 * @version 0.1
 * @date 2023-01-18
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <iostream>
#include <deque>
#include <array>
#include <math.h>
#include <vector>

#include "../car_pathplan/class_pose.h"

using std::array;
using std::deque;
using std::vector;

/**
 * @brief A class for verify if the given path is valid or not.
 *
 */
class ClassPathValidator
{
private:
    deque<array<double, 3>> *path_ptr_;
    ClassPose2D robot_pose_;
    // double robot_x_, robot_y_, robot_yaw_;
    double distance_tolerance_;
    double angle_tolerance_;

    double k_pi2_;

    bool check_robot_close_to_path();

    double mod_2pi(double angle);

    double euclidean_distance(double x1, double y1, double x2, double y2);
    double min_angle_diffrence(double a1, double a2);

public:
    ClassPathValidator();
    ~ClassPathValidator();
    void setup_data(deque<array<double, 3>> *pathptr, double robot_x, double robot_y, double robot_yaw);
    void setup_para(double distance_tole, double angle_tole);
    void validate(bool &result);
};

ClassPathValidator::ClassPathValidator()
{
    distance_tolerance_ = 0.05;
    angle_tolerance_ = M_PI / 6.0;
    k_pi2_ = M_PI * 2.0;
}

ClassPathValidator::~ClassPathValidator()
{
}

void ClassPathValidator::setup_data(deque<array<double, 3>> *pathptr, double robot_x, double robot_y, double robot_yaw)
{
    path_ptr_ = pathptr;
    robot_x_ = robot_x;
    robot_y_ = robot_y;
    robot_yaw_ = robot_yaw;
}

void ClassPathValidator::setup_para(double distance_tole, double angle_tole)
{
    distance_tolerance_ = distance_tole;
    angle_tolerance_ = angle_tole;
}

void ClassPathValidator::validate(bool &result)
{
    if (!check_robot_close_to_path())
        result = false;
}

bool ClassPathValidator::check_robot_close_to_path()
{
    // vector< array<double, 3> > points_near_robot;
    for (auto pt : *path_ptr_)
    {
        double distance = euclidean_distance(pt[0], pt[1], robot_x_, robot_y_);
        if (distance <= distance_tolerance_)
        {
            if (min_angle_diffrence(robot_yaw_, pt[2]) < angle_tolerance_)
            {
                // points_near_robot.push_back( pt );
                return true;
            }
        }
    }
    return false;
}

inline double ClassPathValidator::euclidean_distance(double x1, double y1, double x2, double y2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}

double ClassPathValidator::min_angle_diffrence(double a1, double a2)
{
    a1 = mod_2pi(a1);
    a2 = mod_2pi(a2);
    double amax, amin, diff;
    if (a1 > a2)
    {
        amax = a1;
        amin = a2;
    }
    else if (a2 > a1)
    {
        amax = a2;
        amin = a1;
    }
    else
    {
        return 0.0;
    }

    diff = amax - amin;
    if (diff > M_PI)
    {
        diff = k_pi2_ - diff;
    }

    return diff;
}

double ClassPathValidator::mod_2pi(double angle)
{
    double a_out = std::remainder(angle, k_pi2_);
    if (a_out > 0.0)
    {
        return a_out;
    }
    else
    {
        return a_out + k_pi2_;
    }
}

#endif
