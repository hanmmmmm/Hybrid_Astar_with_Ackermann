#ifndef CLASS_ELEMENTAL_POSE2D_H
#define CLASS_ELEMENTAL_POSE2D_H

/**
 * @file class_pose2d.h
 * @author Mingjie
 * @brief
 * @version 0.1
 * @date 2023-01-18
 *
 * @copyright Copyright (c) 2023
 *
 */

// #include <iostream>

/**
 * @brief
 *
 */
class ClassPose2D
{
private:
public:
    ClassPose2D();
    ClassPose2D(double x_in, double y_in, double yaw_in);
    void set(double x_in, double y_in, double yaw_in);
    ~ClassPose2D();

    double x, y, yaw;
};

ClassPose2D::ClassPose2D(double x_in, double y_in, double yaw_in) : x(x_in), y(y_in), yaw(yaw_in)
{
}

void ClassPose2D::set(double x_in, double y_in, double yaw_in)
{
    x = x_in;
    y = y_in;
    yaw = yaw_in;
}

ClassPose2D::~ClassPose2D()
{
}

#endif