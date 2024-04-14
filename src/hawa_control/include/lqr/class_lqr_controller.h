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

#ifndef HAWA_CONTROL_INCLUDE_LQR_CLASS_LQR_CONTROLLER_H_
#define HAWA_CONTROL_INCLUDE_LQR_CLASS_LQR_CONTROLLER_H_

#include <Eigen/Dense>
#include <Eigen/Core>

#include <iostream> 
#include <math.h>

#include "common/class_elemental_pose2d.h"

namespace hawa
{

/**
 * @brief This is my LQR controller implementation. 
 * The controller is trying to minimize the error between the robot and the reference point.
 * 
 * The states varialbes are 
 *  X  = [robot_x   - ref_point_x]
 *  Y  = [robot_y   - ref_point_y]
 * Yaw = [robot_yaw - ref_point_yaw]
 * 
 * The control variables are
 *   v   = [v_result     - v_ref] 
 * steer = [steer_result - steer_ref]
 * 
 * v_result, and steer_result are the control values that will be applied to the robot.
 * 
 * 
*/
class ClassLQRController
{
private:

    Eigen::Matrix3d A;              // state matrix
    Eigen::Matrix<double, 3, 2> B;  // control matrix
    Eigen::Matrix3d At;             // transpose of state matrix
    Eigen::Matrix<double, 2, 3> Bt; // transpose of control matrix
    
    Eigen::Matrix3d Q;              // cost matrix for state
    Eigen::Matrix2d R;              // cost matrix for control

    Eigen::Matrix<double, 2, 3> K;  // gain matrix
    Eigen::Matrix3d P;              // solution of Riccati equation

    Eigen::Vector3d s;              // the given state vector. 
    Eigen::Vector3d s_ref;          // reference state vector

    Eigen::Vector2d u;              // control vector
    Eigen::Vector2d u_ref;          // the reference control vector. Usually set by the target velocity and steering angle.
    Eigen::Vector2d u_result;       // the result that will be applied to the robot
    Eigen::Vector2d u_err;          // an intermediate variable, the difference between the reference and the result.

    double mk_dt;                   // time step in seconds
    double mk_axle_distance;        // distance between the front wheels and the rear wheels, in meters

    int mk_riccati_max_iter;

private:

    void solveRiccatiEquation();

    void solveK();

    void generateA();

    void generateB();

    void solveU();

    double normalizeAngle(double angle);


public:

    ClassLQRController(const double l, const double dt, const int ricatti_ite);

    ~ClassLQRController();

    void setVref(const double v_ref);
    
    void setSteerRef(const double steer_ref);
    
    void setStateRef(ClassPose2D ref_pose);

    void setState(const ClassPose2D& pose);
    
    void generateQ(double q1, double q2, double q3);

    void generateR(double r1, double r2);

    void solve();

    double getSteer();
    
    double getV();
};


} // namespace hawa





#endif