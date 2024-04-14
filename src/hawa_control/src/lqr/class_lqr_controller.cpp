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

#include "lqr/class_lqr_controller.h"

namespace hawa
{

/**
 * @brief Construct function for LQR controller
*/
ClassLQRController::ClassLQRController(const double l, const double dt, const int ricatti_ite) 
: mk_axle_distance(l), mk_dt(dt), mk_riccati_max_iter(ricatti_ite) 
{
    std::cout << "LQR controller initialized with " ;
    std::cout << "axle distance: " << mk_axle_distance << ", ";
    std::cout << "time step: " << mk_dt << ", ";
    std::cout << "Riccati iteration: " << mk_riccati_max_iter << std::endl;
}


ClassLQRController::~ClassLQRController() 
{
}

/**
 * @brief Set the reference speed for the controller. meter per second
*/
void ClassLQRController::setVref(const double v_ref) 
{ 
    u_ref(0) = v_ref; 
}

/**
 * @brief Set the reference steering angle for the controller. radian
*/
void ClassLQRController::setSteerRef(const double steer_ref) 
{ 
    u_ref(1) = steer_ref;
}

/**
 * @brief Set the reference state for the controller.
*/
void ClassLQRController::setStateRef(ClassPose2D ref_pose)
{
    s_ref(0) = ref_pose.x;
    s_ref(1) = ref_pose.y;
    s_ref(2) = ref_pose.yaw;
}

/**
 * @brief Set the current state for the controller.
*/
void ClassLQRController::setState(const ClassPose2D& pose)
{
    s(0) = pose.x;
    s(1) = pose.y;
    s(2) = pose.yaw;
}

/**
 * @brief Generate the Q matrix for the controller.
*/
void ClassLQRController::generateQ(double q1, double q2, double q3)
{
    Q << q1, 0, 0,
            0, q2, 0,
            0, 0, q3;
}

/**
 * @brief Generate the R matrix for the controller.
*/
void ClassLQRController::generateR(double r1, double r2)
{
    R << r1, 0,
            0, r2;
}

/**
 * @brief Solve the LQR controller.
*/
void ClassLQRController::solve()
{
    generateA();
    generateB();
    solveRiccatiEquation();
    solveK();
    solveU();
}

/**
 * @brief Solve the Riccati equation.
*/
void ClassLQRController::solveRiccatiEquation()
{
    P = Q;
    for (int i = 0; i < mk_riccati_max_iter; i++)
    {
        Eigen::Matrix3d P_next = At * P * A - At * P * B * (R + Bt * P * B).inverse() * Bt * P * A + Q;
        if ((P - P_next).norm() < 1e-5)
        {
            P = P_next;
            break;
        }
        P = P_next;
    }
}

/**
 * @brief Solve the K matrix.
*/
void ClassLQRController::solveK()
{
    K = (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
}

/**
 * @brief Generate the A matrix.
*/
void ClassLQRController::generateA()
{
    A << 1, 0, -mk_dt * u_ref(0) * std::sin(s(2)),
            0, 1, mk_dt * u_ref(0) * std::cos(s(2)),
            0, 0, 1;
    At = A.transpose();
}

/**
 * @brief Generate the B matrix.
*/
void ClassLQRController::generateB()
{
    B << mk_dt * std::cos(s(2)), 
            0,
            mk_dt * std::sin(s(2)), 
            0,
            mk_dt * std::tan(u_ref(1)) / mk_axle_distance , 
            mk_dt * u_ref(0) / (mk_axle_distance * std::pow( std::cos( u_ref(1) ) ,2) );
    Bt = B.transpose();
}

/**
 * @brief Solve the control vector.
*/
void ClassLQRController::solveU()
{
    double x_err = s(0) - s_ref(0);
    double y_err = s(1) - s_ref(1);
    double yaw_err = normalizeAngle(s(2) - s_ref(2));
    Eigen::Vector3d s_err(x_err, y_err, yaw_err);

    u_err = -K * s_err;
    u_result = u_err + u_ref;
}

/**
 * @brief Normalize the angle to the range (-pi, pi]
*/
double ClassLQRController::normalizeAngle(double angle) 
{
    angle = fmod(angle, M_PI*2); // Ensure angle is within the range [0, 360)
    if (angle > M_PI) {
        angle -= M_PI*2; // Shift to the range (-180, 180]
    } else if (angle <= -M_PI) {
        angle += M_PI*2; // Shift to the range (-180, 180]
    }
    return angle;
}

/**
 * @brief Get the steering angle from the controller.
*/
double ClassLQRController::getSteer() 
{ 
    return normalizeAngle(u_result(1)); 
}

/**
 * @brief Get the speed from the controller.
*/
double ClassLQRController::getV() 
{ 
    return u_result(0);
}


} // namespace hawa