
#ifndef HAWA_CONTROL_INCLUDE_LQR_CLASS_LQR_CONTROLLER_H_
#define HAWA_CONTROL_INCLUDE_LQR_CLASS_LQR_CONTROLLER_H_

#include <Eigen/Dense>
#include <Eigen/Core>

#include <iostream> 
#include <math.h>

#include "common/class_elemental_pose2d.h"

/**
 * @brief Class for LQR controller implementation
*/
class ClassLQRController
{
private:

    Eigen::Matrix3d A;
    Eigen::Matrix<double, 3, 2> B;
    Eigen::Matrix3d Q;
    Eigen::Matrix2d R;
    Eigen::Matrix<double, 2, 3> K;
    Eigen::Matrix3d P;

    Eigen::Vector3d s, s_ref;
    Eigen::Vector2d u, u_ref, u_result, u_err;

    double m_dt = 0.1; // time step in seconds
    double m_axle_distance = 0.25; // distance between the front wheels and the rear wheels, in meters

private:

    

    void solveRiccatiEquation()
    {
        P = Q;
        for (int i = 0; i < 500; i++)
        {
            Eigen::Matrix3d P_next = A.transpose() * P * A - A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A + Q;
            if ((P - P_next).norm() < 1e-5)
            {
                P = P_next;
                break;
            }
            P = P_next;
        }
    }

    void solveK()
    {
        K = (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
    }

    void generateA()
    {
        A << 1, 0, -m_dt * u_ref(0) * std::sin(s(2)),
             0, 1, m_dt * u_ref(0) * std::cos(s(2)),
             0, 0, 1;
    }

    void generateB()
    {
        B << m_dt * std::cos(s(2)), 
             0,
             m_dt * std::sin(s(2)), 
             0,
             m_dt * std::tan(u_ref(1)) / m_axle_distance , 
             m_dt * u_ref(0) / (m_axle_distance * std::pow( std::cos( u_ref(1) ) ,2) );
    }

    void solveU()
    {
        double x_err = s(0) - s_ref(0);
        double y_err = s(1) - s_ref(1);
        double yaw_err = normalizeAngle(s(2) - s_ref(2));
        Eigen::Vector3d s_err(x_err, y_err, yaw_err);

        u_err = -K * s_err;
        u_result = u_err + u_ref;

        // std::cout << "u_err:   " << u_err.transpose() << std::endl;
        // std::cout << "u_ref:   " << u_ref.transpose() << std::endl;
        // std::cout << "u_result:   " << u_result.transpose() << std::endl;
    }

    double normalizeAngle(double angle) {
        angle = fmod(angle, M_PI*2); // Ensure angle is within the range [0, 360)
        if (angle > M_PI) {
            angle -= M_PI*2; // Shift to the range (-180, 180]
        } else if (angle <= -M_PI) {
            angle += M_PI*2; // Shift to the range (-180, 180]
        }
        return angle;
    }


public:
    ClassLQRController(const double l, const double dt) :m_axle_distance(l), m_dt(dt) {}
    ~ClassLQRController() {}

    void setVref(const double v_ref) 
    { u_ref(0) = v_ref; }
    
    void setSteerRef(const double steer_ref) 
    { u_ref(1) = steer_ref; }
    
    void setStateRef(ClassPose2D ref_pose)
    {
        s_ref(0) = ref_pose.x;
        s_ref(1) = ref_pose.y;
        s_ref(2) = ref_pose.yaw;
    }

    void setState(const ClassPose2D& pose)
    {
        s(0) = pose.x;
        s(1) = pose.y;
        s(2) = pose.yaw;
    }
    
    void generateQ(double q1, double q2, double q3)
    {
        Q << q1, 0, 0,
             0, q2, 0,
             0, 0, q3;
    }

    void generateR(double r1, double r2)
    {
        R << r1, 0,
             0, r2;
    }

    void solve()
    {
        generateA();
        generateB();
        solveRiccatiEquation();
        solveK();
        solveU();
    }

    double getSteer() { return normalizeAngle(u_result(1)); }
    double getV() { return u_result(0); }

    // /**
    //  * DISCARDED. No longer needed. But keep for future reference.
    //  * 
    //  * @brief Reflect a reference pose with respect to the robot pose. The symetric axle is 
    //  * the y-axis of robot body frame.
    // */
    // ClassPose2D reflectRefPose(const ClassPose2D& ref_pose, const ClassPose2D& robot_pose)
    // {
    //     SE2Pose robot(robot_pose.x, robot_pose.y, robot_pose.yaw);
    //     SE2Pose ref(ref_pose.x, ref_pose.y, ref_pose.yaw);
    //     Eigen::Matrix3d robot_inv = robot.matrix.inverse();
    //     Eigen::Matrix3d ref_in_robot = robot_inv * ref.matrix;
    //     Eigen::Matrix3d new_ref;
    //     new_ref << ref_in_robot(0, 0), ref_in_robot(0, 1), -ref_in_robot(0, 2),
    //                ref_in_robot(1, 0), ref_in_robot(1, 1), ref_in_robot(1, 2),
    //                0, 0, 1;
    //     new_ref = robot.matrix * new_ref;
    //     double new_x = new_ref(0, 2);
    //     double new_y = new_ref(1, 2);
    //     double new_yaw = std::atan2(new_ref(1, 0), new_ref(0, 0));
    //     return ClassPose2D(new_x, new_y, new_yaw);
    // }

    // // DISCARDED. No longer needed.  But keep for future reference.
    // // 
    // struct SE2Pose {
    //     Eigen::Matrix3d matrix;
    //     SE2Pose(double x, double y, double theta) {
    //         matrix << cos(theta), -sin(theta), x,
    //                 sin(theta),  cos(theta), y,
    //                         0,           0, 1;
    //     }
    // };

};





#endif