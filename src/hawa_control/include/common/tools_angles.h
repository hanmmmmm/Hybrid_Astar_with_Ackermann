#ifndef TOOLS_ANGLES
#define TOOLS_ANGLES


#include "tf/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/Quaternion.h"

#include <array>


double mod_2pi( double a)
{
    double angle = a;
    while (angle > 2*M_PI){
        angle -= 2*M_PI;
    }
    while (angle < 0){
        angle += 2*M_PI;
    }
    return angle;
}



/// @brief compute the yaw angle of the line formed by 2 points
/// @param x1
/// @param y1
/// @param x2
/// @param y2
/// @return double , radian.
inline double compute_yaw_of_2_points(double x1, double y1, double x2, double y2)
{
    /*
        /x2
       /
      /
    x1-------
    */
    double dx = x2 - x1;
    double dy = y2 - y1;
    double yaw = atan2(dy, dx);
    return yaw;
}


bool check_if_point_on_left_of_line(double ax, double ay, double bx, double by, double cx, double cy ){
    return ((bx - ax)*(cy - ay) - (by - ay)*(cx - ax)) > 0;
}
        
double quaternion_to_eular_yaw(const double q_x, const double q_y, const double q_z, const double q_w)
{
    tf::Quaternion q( q_x, q_y, q_z, q_w );
    tfScalar yaw, pitch, roll;
    tf::Matrix3x3 mat(q);
    mat.getEulerYPR(yaw, pitch, roll);
    return yaw;
}

double quaternion_to_eular_yaw(const tf::Quaternion q_in)
{
    tfScalar yaw, pitch, roll;
    tf::Matrix3x3 mat(q_in);
    mat.getEulerYPR(yaw, pitch, roll);
    return yaw;
}


double quaternion_to_eular_yaw(const geometry_msgs::Quaternion q_in)
{
    tf::Quaternion q( q_in.x, q_in.y, q_in.z, q_in.w );
    tfScalar yaw, pitch, roll;
    tf::Matrix3x3 mat(q);
    mat.getEulerYPR(yaw, pitch, roll);
    return mod_2pi(yaw);
}




/// @brief 
/// @param p1 
/// @param p2 
/// @param p3 
/// @return the angle in radian. Range:[0, pi]
double calc_angle_by_three_points(std::array<double,3> p1, std::array<double,3> p2, std::array<double,3> p3)
{
    double _p1x = p1[0] - p2[0];
    double _p1y = p1[1] - p2[1];
    double _p2x = p3[0] - p2[0];
    double _p2y = p3[1] - p2[1];

    // get the unit length versions of those 2 vectors

    double _p1_length = std::sqrt(_p1x*_p1x + _p1y*_p1y);
    double _p2_length = std::sqrt(_p2x*_p2x + _p2y*_p2y);

    _p1x /= _p1_length;
    _p1y /= _p1_length;
    _p2x /= _p2_length;
    _p2y /= _p2_length;

    double _angle = std::acos(_p1x*_p2x + _p1y*_p2y);

    return _angle;

}



#endif