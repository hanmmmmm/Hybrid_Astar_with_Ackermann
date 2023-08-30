#ifndef TOOLS_ANGLES
#define TOOLS_ANGLES


#include "tf/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/Quaternion.h"



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







#endif