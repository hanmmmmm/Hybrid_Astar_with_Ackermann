#ifndef CURVE_HELPER_FUNCTIONS_H
#define CURVE_HELPER_FUNCTIONS_H


#include <iostream>
#include <array>
#include <math.h>


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


inline double mod_angle_2pi(const double angle)
{
    return angle - M_PI*2 * floor( angle / (M_PI*2) );
}



double ceil_number_to_multiply_of_base(const double val_in, const double base, int& num_multi)
{
    int _multi = 0;
    while(val_in > (base*double(_multi)) )
    {
        _multi += 1;
    }
    num_multi = _multi;
    return base*double(_multi);
}

void ceil_number_to_multiply_of_base(const double val_in, const double base, int& num_multi, double& val_out)
{
    int _multi = 0;
    while(val_in > (base*double(_multi)) )
    {
        _multi += 1;
    }
    num_multi = _multi;
    val_out = base * double(_multi);
}


double linear_interpolate_angles(double angle_1, double angle_2, const double t)
{
    angle_1 = mod_angle_2pi(angle_1);
    angle_2 = mod_angle_2pi(angle_2);
    if (angle_2 > angle_1)
    {
        double _delta = angle_2 - angle_1;
        if (_delta >= M_PI)
            return angle_1 + t * _delta;
        else
            return angle_1 - t * (M_PI*2 - _delta);
    }
    else
    {
        double _delta = angle_1 - angle_2;
        if (_delta >= M_PI)
            return angle_1 - t * _delta;
        else
            return angle_1 + t * (M_PI*2 - _delta);
    }
}


void solve_quadratic(const double A, const double B, const double C, double& r1, double& r2)
{
    if (std::pow(B, 2) < 4 * A * C )
        std::cout << "Seems the quadratic solution need be complex." << std::endl;

    double sq = std::sqrt( std::pow(B, 2) - 4 * A * C );
    r1 = (-B + sq) / (A*2);
    r2 = (-B - sq) / (A*2);
}


void solve_quadratic_for_unique_postive_root(const double A, const double B, const double C, double& root)
{
    if (std::pow(B, 2) < 4 * A * C )
        std::cout << "Seems the quadratic solution need be complex." << std::endl;
    
    double sq = std::sqrt( std::pow(B, 2) - 4 * A * C );
    double r1 = (-B + sq) / (A*2);
    double r2 = (-B - sq) / (A*2);

    if (r1 > 0 && r2 <= 0)
    {
        root = r1;
    }
    else if (r1 <= 0 && r2 > 0)
    {
        root = r2;
    }
    else if (r1 < 0 && r2 < 0)
    {
        std::cout << "Didn't find postive roots." << std::endl;
    }
    else if (r1 > 0 && r2 > 0)
    {
        std::cout << "Found two postive roots." << std::endl;
    }
    else
    {
        std::cout << "Error in solve_quadratic_for_unique_postive_root." << std::endl;
    }
}


#endif