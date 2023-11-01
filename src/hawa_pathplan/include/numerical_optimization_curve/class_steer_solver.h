#ifndef CLASS_STEER_SOLVER_H
#define CLASS_STEER_SOLVER_H

#include <iostream>
#include <math.h>

class ClassSteerSolver
{
private:
    double m_p1x_, m_p1y_, m_p1t_;
    double m_p2x_, m_p2y_, m_p2t_;

    double m_axle_length_, m_steer_limit_;

    double m_speed_, m_time_sec_;

    bool m_verify_;

    double m_steer_angle_;

public:
    ClassSteerSolver(const double axle_length, const double steer_limit, const double speed, const double time);
    ~ClassSteerSolver();

    bool solve();

    void set_point_1(const double x, const double y, const double t);
    void set_point_2(const double x, const double y);

    double get_steer_angle();
    double get_point2_theta();
};

ClassSteerSolver::ClassSteerSolver(const double axle_length, const double steer_limit, const double speed, const double time)
{
    m_axle_length_ = std::abs(axle_length);
    m_steer_limit_ = steer_limit;
    m_speed_ = speed;
    m_time_sec_ = time;
    // m_verify_ = false;
    m_verify_ = true;
}

ClassSteerSolver::~ClassSteerSolver()
{
}


bool ClassSteerSolver::solve()
{
    // let p1 be the origin of the new coordinate 
    double _p2x_original = m_p2x_ - m_p1x_;
    double _p2y_original = m_p2y_ - m_p1y_;

    double _p2x = std::abs(_p2x_original);
    double _p2y = std::abs(_p2y_original);

    if (m_verify_)
    {
        std::cout << "p1: " << m_p1x_ << " " << m_p1y_ << " " << m_p1t_ << std::endl;
        std::cout << "p2: " << m_p2x_ << " " << m_p2y_ << std::endl;
        std::cout << "offsetted: " << _p2x << " " << _p2y << std::endl;
    }

    // rotate them by the value of the theta of point 1
    double _tempx = std::cos(m_p1t_) * _p2x + std::sin(m_p1t_) * _p2y;
    double _tempy = -std::sin(m_p1t_) * _p2x + std::cos(m_p1t_) * _p2y;

    _p2x = _tempx;
    _p2y = _tempy;

    if (m_verify_)
    {
        std::cout << "rotated: " << _p2x << " " << _p2y << std::endl;
    }

    // double _radius = std::sqrt(_p2x*_p2x + 2*_p2y*_p2y) - std::abs(_p2y);
    double _radius = (_p2x*_p2x + _p2y*_p2y) / (std::abs(_p2y) * 2.0);

    if (m_verify_)
    {
        std::cout << "R : " << _radius << std::endl;
    }

    if (m_verify_)
    {
        double _d_to_p2 = std::sqrt(std::pow(_p2x, 2) + std::pow((_radius - std::abs(_p2y)) , 2));
        double _difference = _d_to_p2 - _radius;
        if (std::abs(_difference) > 0.1)
        {
            std::cerr << "error in finding radius. " << _difference << std::endl;
            return false;
        }
    }

    double _steer = std::atan2(m_axle_length_ , _radius);

    // check the limit
    if (_steer > m_steer_limit_)
    {
        if (m_verify_)
        {
            std::cout << "steer larger than limit: " << _steer << " " << m_steer_limit_ << std::endl;
            _steer = m_steer_limit_;
        }
    }

    // check left or right 
    if (_p2y >= 0)
    {
        _steer = std::abs(_steer);
    }
    else
    {
        _steer = -std::abs(_steer);
    }

    if (m_verify_)
    {
        std::cout << "_steer : " << _steer << std::endl;
    }


    m_steer_angle_ = _steer;

    double _heading_change = m_time_sec_ * m_speed_ * std::tan(_steer) / m_axle_length_;

    m_p2t_ = m_p1t_ + _heading_change;

    if (m_verify_)
    {
        std::cout << "heading change : " << _heading_change << "  " << m_p2t_ << std::endl;
        std::cout << std::endl;
    }

    return true;
}


void ClassSteerSolver::set_point_1(const double x, const double y, const double t)
{
    m_p1x_ = x;
    m_p1y_ = y;
    m_p1t_ = t;
}


void ClassSteerSolver::set_point_2(const double x, const double y)
{
    m_p2x_ = x;
    m_p2y_ = y;
}


double ClassSteerSolver::get_steer_angle()
{
    return m_steer_angle_;
}


double ClassSteerSolver::get_point2_theta()
{
    return m_p2t_;
}


#endif