


#include <iostream>
#include <assert.h>
#include <vector>
#include <algorithm>

#include <cppad/ipopt/solve.hpp>

// #include "dynamics_model_formula.h"
// #include "helper_state_indices.h"
// #include "class_model_limits.h"


// both angle and length are changing.
// start from robot current location, instead of (0, 0)
// change steer variable from yaw_diff to steer_angle.
// change length variable from length to acceleration.


// the planning is not stable when using non-zero velocity and
// acceleration in starting condition; or the result is clearly not optimal.
// More work is needed to find out the reason.
// The zero velocity and acceleration starting condition works well.



const int NUM_SAMPLES = 20;  // the number of points used in the path projection.
const int NUM_CONTROL = 2;  // control variables are the steer_angle and linear acceleration.

const int INDEX_STEER_ANGLE_start = 0;
const int INDEX_ACC_start         = INDEX_STEER_ANGLE_start + NUM_SAMPLES;

const int INDEX_goal_x       = NUM_SAMPLES * NUM_CONTROL + 0;
const int INDEX_goal_y       = NUM_SAMPLES * NUM_CONTROL + 1;
const int INDEX_goal_yaw_sin = NUM_SAMPLES * NUM_CONTROL + 2;
const int INDEX_goal_yaw_cos = NUM_SAMPLES * NUM_CONTROL + 3;
const int INDEX_goal_v       = NUM_SAMPLES * NUM_CONTROL + 4;
const int INDEX_goal_a       = NUM_SAMPLES * NUM_CONTROL + 5;

const int INDEX_start_x      = NUM_SAMPLES * NUM_CONTROL + 6;
const int INDEX_start_y      = NUM_SAMPLES * NUM_CONTROL + 7;
const int INDEX_start_yaw    = NUM_SAMPLES * NUM_CONTROL + 8;
const int INDEX_start_v      = NUM_SAMPLES * NUM_CONTROL + 9;
const int INDEX_start_a      = NUM_SAMPLES * NUM_CONTROL + 10;

const double AKM_AXLE_DISTANCE_METER = 0.25;


namespace {
using CppAD::AD;
class FG_eval {
public:

    double m_goal_x_, m_goal_y_;
    FG_eval(double gx, double gy)
    {
        this->m_goal_x_ = gx;
        this->m_goal_y_ = gy;
    }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    typedef CPPAD_TESTVECTOR(ADvector) ADvectorvector;

    void operator()(ADvector& fg, const ADvector& x)
    {

        AD<double> _goalx = x[INDEX_goal_x];
        AD<double> _goaly = x[INDEX_goal_y];

        AD<double> _goal_yaw_sin = x[INDEX_goal_yaw_sin];
        AD<double> _goal_yaw_cos = x[INDEX_goal_yaw_cos];

        
        AD<double> _pose_x = x[INDEX_start_x];
        AD<double> _pose_y = x[INDEX_start_y];
        AD<double> _yaw = x[INDEX_start_yaw];

        AD<double> _velocity = 0.0; // x[INDEX_start_v];
        AD<double> _velocity_out_range = 0.0;
        

        for(size_t i=0; i<NUM_SAMPLES; i++)
        {
            // calc new velocity
            _velocity += x[INDEX_ACC_start + i];

            // calc robot heading change
            AD<double> _dyaw = _velocity * CppAD::tan(x[INDEX_STEER_ANGLE_start + i]) / AKM_AXLE_DISTANCE_METER;

            // pose changes in robot body frame
            AD<double> _dx_body = _velocity * CppAD::cos(_dyaw);
            AD<double> _dy_body = _velocity * CppAD::sin(_dyaw);
            
            // convert the body frame changes into global fixed frame
            AD<double> _dx_fix = _dx_body * CppAD::cos(_yaw) - _dy_body * CppAD::sin(_yaw);
            AD<double> _dy_fix = _dx_body * CppAD::sin(_yaw) + _dy_body * CppAD::cos(_yaw);

            // update the robot pose
            _pose_x += _dx_fix;
            _pose_y += _dy_fix;
            _yaw += _dyaw;

            // add penalty if velocity is higher than the limits.
            // I think higher velocity can be allowed in this planner because this 
            // high velocity just make the result path a bit longer but the motion
            // controller algorithm should be able to deal with this kind of trajectory.
            if (_velocity > 0.3)
            {
                _velocity_out_range += _velocity - 0.3;
            }
            else if (_velocity < -0.3)
            {
                _velocity_out_range += -0.3 - _velocity;
            }
        }

        // object cost to be minimized
        fg[0] = 0.0;
        
        // add the cost of the difference between the final robbot pose and goal pose.
        fg[0] +=  CppAD::pow(_pose_x - _goalx, 2.0);
        fg[0] +=  CppAD::pow(_pose_y - _goaly, 2.0);
        
        // the heading difference is not calculated by comparing the values in radian,
        // due to the fact that angles can be 2pi*k difference but represent the same angle.
        // So their sin and cos values are used for compareson.
        fg[0] +=  CppAD::pow(CppAD::sin(_yaw) - _goal_yaw_sin, 2.0);
        fg[0] +=  CppAD::pow(CppAD::cos(_yaw) - _goal_yaw_cos, 2.0);
        

        // TODD:
        // fix: once add this constraint, sometimes the solution will stuck at all 0.

        // we hope the final velocity and acceleration to be some specific values.
        fg[1] = 0.0;
        fg[1] += CppAD::pow(_velocity - x[INDEX_goal_v], 2.0) * 0.6;
        fg[1] += CppAD::pow(x[INDEX_ACC_start + NUM_SAMPLES-1] - x[INDEX_goal_a], 2.0) * 0.1;

        // we hope the velocity during the task do not exceed the limits.
        fg[2] = 0.0;
        fg[2] += _velocity_out_range;

        // we hope the steer angle do not change too fast.
        fg[3] = 0.0;
        for (int i=0; i<NUM_SAMPLES-1; i++)
        {
            fg[3] += CppAD::pow(x[INDEX_STEER_ANGLE_start+i] - x[INDEX_STEER_ANGLE_start+i+1], 2.0);
        }
        // fg[3] /= NUM_SAMPLES;
        
        
        std::cout << "fg[0]: " << fg[0] << std::endl;
        std::cout << "fg[1]: " << fg[1] << std::endl;
        std::cout << "fg[2]: " << fg[2] << std::endl;
        std::cout << "fg[3]: " << fg[3] << std::endl;

        return;
    }

};

}




class ClassPathSolver
{
private:

    double m_goal_x_, m_goal_y_, m_goal_yaw_;

    double m_start_x_, m_start_y_, m_start_yaw_;

    std::vector<std::vector<double>> m_result_path_;

    std::vector<std::vector<double>> m_initial_path_;

    std::vector<double> m_last_solution_;
    
public:
    ClassPathSolver();
    ~ClassPathSolver();

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    typedef CPPAD_TESTVECTOR(ADvector) ADvectorvector;

    bool solve( double gx, double gy, double gyaw, double gv, double ga, double sx, double sy, double syaw, double sv, double sa, bool reinit );

    void get_result_path(std::vector<std::vector<double>> & val);
    void get_initial_path(std::vector<std::vector<double>> & val);
};


ClassPathSolver::ClassPathSolver()
{
}

ClassPathSolver::~ClassPathSolver()
{
}



bool ClassPathSolver::solve( double gx, double gy, double gyaw, double gv, double ga, double sx, double sy, double syaw, double sv, double sa, bool reinit )
{
    m_goal_x_ = gx;
    m_goal_y_ = gy;
    m_goal_yaw_ = gyaw;

    m_start_x_ = sx;
    m_start_y_ = sy;
    m_start_yaw_ = syaw;


    typedef CPPAD_TESTVECTOR(double) Dvector;

    int _total_num_control_variables = NUM_SAMPLES * NUM_CONTROL + 6 + 5;

    Dvector _control_var(_total_num_control_variables);

    if (reinit)
    {
        std::cout << "reinit guess: " << std::endl;
        for (size_t i = 0; i < NUM_SAMPLES; i++) {
        _control_var[INDEX_STEER_ANGLE_start + i] = 0.0;
        _control_var[INDEX_ACC_start + i] = 0.01;
        }
    }
    else
    {
        std::cout << "not reinit guess: " << std::endl;
        for (size_t i = 0; i < _total_num_control_variables; i++) {
        _control_var[i] = m_last_solution_[i];
        }
    }

    // _control_var[INDEX_ACC_start] = sa;
    
    _control_var[INDEX_goal_x] = m_goal_x_;
    _control_var[INDEX_goal_y] = m_goal_y_;
    _control_var[INDEX_goal_yaw_sin] = std::sin(m_goal_yaw_);
    _control_var[INDEX_goal_yaw_cos] = std::cos(m_goal_yaw_);
    _control_var[INDEX_goal_v] = gv;
    _control_var[INDEX_goal_a] = ga;


    _control_var[INDEX_start_x] = m_start_x_;
    _control_var[INDEX_start_y] = m_start_y_;
    _control_var[INDEX_start_yaw] = m_start_yaw_;
    _control_var[INDEX_start_v] = sv;
    _control_var[INDEX_start_a] = sa;

    

    std::cout << "initial guess: " << _control_var << std::endl;

    Dvector  _control_var_lowerbound(_total_num_control_variables);
    Dvector  _control_var_upperbound(_total_num_control_variables);


    // Set lower and upper limits for variables.
    for (int i = INDEX_STEER_ANGLE_start; i < INDEX_STEER_ANGLE_start + NUM_SAMPLES; i++) 
    {
        _control_var_lowerbound[i] = -0.5;
        _control_var_upperbound[i] = 0.5;
    }
    for (int i = INDEX_ACC_start; i < INDEX_ACC_start + NUM_SAMPLES; i++) 
    {
        _control_var_lowerbound[i] = -0.1;
        _control_var_upperbound[i] = 0.1;
    }

    _control_var_lowerbound[INDEX_goal_x] = m_goal_x_;
    _control_var_upperbound[INDEX_goal_x] = m_goal_x_;

    _control_var_lowerbound[INDEX_goal_y] = m_goal_y_;
    _control_var_upperbound[INDEX_goal_y] = m_goal_y_;

    _control_var_lowerbound[INDEX_goal_yaw_sin] = _control_var[INDEX_goal_yaw_sin]-0.1;
    _control_var_upperbound[INDEX_goal_yaw_sin] = _control_var[INDEX_goal_yaw_sin]+0.1;
    
    _control_var_lowerbound[INDEX_goal_yaw_cos] = _control_var[INDEX_goal_yaw_cos]-0.1;
    _control_var_upperbound[INDEX_goal_yaw_cos] = _control_var[INDEX_goal_yaw_cos]+0.1;

    _control_var_lowerbound[INDEX_goal_v] = gv;
    _control_var_upperbound[INDEX_goal_v] = gv;

    _control_var_lowerbound[INDEX_goal_a] = ga;
    _control_var_upperbound[INDEX_goal_a] = ga;


    _control_var_lowerbound[INDEX_start_x] = m_start_x_;
    _control_var_upperbound[INDEX_start_x] = m_start_x_;

    _control_var_lowerbound[INDEX_start_y] = m_start_y_;
    _control_var_upperbound[INDEX_start_y] = m_start_y_;

    _control_var_lowerbound[INDEX_start_yaw] = m_start_yaw_;
    _control_var_upperbound[INDEX_start_yaw] = m_start_yaw_;

    _control_var_lowerbound[INDEX_start_v] = sv;
    _control_var_upperbound[INDEX_start_v] = sv;

    _control_var_lowerbound[INDEX_start_a] = sa;
    _control_var_upperbound[INDEX_start_a] = sa;

    std::cout << "_control_var_lowerbound: " << _control_var_lowerbound << std::endl;
    std::cout << "_control_var_upperbound: " << _control_var_upperbound << std::endl;


    int _num_contraints = 4;

    Dvector constraints_lowerbound(_num_contraints);
    Dvector constraints_upperbound(_num_contraints);
    constraints_lowerbound[0] = -0.0;
    constraints_upperbound[0] = 0.0;

    constraints_lowerbound[1] = -2.0;
    constraints_upperbound[1] = 2.0;

    constraints_lowerbound[2] = -1.0;
    constraints_upperbound[2] = 1.0;

    constraints_lowerbound[3] = -1.0;
    constraints_upperbound[3] = 1.0;

    std::cout << "constraints_lowerbound: " << constraints_lowerbound << std::endl;
    std::cout << "constraints_upperbound: " << constraints_upperbound << std::endl;



    FG_eval fg_eval( m_goal_x_, m_goal_y_);

    std::cout << "m_goal_state_ x : " << m_goal_x_ << std::endl;
    std::cout << "m_goal_state_ y : " << m_goal_y_ << std::endl;
    std::cout << "m_goal_state_ yaw : " << m_goal_yaw_ << std::endl;

    // options for IPOPT solver
    std::string options;
    options += "Integer print_level  2\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    // options += "Numeric max_cpu_time 0.5\n";

    options += "Numeric max_cpu_time 0.10\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    std::cout << "CppAD::ipopt::solve start" << std::endl;
    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, 
        _control_var, 
        _control_var_lowerbound, 
        _control_var_upperbound, 
        constraints_lowerbound,
        constraints_upperbound, 
        fg_eval, 
        solution
        );
    std::cout << "CppAD::ipopt::solve done" << std::endl;

    // Check some of the solution values
    bool ok = true;
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    if (ok)
    {
        std::cout << "IPOPT found solution." << std::endl;
    }
    else
    {
        std::cout << "IPOPT failed." << std::endl;
    }

    m_last_solution_.clear();

    std::cout << "solution:" << std::endl;
    for (int solct=0; solct<NUM_SAMPLES; solct++)
    {
        double steer = solution.x[INDEX_STEER_ANGLE_start + solct];
        double acc = solution.x[INDEX_ACC_start + solct];
        std::cout << steer << "  " << acc << std::endl;
    }
    for(auto sx:solution.x)
    {
        m_last_solution_.push_back(sx);
    }

    
    
    

    auto _computed_obj_value = solution.obj_value;
    std::cout << "_computed_obj_value: " << _computed_obj_value << std::endl;

    auto _computed_g = solution.g;
    std::cout << "_computed_g: " << _computed_g << std::endl;




    m_result_path_.clear();

    double _pose_x = m_start_x_;
    double _pose_y = m_start_y_;
    double _yaw = m_start_yaw_;


    double _velocity = 0.0;
    for(size_t i=0; i<NUM_SAMPLES; i++)
    {
        _velocity += solution.x[INDEX_ACC_start + i];
        double _dyaw = _velocity * std::tan(solution.x[INDEX_STEER_ANGLE_start + i]) / AKM_AXLE_DISTANCE_METER;
        double _dx_body = _velocity * std::cos(_dyaw);
        double _dy_body = _velocity * std::sin(_dyaw);
        
        double _dx_fix = _dx_body * std::cos(_yaw) - _dy_body * std::sin(_yaw);
        double _dy_fix = _dx_body * std::sin(_yaw) + _dy_body * std::cos(_yaw);
        _pose_x += _dx_fix;
        _pose_y += _dy_fix;
        _yaw += _dyaw;

        // std::cout << " result_path : " <<  _pose_x  << "  " << _pose_y << "  " << _yaw << std::endl;
        m_result_path_.push_back(std::vector<double>{_pose_x, _pose_y,_yaw  });
    }

    return true;

    


}


void ClassPathSolver::get_result_path(std::vector<std::vector<double>> & val)
{
    val = m_result_path_;
}


void ClassPathSolver::get_initial_path(std::vector<std::vector<double>> & val)
{
    val = m_initial_path_;
}












