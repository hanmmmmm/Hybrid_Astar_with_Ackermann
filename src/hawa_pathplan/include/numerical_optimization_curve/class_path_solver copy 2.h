#ifndef CLASS_PATH_SOLVER_H
#define CLASS_PATH_SOLVER_H

#include <iostream>
#include <assert.h>
#include <vector>
#include <algorithm>

#include <cppad/ipopt/solve.hpp>

#include "dynamics_model_formula.h"
#include "helper_state_indices.h"
#include "class_model_limits.h"


namespace {
using CppAD::AD;
class FG_eval {
public:

    StructModelStates m_inital_state_, m_goal_state_;

    int m_num_steps_;

    double m_step_time_sec_;

    FG_eval(
        StructModelStates init_state, 
        StructModelStates goal_state, 
        const int steps,
        const double sec
        )
    {
        this->m_inital_state_ = init_state;
        this->m_goal_state_ = goal_state;
        this->m_num_steps_ = steps;
        this->m_step_time_sec_ = sec;
    }


    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    typedef CPPAD_TESTVECTOR(ADvector) ADvectorvector;


    void operator()(ADvector& fg, const ADvector& x)
    {
        // assert(fg.size() == 3);
        assert(x.size() == 2 * m_num_steps_);

        // decode the vector of states into variables

        ADvectorvector _samples;

        // initial state
        ADvector _temp(model_states_total_number);
        _temp[index_pose_x] = m_inital_state_.x;
        _temp[index_pose_y] = m_inital_state_.y;
        _temp[index_pose_yaw] = m_inital_state_.yaw;
        _temp[index_linear_v] = m_inital_state_.v;
        _temp[index_linear_a] = m_inital_state_.a;
        _temp[index_aglr_str_angle] = m_inital_state_.str_angl;
        _temp[index_aglr_str_rate] = m_inital_state_.str_rate;

        _samples.push_back(_temp);


        // project the future waypoints from the given control variables.

        for(int i = 0; i < m_num_steps_; i++)
        {
            _temp = update_robot_model( _samples[_samples.size()-1] , m_step_time_sec_);
            _temp[index_linear_a] = x[i*2];
            _temp[index_aglr_str_rate] = x[i*2+1];
            _samples.push_back(_temp);
        }

        std::cout << "_samples size: " << _samples.size() << std::endl;


        // // cost and constraints

        // AD<double> j_all_acc = 0;
        // for (int i=0; i<m_num_steps_; i++)
        // {
        //     j_all_acc += CppAD::pow(_samples[i+1][index_linear_a], 2);
        // }
        // j_all_acc *= 0.01;


        // AD<double> j_all_str_rate = 0;
        // for (int i=0; i<m_num_steps_; i++)
        // {
        //     j_all_str_rate += CppAD::pow(_samples[i+1][index_aglr_str_rate], 2);
        // }
        // j_all_str_rate *= 0.01;

        // AD<double> j_last_acc = CppAD::abs(_samples[_samples.size()-1][index_linear_a]);
        // j_last_acc *= 1.0;

        // AD<double> j_last_v = CppAD::abs(_samples[_samples.size()-1][index_linear_v]);
        // j_last_v *= 1.0;

        AD<double> _dx = m_goal_state_.x - _samples[_samples.size()-1][index_pose_x];
        AD<double> _dy = m_goal_state_.y - _samples[_samples.size()-1][index_pose_y];

        AD<double> _dyaw_sin = CppAD::sin(m_goal_state_.yaw)  - CppAD::sin(_samples[_samples.size()-1][index_pose_yaw]) ;
        AD<double> _dyaw_cos = CppAD::cos(m_goal_state_.yaw)  - CppAD::cos(_samples[_samples.size()-1][index_pose_yaw]) ;
        AD<double> _dyaw = CppAD::pow( _dyaw_sin, 2 ) + CppAD::pow( _dyaw_cos, 2 );
        // _dyaw *= 0.1;

        // // AD<double> j_offset_to_goal = CppAD::sqrt(_dx * _dx + _dy * _dy);
        // AD<double> j_offset_to_goal = CppAD::pow( _dx, 2 )+ CppAD::pow( _dy, 2 ) + _dyaw;
        // j_offset_to_goal *= 10.0;

        std::cout << "last point x: " << _samples[_samples.size()-1][index_pose_x] << std::endl;
        std::cout << "last point y: " << _samples[_samples.size()-1][index_pose_y] << std::endl;
        std::cout << "goal point x: " << m_goal_state_.x << std::endl;
        std::cout << "goal point y: " << m_goal_state_.y << std::endl;

        std::cout << "_dx: " << _dx << std::endl;
        std::cout << "_dy: " << _dy << std::endl;
        std::cout << "_dyaw: " << _dyaw << std::endl;
        // std::cout << "j_offset_to_goal: " << j_offset_to_goal << std::endl;
        

        // f(x) objective function
        // fg[0] =  j_offset_to_goal;
        
        fg[0] =  CppAD::pow( _dx, 2 )+ CppAD::pow( _dy, 2 ) + _dyaw * 0.5;

        std::cout << "fg[0]: " << fg[0] << std::endl;

        // // constraints
        // fg[1] = j_all_acc / m_num_steps_ + j_all_str_rate / m_num_steps_;
        // fg[2] = j_last_acc + j_last_v;
        
        return;
    }

};

}




class ClassPathSolver
{
private:

    StructModelStates m_inital_state_, m_goal_state_;

    int m_num_steps_;

    double m_step_time_sec_;

    std::vector<std::vector<double>> m_result_path_;

    std::vector<std::vector<double>> m_initial_path_;
    
public:
    ClassPathSolver();
    ~ClassPathSolver();

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    typedef CPPAD_TESTVECTOR(ADvector) ADvectorvector;

    bool solve(
        StructModelStates init_state, 
        StructModelStates goal_state, 
        const int steps,
        const double sec
        );

    void get_result_path(std::vector<std::vector<double>> & val);
    void get_initial_path(std::vector<std::vector<double>> & val);
};


ClassPathSolver::ClassPathSolver()
{
}

ClassPathSolver::~ClassPathSolver()
{
}



bool ClassPathSolver::solve(
        StructModelStates init_state, 
        StructModelStates goal_state, 
        const int steps,
        const double sec
        )
{
    m_inital_state_ = init_state;
    m_goal_state_ = goal_state;
    m_num_steps_ = steps;
    m_step_time_sec_ = sec;

    typedef CPPAD_TESTVECTOR(double) Dvector;

    int _total_num_control_variables = m_num_steps_ * 2;

    Dvector _control_var(_total_num_control_variables);
    size_t i;
    // for (i = 0; i < _total_num_control_variables; i++) {
    //     _control_var[i] = 0;
    // }
    for (i = 0; i < m_num_steps_; i++) {
        _control_var[i*2] = 0.1;
        _control_var[i*2+1] = 0.0;
        // if(i%2 == 0)
        // {
        //     // _control_var[i*2] = 0.05;
        //     _control_var[i*2+1] *= -1.0;
        // }
    }

    // _control_var[1] = -0.3;
    // _control_var[3] = -0.1;
    // _control_var[5] = -0.0;
    // _control_var[7] = 0.1;
    // _control_var[9] = 0.2;
    // _control_var[11] = 0.2;
    // _control_var[13] = 0.0;


    // _control_var[14] = -0.1;
    // _control_var[18] = -0.2;
    // _control_var[22] = -0.2;
    // _control_var[26] = -0.1;
    // _control_var[30] = -0.1;
    // _control_var[34] = -0.1;
    // _control_var[24] = -0.1;

    std::cout << "initial guess: " << _control_var << std::endl;

    Dvector  _control_var_lowerbound(_total_num_control_variables);
    Dvector  _control_var_upperbound(_total_num_control_variables);


    ADvectorvector _samples;

    // initial state
    ADvector _temp(model_states_total_number);
    _temp[index_pose_x] = m_inital_state_.x;
    _temp[index_pose_y] = m_inital_state_.y;
    _temp[index_pose_yaw] = m_inital_state_.yaw;
    _temp[index_linear_v] = m_inital_state_.v;
    _temp[index_linear_a] = m_inital_state_.a;
    _temp[index_aglr_str_angle] = m_inital_state_.str_angl;
    _temp[index_aglr_str_rate] = m_inital_state_.str_rate;

    _samples.push_back(_temp);

    m_initial_path_.clear();

    m_initial_path_.push_back(std::vector<double>{m_inital_state_.x, m_inital_state_.y});


    // project the future waypoints from the given control variables.

    for(int i = 0; i < m_num_steps_; i++)
    {
        _temp = update_robot_model( _samples[_samples.size()-1] , m_step_time_sec_);
        _temp[index_linear_a] = _control_var[i*2];
        _temp[index_aglr_str_rate] = _control_var[i*2+1];
        _samples.push_back(_temp);
        m_initial_path_.push_back(std::vector<double>{CppAD::Value(CppAD::Var2Par(_temp[index_pose_x])) , CppAD::Value(CppAD::Var2Par(_temp[index_pose_y])) });
    }




    // Set lower and upper limits for variables.
    for (i = 0; i < m_num_steps_; i++) {
        _control_var_lowerbound[i*2] = CppAD::Value(CppAD::Var2Par(acc_negative_limit_));
        _control_var_lowerbound[i*2+1] = CppAD::Value(CppAD::Var2Par(steer_rate_negative_limit_));
        _control_var_upperbound[i*2] = CppAD::Value(CppAD::Var2Par(acc_positive_limit_));
        _control_var_upperbound[i*2+1] = CppAD::Value(CppAD::Var2Par(steer_rate_positive_limit_));
    }

    std::cout << "_control_var_lowerbound: " << _control_var_lowerbound << std::endl;
    std::cout << "_control_var_upperbound: " << _control_var_upperbound << std::endl;

    int _num_contraints = 1;

    Dvector constraints_lowerbound(_num_contraints);
    Dvector constraints_upperbound(_num_contraints);
    constraints_lowerbound[0] = -0.0;
    constraints_upperbound[0] = 0.0;
    // constraints_lowerbound[1] = -0.1;
    // constraints_upperbound[1] = 0.1;
    // constraints_lowerbound[2] = -9999999.2;
    // constraints_upperbound[2] = 9999999.2;


    std::cout << "constraints_lowerbound: " << constraints_lowerbound << std::endl;
    std::cout << "constraints_upperbound: " << constraints_upperbound << std::endl;

    FG_eval fg_eval(m_inital_state_, m_goal_state_, m_num_steps_, m_step_time_sec_);

    std::cout << "m_inital_state_ x : " << m_inital_state_.x << std::endl;
    std::cout << "m_inital_state_ y : " << m_inital_state_.y << std::endl;
    std::cout << "m_inital_state_ yaw : " << m_inital_state_.yaw << std::endl;
    std::cout << "m_inital_state_ v : " << m_inital_state_.v << std::endl;
    std::cout << "m_inital_state_ a : " << m_inital_state_.a << std::endl;
    std::cout << "m_inital_state_ str_angl : " << m_inital_state_.str_angl << std::endl;
    std::cout << "m_inital_state_ str_rate : " << m_inital_state_.str_rate << std::endl;

    std::cout << "m_goal_state_ x : " << m_goal_state_.x << std::endl;
    std::cout << "m_goal_state_ y : " << m_goal_state_.y << std::endl;
    std::cout << "m_goal_state_ yaw : " << m_goal_state_.yaw << std::endl;
    std::cout << "m_goal_state_ v : " << m_goal_state_.v << std::endl;
    std::cout << "m_goal_state_ a : " << m_goal_state_.a << std::endl;
    std::cout << "m_goal_state_ str_angl : " << m_goal_state_.str_angl << std::endl;
    std::cout << "m_goal_state_ str_rate : " << m_goal_state_.str_rate << std::endl;


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

    options += "Numeric max_cpu_time 1.5\n";

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

    auto _computed_control_variables = solution.x;

    std::cout << "solution: " << _computed_control_variables << std::endl;

    auto _computed_obj_value = solution.obj_value;
    std::cout << "_computed_obj_value: " << _computed_obj_value << std::endl;

    auto _computed_g = solution.g;
    std::cout << "_computed_g: " << _computed_g << std::endl;


    m_result_path_.clear();

    // ADvectorvector _samples;
    _samples.clear();

    // initial state
    // ADvector _temp(model_states_total_number);
    _temp[index_pose_x] = m_inital_state_.x;
    _temp[index_pose_y] = m_inital_state_.y;
    _temp[index_pose_yaw] = m_inital_state_.yaw;
    _temp[index_linear_v] = m_inital_state_.v;
    _temp[index_linear_a] = m_inital_state_.a;
    _temp[index_aglr_str_angle] = m_inital_state_.str_angl;
    _temp[index_aglr_str_rate] = m_inital_state_.str_rate;

    _samples.push_back(_temp);

    m_result_path_.push_back(std::vector<double>{m_inital_state_.x, m_inital_state_.y});


    // project the future waypoints from the given control variables.

    for(int i = 0; i < m_num_steps_; i++)
    {
        _temp = update_robot_model( _samples[_samples.size()-1] , m_step_time_sec_);
        _temp[index_linear_a] = _computed_control_variables[i*2];
        _temp[index_aglr_str_rate] = _computed_control_variables[i*2+1];
        _samples.push_back(_temp);
        m_result_path_.push_back(std::vector<double>{CppAD::Value(CppAD::Var2Par(_temp[index_pose_x])) , CppAD::Value(CppAD::Var2Par(_temp[index_pose_y])) });
    }

    std::cout << "final x : " << _samples[_samples.size()-1][index_pose_x] << std::endl;
    std::cout << "final y : " << _samples[_samples.size()-1][index_pose_y] << std::endl;
    std::cout << "final yaw : " << _samples[_samples.size()-1][index_pose_yaw] << std::endl;
    std::cout << "final v : " << _samples[_samples.size()-1][index_linear_v] << std::endl;
    std::cout << "final a : " << _samples[_samples.size()-1][index_linear_a] << std::endl;
    std::cout << "final str_angl : " << _samples[_samples.size()-1][index_aglr_str_angle] << std::endl;
    std::cout << "final str_rate : " << _samples[_samples.size()-1][index_aglr_str_rate] << std::endl;

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


#endif