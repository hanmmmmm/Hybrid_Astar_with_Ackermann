#ifndef CLASS_PATH_SOLVER_H
#define CLASS_PATH_SOLVER_H

#include <iostream>
#include <assert.h>
#include <vector>
#include <algorithm>

#include <cppad/ipopt/solve.hpp>

#include "dynamics_model_formula.h"
#include "helper_state_indices.h"


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
        // assert(x.size() == 4);

        // decode the vector of states into variables

        ADvectorvector _samples;

        // initial state
        ADvector _temp(model_states_total_number);
        _temp[index_pose_x] = x[index_pose_x];
        _temp[index_pose_y] = x[index_pose_y];
        _temp[index_pose_yaw] = x[index_pose_yaw];
        _temp[index_linear_v] = x[index_linear_v];
        _temp[index_linear_a] = x[index_linear_a];
        _temp[index_aglr_str_angle] = x[index_aglr_str_angle];
        _temp[index_aglr_str_rate] = x[index_aglr_str_rate];

        _samples.push_back(_temp);

        int _num_future_samples = (x.size()- 3 - model_states_total_number) / 2;  // the times of state updating required

        for(int i = 0; i < _num_future_samples; i++)
        {
            _temp = update_robot_model( _samples[_samples.size()-1] , 1.0);
            _temp[index_linear_a] = x[model_states_total_number+i*2];
            _temp[index_aglr_str_rate] = x[model_states_total_number+i*2+1];
            _samples.push_back(_temp);
        }

        AD<double> j_all_acc = 0;
        for (int i=0; i<_num_future_samples; i++)
        {
            j_all_acc += CppAD::pow(_samples[i+1][index_linear_a], 2);
        }
        j_all_acc *= 1.0;


        AD<double> j_all_str_rate = 0;
        for (int i=0; i<_num_future_samples; i++)
        {
            j_all_str_rate += CppAD::pow(_samples[i+1][index_aglr_str_rate], 2);
        }
        j_all_str_rate *= 1.0;

        AD<double> j_last_acc = CppAD::abs(_samples[_samples.size()-1][index_linear_a]);
        j_last_acc *= 1.0;

        AD<double> j_last_v = CppAD::abs(_samples[_samples.size()-1][index_linear_v]);
        j_last_v *= 1.0;

        AD<double> _dx = m_goal_state_.x - _samples[_samples.size()-1][index_pose_x];
        AD<double> _dy = m_goal_state_.y - _samples[_samples.size()-1][index_pose_y];
        AD<double> j_offset_to_goal = CppAD::sqrt(_dx * _dx + _dy * _dy);
        

        // f(x) objective function
        fg[0] = j_all_acc + j_all_str_rate + j_last_acc + j_last_v + j_offset_to_goal;
        
        // // constraints
        // fg[1] = x1 * x2 * x3 * x4;
        // fg[2] = x1 * x1 + x2 * x2 + x3 * x3 + x4 * x4;
        
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
    
public:
    ClassPathSolver();
    ~ClassPathSolver();

    bool solve(
        StructModelStates init_state, 
        StructModelStates goal_state, 
        const int steps,
        const double sec
        );
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

    Dvector vars(n_vars);
    size_t i;
    for (i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }




}




#endif