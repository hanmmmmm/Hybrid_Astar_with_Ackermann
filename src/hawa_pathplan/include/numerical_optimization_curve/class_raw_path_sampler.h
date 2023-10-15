#ifndef CLASS_RAW_PATH_SAMPLER_H
#define CLASS_RAW_PATH_SAMPLER_H


// Work Flow:
//
// receive path input
// check how many points in it
// decide how many points to sample
// decide sample every n points
// start sample
// use robot pose as the first sample
// loop through all points, but start from 2nd point in the path.
// if point is not a stop (sudden direction change)
//   if index is multiple of n,
//     then add this point in result
//   otherwise, skip this point
// if point is stop, then add it into result; and put velocity as 0.
// when reaching the end of path, donot append the last point in path,
// instead, using goal pose as the last point in result.
// return result.

#include <iostream>
#include <vector>
#include "nav_msgs/Path.h"
#include <math.h>

#include "../car_pathplan/struct_simple_pose.h"
#include "../car_pathplan/class_custom_path.h"
#include "../planner/conversion_tools.h"
#include "class_steer_solver.h"
#include "helper_functions.h"


// TODO:
// find out how to interpolate angles.
// estimate steering for each sampled point.
// make container for acceleration and steer. 


// using std::cout;
// using std::endl;
// using std::vector;


struct StructMotionSequence
{
    bool is_forward = true;

    double dist_accel = 0;
    double dist_const = 0;
    double dist_decel = 0;
    double dist_total = 0;

    double time_accel = 0;
    double time_const = 0;
    double time_decel = 0;
    double time_total = 0;

    double limit_velocity = 0;

    double limit_accel = 0;
    double limit_decel = 0;

    double actual_accel = 0;
    double actual_decel = 0;

    double actual_top_velocity = 0;

    double time_step_sec = 0;

    int steps_in_accel = 0;  // the amount of steps needed during the stage of increasing speed
    int steps_in_const = 0;  // the amount of steps needed during the stage of constant speed
    int steps_in_decel = 0;  // the amount of steps needed during the stage of decreasing speed

    void set_total_distance(double val);
    void set_time_step_sec(double val);
    void set_forwarding();
    void set_reversing();
    void set_limit_accel(double val);
    void set_limit_decel(double val);
    void set_limit_velocity(double val);

    bool verify_signs();
    void print_parameters();
    
    double calc_distance_by_time(const double dt);
    double calc_speed_by_time(const double dt);
    void calc_speed_from_distances(const std::vector<double>& dists, std::vector<double>& spds);

    void calc_key_distances_by_timestep(vector<double>& distances, vector<double>& speeds);
};

void StructMotionSequence::set_total_distance(double val)
{
    if (val <= 0)
        std::cerr << "set_total_distance() got negative value: " << val << std::endl;

    this->dist_total = val;
}

void StructMotionSequence::set_time_step_sec(double val)
{
    if (val <= 0)
        std::cerr << "set_time_step_sec() got negative value: " << val << std::endl;

    this->time_step_sec = val;
}


void StructMotionSequence::set_limit_accel(double val)
{
    if (val <= 0)
        std::cerr << "set_limit_accel() got negative value: " << val << std::endl;

    this->limit_accel = val;
}

void StructMotionSequence::set_limit_decel(double val)
{
    if (val >= 0)
        std::cerr << "set_limit_decel() got positive value: " << val << std::endl;

    this->limit_decel = val;
}


void StructMotionSequence::set_limit_velocity(double val)
{
    if (val <= 0)
        std::cerr << "set_limit_velocity() got negative value: " << val << std::endl;

    this->limit_velocity = val;
}


void StructMotionSequence::set_forwarding()
{
    this->is_forward = true;
}


void StructMotionSequence::set_reversing()
{
    this->is_forward = false;
}


bool StructMotionSequence::verify_signs()
{
    if (this->is_forward)
    {
        if (! (this->limit_accel > 0))
        {
            return false;
        }
        if (! (this->limit_decel < 0))
        {
            return false;
        }
    }
    else
    {
        if (! (this->limit_accel > 0))
        {
            return false;
        }
        if (! (this->limit_decel < 0))
        {
            return false;
        }
    }
}


/// @brief 
/// @param dt: time in seconds since the moment when velocity changes from 0.
/// @return the distance from where the velocity changes from 0. 
double StructMotionSequence::calc_distance_by_time(const double dt)
{
    if (dt < time_accel)
        return 0.5 * actual_accel * dt * dt;
    else if (dt == time_accel)
        return dist_accel;
    else if (time_accel < dt && dt < (time_accel+time_const))
        return dist_accel + (dt-time_accel)*dist_const/time_const;
    else if (dt == time_accel + time_const)
        return dist_accel + dist_const;
    else if ( (dt > time_accel + time_const) && (dt < time_accel + time_const + time_decel))
    {
        double _t_in_down = dt - time_accel - time_const;
        double _d_in_down = actual_top_velocity * _t_in_down + 0.5 * actual_decel * _t_in_down * _t_in_down;
        return dist_accel + dist_const + _d_in_down;
    }
    else if (dt >= time_accel + time_const + time_decel)
        return dist_total;
}


double StructMotionSequence::calc_speed_by_time(const double dt)
{
    if (dt <= time_accel)
    {
        return actual_accel * (dt+0);
    }
    else if (time_accel < dt && dt < (time_accel+time_const)*0.99)
    {
        return actual_top_velocity;
    }
    else
    {
        double _dt_in_decel = dt - time_accel - time_const ;
        return actual_top_velocity + actual_decel * _dt_in_decel;
    }   
}

void StructMotionSequence::calc_speed_from_distances(const std::vector<double>& dists, std::vector<double>& spds)
{
    spds.clear();

    // std::cout << "speeds : ";

    for (int ct=1; ct<dists.size(); ct++)
    {
        double _displacement = dists.at(ct) - dists.at(ct-1);
        spds.push_back(_displacement / time_step_sec);
        // std::cout << " " << (_displacement / time_step_sec);
    }
    // std::cout << std::endl;
    
    // the last point needs a bit special calc
    double _displacement = dist_total - dists.at(dists.size()-1);
    spds.push_back(_displacement / time_step_sec);
    // std::cout << "the last displacement  " << _displacement << "  the last speed  " << (_displacement / time_step_sec) << std::endl;
    
}

void StructMotionSequence::calc_key_distances_by_timestep(vector<double>& distances, vector<double>& speeds)
{
    distances.clear();
    speeds.clear();

    double _total_dt = 0.0;
    double _key_distance = 0.0;
    double _speed = 0.0;

    while (_total_dt <= this->time_total)
    {
        _key_distance = calc_distance_by_time( _total_dt);
        distances.push_back(_key_distance);
        
        // _speed = calc_speed_by_time( _total_dt);
        // speeds.push_back(_speed);

        // cout << "calc_key_distances_by_timestep _total_dt  " << _total_dt  << "  dist  " << _key_distance << endl;
        // cout << _total_dt  << "," << _key_distance << endl;

        _total_dt += this->time_step_sec;
    }
    if (distances.size())
    {
        // slightly shorten the last section, otherwise some float precision issue happens occasionally
        distances[distances.size()-1] *= 0.999;  
        calc_speed_from_distances(distances, speeds);
    }


    // std::cout << "   D     V" << std::endl; 
    // std::cout << "#" << 0 << " " << distances[0] << "  " << speeds[0] << std::endl;
    // for (int ct=1; ct<distances.size(); ct++)
    // {
    //     std::cout << "#" << ct << " " << distances.at(ct) << "  " << distances[ct]-distances[ct-1] << "  " << speeds[ct] << std::endl;
    // }
}


void StructMotionSequence::print_parameters()
{
    cout << "_motion_seq time_accel   " << time_accel << endl;
    cout << "_motion_seq time_const " << time_const << endl;
    cout << "_motion_seq time_decel " << time_decel << endl;
    
    cout << "_motion_seq dist_accel   " << dist_accel << endl;
    cout << "_motion_seq dist_const " << dist_const << endl;
    cout << "_motion_seq dist_decel " << dist_decel << endl;

    cout << "_motion_seq limit_accel " << limit_accel << endl;
    cout << "_motion_seq actual_accel " << actual_accel << endl;
    cout << "_motion_seq limit_decel " << limit_decel << endl;
    cout << "_motion_seq actual_decel " << actual_decel << endl;
    cout << "_motion_seq limit_velocity " << limit_velocity << endl;
    cout << "_motion_seq actual_top_velocity " << actual_top_velocity << endl;

    cout << "_motion_seq time_total " << time_total << endl;
    cout << "_motion_seq dist_total " << dist_total << endl;
}


/// @brief This fuction is to find the time duration for each of these 2 conditions.
///  
///   ^     _______                     ^     
/// v |    /       \                  v |    
///   |   /         \                   |   /\
///   |  /           \                  |  /  \
///   +-.--.-------.--.--> t            +---------> t
///       t1   t2  t3                     t1 t2 t3
///
/// @param motion_seq 
void calc_motion_sequence_parameters(StructMotionSequence& motion_seq)
{
    // first using the extreme values to estimate the times and distances in 3 stages. 
    double _guess_acc_time = std::abs(motion_seq.limit_velocity / motion_seq.limit_accel);
    double _guess_dec_time = std::abs(motion_seq.limit_velocity / motion_seq.limit_decel);
    
    double _guess_acc_distance = 0.5 * motion_seq.limit_accel * std::pow(_guess_acc_time, 2);
    double _guess_dec_distance = 0.5 * motion_seq.limit_decel * std::pow(_guess_dec_time, 2) + motion_seq.limit_velocity * _guess_dec_time;

    double _guess_const_distance = motion_seq.dist_total - (_guess_acc_distance + _guess_dec_distance);
    double _guess_cnst_time = _guess_const_distance / motion_seq.limit_velocity;

    double _step_size = motion_seq.time_step_sec;

    if (_guess_const_distance > 0)
    {
        // this is the first case mentioned above. The total distance is long enough so the robot can 
        // accelerates to the max speed then slow dowm.

        // calc the actual time length for each of the 3 stages. The values should be
        // 1. larger than the guessed value, so the actual acc/dec will not exceed their limits
        // 2. exact interger multiply of the time_step length, so the sampled points are evenly along the path. 

        double _actual_cnst_time, _actual_acc_time, _actual_dec_time;

        ceil_number_to_multiply_of_base(_guess_cnst_time, _step_size, motion_seq.steps_in_const, _actual_cnst_time);

        ceil_number_to_multiply_of_base(_guess_acc_time, _step_size, motion_seq.steps_in_accel, _actual_acc_time);

        ceil_number_to_multiply_of_base(_guess_dec_time, _step_size, motion_seq.steps_in_decel, _actual_dec_time);

        // calc the actual acceleration and deceleration. 
        // d = (1/2) * (a * t^2)

        double t1 = _actual_acc_time;
        double t2 = _actual_cnst_time;
        double t3 = _actual_dec_time;

        double _temp = 0.5 * t1 * t1 + t1 * t2 + 0.5 * t1 * t3; 
        motion_seq.actual_accel = motion_seq.dist_total / _temp;
        motion_seq.actual_decel = -(motion_seq.actual_accel * t1) / t3;
        
        motion_seq.time_accel = _actual_acc_time;
        motion_seq.time_const = _actual_cnst_time;
        motion_seq.time_decel = _actual_dec_time;
        
        motion_seq.dist_accel = 0.5 * motion_seq.actual_accel * motion_seq.time_accel * motion_seq.time_accel;
        motion_seq.actual_top_velocity = motion_seq.actual_accel * motion_seq.time_accel;

        motion_seq.dist_const = motion_seq.actual_top_velocity * motion_seq.time_const;

        motion_seq.dist_decel = motion_seq.actual_top_velocity * motion_seq.time_decel + 0.5 * motion_seq.actual_decel * motion_seq.time_decel * motion_seq.time_decel;

        motion_seq.time_total = 0;
        motion_seq.time_total += motion_seq.time_accel;
        motion_seq.time_total += motion_seq.time_const;
        motion_seq.time_total += motion_seq.time_decel;
    }
    else
    {
        double _actual_acc_time, _actual_dec_time;

        ceil_number_to_multiply_of_base(_guess_acc_time, _step_size, motion_seq.steps_in_accel, _actual_acc_time);

        ceil_number_to_multiply_of_base(_guess_dec_time, _step_size, motion_seq.steps_in_decel, _actual_dec_time);

        double t1 = _actual_acc_time;
        double t3 = _actual_dec_time;

        double _temp = 0.5 * t1 * t1 + 0.5 * t1 * t3; 
        motion_seq.actual_accel = motion_seq.dist_total / _temp;
        motion_seq.actual_decel = -(motion_seq.actual_accel * t1) / t3;


        motion_seq.time_accel = _actual_acc_time;
        motion_seq.time_const = 0.0;
        motion_seq.time_decel = _actual_dec_time;
        
        motion_seq.dist_accel = 0.5 * motion_seq.actual_accel * motion_seq.time_accel * motion_seq.time_accel;
        motion_seq.actual_top_velocity = motion_seq.actual_accel * motion_seq.time_accel;

        motion_seq.dist_const = 0;

        motion_seq.dist_decel = motion_seq.actual_top_velocity * motion_seq.time_decel + 0.5 * motion_seq.actual_decel * motion_seq.time_decel * motion_seq.time_decel;

        motion_seq.time_total = 0;
        motion_seq.time_total += motion_seq.time_accel;
        motion_seq.time_total += motion_seq.time_decel;
    }

}


void verify_motion_sequence_parameters(const StructMotionSequence& motion_seq)
{
    using std::cout;
    using std::endl;

    cout << "verify_motion_sequence_parameters" << endl;

    // time
    cout << "Time  ";
    cout << motion_seq.time_accel << " + " << motion_seq.time_const << " + " << motion_seq.time_decel << " = " ;
    cout << motion_seq.time_accel + motion_seq.time_const + motion_seq.time_decel << " : " << motion_seq.time_total << endl;

    // dist
    cout << "dist  ";
    cout << motion_seq.dist_accel << " + " << motion_seq.dist_const << " + " << motion_seq.dist_decel << " = " ;
    cout << motion_seq.dist_accel + motion_seq.dist_const + motion_seq.dist_decel << " : " << motion_seq.dist_total << endl;

    // speed
    cout << "speed acc  ";
    cout << motion_seq.actual_accel << " * " << motion_seq.time_accel << " = " << motion_seq.actual_accel * motion_seq.time_accel;
    cout << " : " << motion_seq.actual_top_velocity << endl;
    cout << "speed dec  ";
    cout << motion_seq.actual_decel << " * " << motion_seq.time_decel << " = " << motion_seq.actual_decel * motion_seq.time_decel;
    cout << " : " << motion_seq.actual_top_velocity << endl;

    // dist by v t
    cout << "dist acc from topv and time  ";
    double _dist_by_v = 0.5 * motion_seq.actual_top_velocity * motion_seq.time_accel;
    cout << _dist_by_v << " : " << motion_seq.dist_accel << endl;
    cout << "dist dec from topv and time  ";
    _dist_by_v = 0.5 * motion_seq.actual_top_velocity * motion_seq.time_decel;
    cout << _dist_by_v << " : " << motion_seq.dist_decel << endl;
    

    // dist by acc time
    cout << "dist from acc and time  ";
    double _dist_by_acc = 0.5 * motion_seq.actual_accel * motion_seq.time_accel * motion_seq.time_accel;
    cout << _dist_by_acc << " : " << motion_seq.dist_accel << endl;
    cout << "dist from const and time  ";
    double _dist_by_const = motion_seq.actual_top_velocity * motion_seq.time_const;
    cout << _dist_by_const << " : " << motion_seq.dist_const << endl;
    cout << "dist from dec and time  ";
    double _dist_by_dec = motion_seq.actual_top_velocity * motion_seq.time_decel + 0.5 * motion_seq.actual_decel * motion_seq.time_decel * motion_seq.time_decel;
    cout << _dist_by_dec << " : " << motion_seq.dist_decel << endl;

}


vector<StructPoseReal> sample_from_one_segment_by_distance(const vector<double> distances, const vector<StructPoseReal>& original_path )
{
    vector<StructPoseReal> _result;
    vector<double> _original_path_step_lengths;
    for(int i=1; i<original_path.size(); i++)
    {
        double _dx = original_path[i-1].x - original_path[i].x;
        double _dy = original_path[i-1].y - original_path[i].y;
        _original_path_step_lengths.push_back( std::sqrt(_dx * _dx + _dy * _dy) );
    }
    
    int _temp_ct = -1;
    for(double dist : distances)
    {
        _temp_ct+=1;
        int _count = 0;
        double _distance_sum = 0.0;
        double _tail_distance = 0.0;
        bool _find_interval;
        for(int i=0; i<_original_path_step_lengths.size(); i++)
        {
            double _new_distance = _distance_sum + _original_path_step_lengths[i];
            _find_interval = _distance_sum < dist  &&  dist <= _new_distance;
            if (_find_interval)
            {
                _count = i;
                _tail_distance = dist - _distance_sum;
                break;
            }
            _distance_sum = _new_distance;
        }
        // cout << "#" << _temp_ct << " find_interval " << bool(_find_interval)  << "  dist  " << dist  << "  _count  " << _count << "   tail  " << _tail_distance << endl;
        StructPoseReal p1 = original_path[_count];
        StructPoseReal p2 = original_path[_count+1];
        StructPoseReal pnew;
        pnew.x = p1.x + (_tail_distance / _original_path_step_lengths[_count])*(p2.x-p1.x);
        pnew.y = p1.y + (_tail_distance / _original_path_step_lengths[_count])*(p2.y-p1.y);
        p1.yaw = mod_angle_2pi(p1.yaw);
        p2.yaw = mod_angle_2pi(p2.yaw);
        pnew.yaw = p1.yaw + (_tail_distance / _original_path_step_lengths[_count])*(p2.yaw-p1.yaw);
        _result.push_back(pnew);
    }
    _result.push_back(original_path.back());
    return _result;
}


double calc_total_distance_of_one_segment(const vector<StructPoseReal>& original_path)
{
    double _result = 0;
    for(int i=1; i<original_path.size(); i++)
    {
        double _dx = original_path[i-1].x - original_path[i].x;
        double _dy = original_path[i-1].y - original_path[i].y;
        _result += std::sqrt(_dx * _dx + _dy * _dy);
    }
    return _result;
}



class ClassRawPathSampler
{
private:

    ClassCustomPathContainer m_path_;

    ClassCustomPathContainer m_result_;

    StructWaypointWithTwist m_robot_init_state_, m_goal_state_;

    struct StructAccSteerSeries
    {
        vector<double> accel_series;
        vector<double> steer_series;
    }m_accel_steer_series;
    
    

    // int m_expected_num_of_points_in_result_;

    double m_linear_speed_, m_linear_acc_, m_steer_angle, m_steer_rate_;

    double m_step_duration_sec_;

    double m_axle_distance_, m_steer_angle_limit_;
    
private:

    double check_if_point_direction_change(int index);

    // void append_0speed_waypoint(array<double, 3> &point);
    // void append_normal_waypoint(array<double, 3> &point);

public:
    ClassRawPathSampler();
    ~ClassRawPathSampler();

    void set_raw_path_ptr(ClassCustomPathContainer &r_path);

    void set_robot_initial_state(StructWaypointWithTwist robot_init_state);
    void set_goal_state(StructWaypointWithTwist goal_state);

    bool process_raw_data();
    bool convert_custom_path_to_ros_path(nav_msgs::Path& r_sampled_path);

    vector<double> calc_steer_for_a_segment(vector<StructPoseReal>& points, const vector<double>& speeds);

    bool estimate_direction_is_forward(const vector<StructPoseReal>& points);

    vector<int> find_singular_points();

    void split_whole_path(vector<int> singular_points, vector<vector<StructPoseReal>>& result);

    void get_path(ClassCustomPathContainer &r_path);
};

ClassRawPathSampler::ClassRawPathSampler()
{
    // m_expected_num_of_points_in_result_ = 20;
    m_axle_distance_ = 0.14; 
    m_steer_angle_limit_ = 0.9;
    m_step_duration_sec_ = 0.9;
}

ClassRawPathSampler::~ClassRawPathSampler()
{
}


void ClassRawPathSampler::set_raw_path_ptr(ClassCustomPathContainer &r_path)
{
    m_path_.copy_from(r_path);
}


void ClassRawPathSampler::set_goal_state(StructWaypointWithTwist goal_state)
{
    m_goal_state_ = goal_state;
}


void ClassRawPathSampler::set_robot_initial_state(StructWaypointWithTwist robot_init_state)
{
    m_robot_init_state_ = robot_init_state;
}


vector<int> ClassRawPathSampler::find_singular_points()
{
    vector<int> _result;
    size_t _num_points = m_path_.number_of_points();

    if (_num_points < 3)
        return _result;
    
    for(int i=1; i<_num_points-1; i++)
    {
        auto _point_last = m_path_.get_at(i-1);
        auto _point_this = m_path_.get_at(i  );
        auto _point_next = m_path_.get_at(i+1);

        // if (i < 3)
        // {
        //     cout << endl;
        //     cout << _point_last[0] << " " << _point_last[1] << " " << _point_last[2] << endl;
        //     cout << _point_this[0] << " " << _point_this[1] << " " << _point_this[2] << endl;
        //     cout << _point_next[0] << " " << _point_next[1] << " " << _point_next[2] << endl;
        // }

        if (calc_angle_by_three_points(_point_last, _point_this, _point_next) < M_PI/2.0)
        {
            _result.push_back(i);
        }
    }
    return _result;
}


void ClassRawPathSampler::split_whole_path(vector<int> singular_points, vector<vector<StructPoseReal>>& result)
{
    result.clear();

    cout << "singular_points: " <<endl;
    for(int sp : singular_points)
    {
        cout << sp << " ";
    }
    cout << endl;

    vector<array<int,2>> _range_of_all_segments;

    if (singular_points.size() == 0)
    {
        _range_of_all_segments.push_back(array<int,2>{0, int(m_path_.number_of_points()-1)});
    }
    else if (singular_points.size() > 0)
    {
        _range_of_all_segments.push_back(array<int,2>{0, singular_points[0]});
        for(int i=0; i<singular_points.size()-1; i++)
        {
            _range_of_all_segments.push_back(array<int,2>{singular_points[i], singular_points[i+1]});
        }
        _range_of_all_segments.push_back(array<int,2>{singular_points.back(), int(m_path_.number_of_points()-1)});
    }

    auto _original_path = m_path_.get_path();
    
    cout << "_range_of_all_segments" << endl;
    for (auto rg : _range_of_all_segments)
    {
        vector<StructPoseReal> _one_segment;
        cout << "[ " << rg[0] << "  " << rg[1] << " ]" << endl;
        for (int ct=rg[0]; ct<rg[1]+1; ct++)
        {
            cout << ct << " ";
            StructPoseReal _pt(_original_path[ct][0], _original_path[ct][1], _original_path[ct][2]);
            _one_segment.push_back(_pt);
        }
        cout << endl;
        result.push_back(_one_segment);
    }
    
}


bool ClassRawPathSampler::process_raw_data()
{

    if (m_path_.number_of_points() <= 2)
    {
        cout << "too less points in the path. Exit." << endl;
        return false;
    }

    // the input path seems good, so start the processing. 
    int _counter = -1;
    m_result_.clear_points();

    vector<int> _indices_of_singular_points = find_singular_points();

    vector<vector<StructPoseReal>> _result;
    split_whole_path(_indices_of_singular_points, _result);

    // After the whole path is splitted into shorter segments (each segment is a simple
    // one-direction motion), some points are to be sampled from each segment. 
    // Then estimate the required steering and speed for each points. These steer and 
    // speed values will be the initial guess for the curve optimizer.  
    for(vector<StructPoseReal> segment : _result)
    {
        double _segment_length = calc_total_distance_of_one_segment(segment);

        StructMotionSequence _motion_seq;
        _motion_seq.set_time_step_sec(m_step_duration_sec_);
        _motion_seq.set_total_distance(_segment_length);
        _motion_seq.set_limit_accel(0.07);
        _motion_seq.set_limit_decel(-0.1);
        _motion_seq.set_limit_velocity(0.2);
        
        calc_motion_sequence_parameters(_motion_seq);

        // _motion_seq.print_parameters();

        // verify_motion_sequence_parameters(_motion_seq);
        
        
        vector<double> _distance_seq, _speed_seq, _steer_seq;
        _motion_seq.calc_key_distances_by_timestep(_distance_seq, _speed_seq);
        vector<StructPoseReal> _sampled_poses_one_segment = sample_from_one_segment_by_distance(_distance_seq, segment);

        if (estimate_direction_is_forward(_sampled_poses_one_segment))
        {
            _motion_seq.set_forwarding();
        }
        else
        {
            _motion_seq.set_reversing();
            for (int i=0; i<_speed_seq.size(); i++)
            {
                _speed_seq[i] *= -1;
            }
        }

        // cout << "_sampled_poses_one_segment size " << _sampled_poses_one_segment.size();
        // cout << "   _distance_seq size " << _distance_seq.size();
        // cout << "   _speed_seq size " << _speed_seq.size() << endl; // print: 19 18 18

        _steer_seq = calc_steer_for_a_segment(_sampled_poses_one_segment, _speed_seq);

        for(auto ps : _sampled_poses_one_segment)
        {
            m_result_.pushback(ps.to_array3());
            // cout << ps.x << "  " << ps.y << "  " << ps.yaw << endl;
        }
    }


    return true;

}


bool ClassRawPathSampler::estimate_direction_is_forward(const vector<StructPoseReal>& points)
{
    if (points.size() < 2)
    {
        std::cerr << "estimate_direction_is_forward. Not enough points. " << std::endl;
    }

    StructPoseReal p1 = points[0];
    StructPoseReal p2 = points[1];

    double _dx = p2.x - p1.x;
    double _dy = p2.y - p1.y;
    double _yaw_p2_to_p1 = atan2(_dy, _dx);
    _yaw_p2_to_p1 = mod_angle_2pi(_yaw_p2_to_p1);
    double _yaw_p1 = mod_angle_2pi(p1.yaw);
    double _large = std::max(_yaw_p2_to_p1, _yaw_p1);
    double _small = std::min(_yaw_p2_to_p1, _yaw_p1);
    double _diff = _large - _small;
    if (_diff > M_PI)
    {
        _diff = 2*M_PI - _diff;
    }
    if (_diff < (M_PI/2.0))
    {
        return true;
    }
    else
    {
        return false;
    }


}

vector<double> ClassRawPathSampler::calc_steer_for_a_segment(vector<StructPoseReal>& points, const vector<double>& speeds)
{
    // assert(!(points.size() == (speeds.size() + 1)));

    // assert(points.size() == 0);

    vector<double> _result;
    _result.resize(speeds.size());

    for (int i=0; i<points.size()-1; i++)
    {
        ClassSteerSolver _ss(m_axle_distance_, m_steer_angle_limit_, speeds.at(i), m_step_duration_sec_);
        // ClassSteerSolver _ss(m_axle_distance_, m_steer_angle_limit_, 0.2, m_step_duration_sec_);
        _ss.set_point_1(points.at(i).x, points.at(i).y, points.at(i).yaw);
        _ss.set_point_2(points.at(i+1).x, points.at(i+1).y);
        if ( ! _ss.solve())
        {
            std::cerr << "Could not solve the steer for " << i << "-th point." << std::endl;
        }
        _result.assign(i, _ss.get_steer_angle());
        points.at(i+1).yaw = _ss.get_point2_theta();
    }
    return _result;
}



/// @brief In order to visulize the path in RVIZ, the custom type path needs to be converted to
/// ROS nav_msgs::path type.
/// @param r_sampled_path contains the result
/// @return 
bool ClassRawPathSampler::convert_custom_path_to_ros_path(nav_msgs::Path& r_sampled_path)
{
    if (r_sampled_path.poses.size() > 0)
    {
        r_sampled_path.poses.clear();
    }
    
    if (m_result_.number_of_points() < 2)
    {
        return false;
    }

    for (auto pt : m_result_.get_path())
    {
        geometry_msgs::PoseStamped _ps;
        _ps.pose.position.x = pt[0];
        _ps.pose.position.y = pt[1];
        _ps.pose.position.z = 0.0;

        _ps.pose.orientation = tf2qua_to_geoQua(twoD_yaw_to_tf2qua(pt[2]));

        r_sampled_path.poses.push_back( _ps );
    }

    return true;
}


double ClassRawPathSampler::check_if_point_direction_change(int index)
{
    // construct the 3 points.
    // StructPoseReal _prev_point(m_path_ptr_.poses[index-1].pose.position.x, m_path_ptr_.poses[index-1].pose.position.y, 0.0);
    // StructPoseReal _this_point(m_path_ptr_.poses[index  ].pose.position.x, m_path_ptr_.poses[index  ].pose.position.y, 0.0);
    // StructPoseReal _next_point(m_path_ptr_.poses[index+1].pose.position.x, m_path_ptr_.poses[index+1].pose.position.y, 0.0);
    bool _check;
    array<double,3> _temp;
    _temp = m_path_.get_at(index-1);
    StructPoseReal _prev_point(_temp[0], _temp[1], 0.0);
    _temp = m_path_.get_at(index  );
    StructPoseReal _this_point(_temp[0], _temp[1], 0.0);
    _temp = m_path_.get_at(index+1);
    StructPoseReal _next_point(_temp[0], _temp[1], 0.0);

    // get the 2 vectors formed by these 3 points

    double _p1x = _prev_point.x - _this_point.x;
    double _p1y = _prev_point.y - _this_point.y;
    double _p2x = _next_point.x - _this_point.x;
    double _p2y = _next_point.y - _this_point.y;

    // get the unit length versions of those 2 vectors

    double _p1_length = sqrt(_p1x*_p1x + _p1y*_p1y);
    double _p2_length = sqrt(_p2x*_p2x + _p2y*_p2y);

    _p1x /= _p1_length;
    _p1y /= _p1_length;
    _p2x /= _p2_length;
    _p2y /= _p2_length;

    double _angle = acos(_p1x*_p2x + _p1y*_p2y);

    return std::abs(_angle);
}



void ClassRawPathSampler::get_path(ClassCustomPathContainer &r_path)
{
    r_path.clear_points();
    r_path = m_result_;
}




#endif

