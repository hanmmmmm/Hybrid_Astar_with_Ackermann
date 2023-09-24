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
#include "helper_functions.h"


// TODO:
// find out how to interpolate angles.
// estimate steering for each sampled point.
// make container for acceleration and steer. 


using std::cout;
using std::endl;
using std::vector;


struct StructMotionSequence
{
    double t_up = 0;
    double t_hold = 0;
    double t_down = 0;
    double d_up = 0;
    double d_hold = 0;
    double d_down = 0;
    double m_acc = 0;
    double m_max_v = 0;

    double calc_distance_by_time(const double dt, const double velocity_reached);

    vector<double> calc_key_distances_by_timestep(const double step);
};

/// @brief 
/// @param dt: time in seconds since the moment when velocity changes from 0.
/// @return the distance from where the velocity changes from 0. 
double StructMotionSequence::calc_distance_by_time(const double dt, const double velocity_reached)
{
    if (dt < t_up)
        return 0.5 * m_acc * dt * dt;
    else if (dt == t_up)
        return d_up;
    else if (t_up < dt && dt < (t_up+t_hold))
        return d_up + (dt-t_up)*d_hold/t_hold;
    else if (dt == t_up + t_hold)
        return d_up + d_hold;
    else if ( (dt > t_up + t_hold) && (dt < t_up + t_hold + t_down))
    {
        double _t_in_down = dt - t_up - t_hold;
        double _d_in_down = velocity_reached * _t_in_down - 0.5 * m_acc * _t_in_down * _t_in_down;
        return d_up + d_hold + _d_in_down;
    }
    else if (dt >= t_up + t_hold + t_down)
        return d_up + d_hold + d_down;
}


vector<double> StructMotionSequence::calc_key_distances_by_timestep(const double step)
{
    vector<double> _result;
    // _result.push_back(0.0);
    double _total_dt = 0.0;

    while (_total_dt <= (t_up + t_hold + t_down))
    {
        double _key_distance = calc_distance_by_time( _total_dt, m_acc * t_up);
        _result.push_back(_key_distance);

        // cout << "calc_key_distances_by_timestep _total_dt  " << _total_dt  << "  dist  " << _key_distance << endl;
        cout << _total_dt  << "," << _key_distance << endl;

        double _step_size = step;
        if (_total_dt < t_up  &&  (_total_dt + step) > t_up )
        {
            _step_size = t_up - _total_dt;
        }
        else if (_total_dt < (t_up+t_hold)  &&  (_total_dt + step)> (t_up+t_hold) )
        {
            _step_size = t_up + t_hold - _total_dt;
        }
        _total_dt += _step_size;
    }

    return _result;
}



// /// @brief This fuction is to find the time duration for each of these 2 conditions.
// ///  
// ///   ^     _______                     ^     
// /// v |    /       \                  v |    
// ///   |   /         \                   |   /\
// ///   |  /           \                  |  /  \
// ///   +-.--.-------.--.--> t            +---------> t
// ///     t1  t2    t3  t4                  t1 t2 t3
// ///
// /// @param v_max 
// /// @param acc 
// /// @param total_distance 
// /// @return 
// StructMotionSequence get_motion_sequence_durations(double v_max, double acc, double total_distance)
// {
//     /*
//     This method generates valid solution, but it has a proble,:
//     the durations are not integer times of the sampling time-step,
//     so the sampled points do not have constant timestep.
//     A way to imrpve this is increasing the durations calculated here
//     to integer times of the sampling time-step. Then recalculate the v_max 
//     and acc.
//     */
//     StructMotionSequence _result;
//     _result.m_acc = acc;
//     _result.m_max_v = v_max;

//     double _complete_ramp_time = v_max / acc;
//     double _complete_ramp_length = 0.5 * acc * std::pow(_complete_ramp_time, 2);

//     if (total_distance > _complete_ramp_length * 2)
//     {
//         _result.t_up = _complete_ramp_time;
//         _result.t_down = _complete_ramp_time;
//         _result.t_hold = (total_distance - _complete_ramp_length * 2) / v_max;
//         _result.d_up = _complete_ramp_length;
//         _result.d_down = _complete_ramp_length;
//         _result.d_hold = total_distance - _complete_ramp_length * 2;
//     }
//     else if (total_distance < _complete_ramp_length * 2)
//     {
//         double _dt = std::pow(total_distance / acc , 0.5);  // from formula:  d=a*t*t/2
//         _result.t_up = _dt;
//         _result.t_down = _dt;
//         _result.t_hold = 0;
//         _result.d_up = total_distance/2.0;
//         _result.d_down = total_distance/2.0;
//         _result.d_hold = 0;
//     }
//     else if (total_distance == _complete_ramp_length * 2)
//     {
//         _result.t_up = _complete_ramp_time;
//         _result.t_down = _complete_ramp_time;
//         _result.t_hold = 0;
//         _result.d_up = total_distance/2.0;
//         _result.d_down = total_distance/2.0;
//         _result.d_hold = 0;
//     }
    
//     return _result;
// }




/// @brief This fuction is to find the time duration for each of these 2 conditions.
///  
///   ^     _______                     ^     
/// v |    /       \                  v |    
///   |   /         \                   |   /\
///   |  /           \                  |  /  \
///   +-.--.-------.--.--> t            +---------> t
///     t1  t2    t3  t4                  t1 t2 t3
///
/// @param v_max 
/// @param acc 
/// @param total_distance 
/// @return 
StructMotionSequence get_motion_sequence_durations(double v_max, double acc, double total_distance)
{
    /*
    V1:
    This method generates valid solution, but it has a problem:
    the durations are not integer multiply of the sampling time-step,
    so the sampled points do not have constant timestep.
    A way to imrpve this is increasing the durations calculated here
    to integer multiply of the sampling time-step. Then recalculate the v_max 
    and acc.
    */

    /*
    V2:
    This method kind of solves the problem mentioned above. 
    But it has a flaw that I cannot come up with solution right now. 
    The constant speed region might have differeent speed than the top 
    speed reached at the end of the acceleration stage. 
    The reason is that the duration for each stage is extended to integer values,
    so the change in each of them are not same.
    */

    StructMotionSequence _result;

    double _complete_ramp_time = v_max / acc;
    double _complete_ramp_length = 0.5 * acc * std::pow(_complete_ramp_time, 2);

    if (total_distance > _complete_ramp_length * 2)
    {
        _result.t_up = _complete_ramp_time;
        _result.t_up = ceil_number_to_multiply_of_base(_result.t_up, 0.9);
        _result.t_down = _complete_ramp_time;
        _result.t_down = ceil_number_to_multiply_of_base(_result.t_down, 0.9);
        _result.t_hold = (total_distance - _complete_ramp_length * 2) / v_max;
        _result.t_hold = ceil_number_to_multiply_of_base(_result.t_hold, 0.9);

        _result.d_up = _complete_ramp_length;
        _result.d_down = _complete_ramp_length;
        _result.d_hold = total_distance - _complete_ramp_length * 2;

        _result.m_max_v = _result.d_hold / _result.t_hold;
        _result.m_acc = (_result.d_up * 2)/(_result.t_up * _result.t_up);
    }
    else if (total_distance < _complete_ramp_length * 2)
    {
        double _dt = std::pow(total_distance / acc , 0.5);  // from formula:  d=a*t*t/2
        _result.t_up = _dt;
        _result.t_up = ceil_number_to_multiply_of_base(_result.t_up, 0.9);
        _result.t_down = _dt;
        _result.t_down = ceil_number_to_multiply_of_base(_result.t_down, 0.9);
        _result.t_hold = 0;
        _result.d_up = total_distance/2.0;
        _result.d_down = total_distance/2.0;
        _result.d_hold = 0;

        _result.m_acc = (_result.d_up * 2)/(_result.t_up * _result.t_up);
        _result.m_max_v = _result.m_acc * _result.t_up;
    }
    else if (total_distance == _complete_ramp_length * 2)
    {
        _result.t_up = _complete_ramp_time;
        _result.t_up = ceil_number_to_multiply_of_base(_result.t_up, 0.9);
        _result.t_down = _complete_ramp_time;
        _result.t_down = ceil_number_to_multiply_of_base(_result.t_down, 0.9);
        _result.t_hold = 0;
        _result.d_up = total_distance/2.0;
        _result.d_down = total_distance/2.0;
        _result.d_hold = 0;

        _result.m_acc = (_result.d_up * 2)/(_result.t_up * _result.t_up);
        _result.m_max_v = _result.m_acc * _result.t_up;
    }
    
    return _result;
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
        cout << _temp_ct << "  _find_interval  " << _find_interval  << "  dist  " << dist  << "  _count  " << _count << "   tail  " << _tail_distance << endl;
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

    // int m_expected_num_of_points_in_result_;

    double m_linear_speed_, m_linear_acc_, m_steer_angle, m_steer_rate_;

    double m_step_duration_sec_;
    
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

    vector<int> find_singular_points();

    void split_whole_path(vector<int> singular_points, vector<vector<StructPoseReal>>& result);

    void get_path(ClassCustomPathContainer &r_path);
};

ClassRawPathSampler::ClassRawPathSampler()
{
    // m_expected_num_of_points_in_result_ = 20;
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

    vector<array<int, 2>> _range_of_all_segments;

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
    size_t _total_num_of_points = m_path_.number_of_points();

    if (_total_num_of_points <= 2)
    {
        cout << "too less points in the path. Exit." << endl;
        return false;
    }

    int _counter = -1;
    m_result_.clear_points();

    vector<vector<StructPoseReal>> _result;

    vector<int> _indices_of_singular_points = find_singular_points();
    split_whole_path(_indices_of_singular_points, _result);

    for(vector<StructPoseReal> segment : _result)
    {
        vector<double> distances;
        double _segment_length = calc_total_distance_of_one_segment(segment);
        StructMotionSequence _motion_seq = get_motion_sequence_durations(0.25, 0.08, _segment_length);
        vector<double> _distance_seq = _motion_seq.calc_key_distances_by_timestep(0.9);
        vector<StructPoseReal> _sampled_poses_one_segment = sample_from_one_segment_by_distance(_distance_seq, segment);
        cout << "_motion_seq t_up   " << _motion_seq.t_up << endl;
        cout << "_motion_seq t_hold " << _motion_seq.t_hold << endl;
        cout << "_motion_seq t_down " << _motion_seq.t_down << endl;
        cout << "_motion_seq d_up   " << _motion_seq.d_up << endl;
        cout << "_motion_seq d_hold " << _motion_seq.d_hold << endl;
        cout << "_motion_seq d_down " << _motion_seq.d_down << endl;
        cout << "_motion_seq m_acc " << _motion_seq.m_acc << endl;
        cout << "_motion_seq m_max_v " << _motion_seq.m_max_v << endl;
        cout << "_motion_seq t total " << _motion_seq.t_up + _motion_seq.t_hold + _motion_seq.t_down << endl;
        cout << "_motion_seq d total " << _motion_seq.d_up + _motion_seq.d_hold + _motion_seq.d_down << endl;
        cout << "one segment total dist  " << _segment_length << endl;
        // for(auto dist : _distance_seq)
        // {
        //     cout << "dist  " << dist << endl;
        // }
        for(auto ps : _sampled_poses_one_segment)
        {
            m_result_.pushback(ps.to_array3());
            cout << ps.x << "  " << ps.y << "  " << ps.yaw << endl;
        }
    }


    return true;

}


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

