// MIT License

// Copyright (c) 2023 Mingjie

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


/*

input: the complete Navmsg/path 

processing:

find singular points;
split into short segment, so that each segment only one direction;
find directoin in each segment;
a counter to know which segment should be used now; 
receive the signal to know robot has reached the end of the current segment;


*/

#ifndef HAWA_CLASS_MULTI_SEGMENT_MANAGER_H
#define HAWA_CLASS_MULTI_SEGMENT_MANAGER_H

#include <ros/console.h>
#include <vector>
#include <array>

#include "nav_msgs/Path.h"
#include "class_elemental_path2d_segment.h"
#include "tools_angles.h"


class ClassHawaMultiSegmentManager
{
private:

    int m_counter_current_segment_;

    nav_msgs::Path m_original_path_;

    std::vector<ClassPath2DSegment> m_vector_segments_;

    bool m_finished_all_;

private:
    std::vector<int> findSingularPoints();
    void split_whole_path(std::vector<int> singular_points, std::vector<ClassPath2DSegment>& result);

    bool estimate_direction_is_forward(ClassPath2DSegment& points);

    bool checkCurrentSegFinish(const double robot_x,const double robot_y,const double robot_yaw);

    
public:
    ClassHawaMultiSegmentManager();
    ~ClassHawaMultiSegmentManager();

    void setPath(nav_msgs::Path path_input);

    void finishCurrentSegment();

    bool doesPathExist();

    ClassPath2DSegment getCurrentSegment();

    int getCounter();

    nav_msgs::Path getOriginalPath();

    void update(const double robot_x,const double robot_y,const double robot_yaw);
    
};

ClassHawaMultiSegmentManager::ClassHawaMultiSegmentManager()
{
    m_counter_current_segment_ = 0;
    m_finished_all_ = true;
}

ClassHawaMultiSegmentManager::~ClassHawaMultiSegmentManager()
{
}


void ClassHawaMultiSegmentManager::finishCurrentSegment()
{
    m_counter_current_segment_ ++;
    ROS_INFO_STREAM("finishCurrentSegment() " << m_counter_current_segment_);
    if (m_counter_current_segment_ >= m_vector_segments_.size())
    {
        m_finished_all_ = true;
    }
}


void ClassHawaMultiSegmentManager::setPath(nav_msgs::Path path_input)
{
    m_original_path_ = path_input;

    std::vector<int> _sgpoints = findSingularPoints();
    split_whole_path(_sgpoints, m_vector_segments_);

    m_finished_all_ = false;

    m_counter_current_segment_ = 0;

}


std::vector<int> ClassHawaMultiSegmentManager::findSingularPoints()
{
    std::vector<int> _result;
    size_t _num_points = m_original_path_.poses.size();

    if (_num_points < 3)
        return _result;

    std::vector<geometry_msgs::PoseStamped>& _ptr_poses = m_original_path_.poses;
    
    for(int i=1; i<_num_points-1; i++)
    {
        std::array<double, 3> _point_last = {_ptr_poses.at(i-1).pose.position.x,
                                            _ptr_poses.at(i-1).pose.position.y,
                                            quaternion_to_eular_yaw(_ptr_poses.at(i-1).pose.orientation)};
        std::array<double, 3> _point_this = {_ptr_poses.at(i).pose.position.x,
                                            _ptr_poses.at(i).pose.position.y,
                                            quaternion_to_eular_yaw(_ptr_poses.at(i).pose.orientation)};
        std::array<double, 3> _point_next = {_ptr_poses.at(i+1).pose.position.x,
                                            _ptr_poses.at(i+1).pose.position.y,
                                            quaternion_to_eular_yaw(_ptr_poses.at(i+1).pose.orientation)};

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


void ClassHawaMultiSegmentManager::split_whole_path(std::vector<int> singular_points, 
                                                    std::vector<ClassPath2DSegment>& result)
{
    result.clear();

    // // cout << "singular_points: " <<endl;
    // for(int sp : singular_points)
    // {
    //     cout << sp << " ";
    // }
    // cout << endl;

    ROS_INFO_STREAM("split_whole_path(), " << singular_points.size() << " sg points.");

    std::vector<std::array<int,2>> _range_of_all_segments;

    if (singular_points.size() == 0)
    {
        _range_of_all_segments.push_back(std::array<int,2>{0, int(m_original_path_.poses.size()-1)});
    }
    else if (singular_points.size() > 0)
    {
        _range_of_all_segments.push_back(std::array<int,2>{0, 
                                                           singular_points[0]});
        for(int i=0; i<singular_points.size()-1; i++)
        {
            _range_of_all_segments.push_back(std::array<int,2>{singular_points[i], 
                                                               singular_points[i+1]});
        }
        _range_of_all_segments.push_back(std::array<int,2>{singular_points.back(), 
                                                           int(m_original_path_.poses.size()-1)});
    }

    // auto _original_path = m_path_.get_path();
    
    // cout << "_range_of_all_segments" << endl;
    for (auto rg : _range_of_all_segments)
    {
        ClassPath2DSegment _one_segment;
        // vector<StructPoseReal> _one_segment;
        // std::cout << "[ " << rg[0] << "  " << rg[1] << " ]" << std::endl;
        for (int ct=rg[0]; ct<rg[1]+1; ct++)
        {
            // cout << ct << " ";
            // StructPoseReal _pt(_original_path[ct][0], _original_path[ct][1], _original_path[ct][2]);
            _one_segment.pushback_for_original(ClassPose2D(m_original_path_.poses.at(ct).pose.position.x, 
                                                           m_original_path_.poses.at(ct).pose.position.y, 
                                                           quaternion_to_eular_yaw( m_original_path_.poses.at(ct).pose.orientation)));
            _one_segment.pushback_for_extended(ClassPose2D(m_original_path_.poses.at(ct).pose.position.x, 
                                                           m_original_path_.poses.at(ct).pose.position.y, 
                                                           quaternion_to_eular_yaw( m_original_path_.poses.at(ct).pose.orientation)));
            // _one_segment.push_back(_pt);
        }
        _one_segment.extendThePath();
        if (estimate_direction_is_forward(_one_segment))
        {
            _one_segment.set_direction_forward();
        }
        else
        {
            _one_segment.set_direction_backward();
        }
        
        // cout << endl;
        // result.push_back(_one_segment);
        m_vector_segments_.push_back(_one_segment);
    }
    ROS_INFO_STREAM("num of segments: " << m_vector_segments_.size());
}

ClassPath2DSegment ClassHawaMultiSegmentManager::getCurrentSegment()
{
    return m_vector_segments_.at(m_counter_current_segment_);
}

bool ClassHawaMultiSegmentManager::doesPathExist()
{
    ROS_INFO_STREAM("doesPathExist: " << m_vector_segments_.size() << " " <<  m_counter_current_segment_);
    return bool(m_vector_segments_.size() - m_counter_current_segment_);
}



bool ClassHawaMultiSegmentManager::estimate_direction_is_forward(ClassPath2DSegment& r_segment)
{
    if (r_segment.path_segment__extended_.size() < 2)
    {
        std::cerr << "estimate_direction_is_forward. Not enough points. " << std::endl;
    }

    ClassPose2D p1 = r_segment.path_segment__extended_[0];
    ClassPose2D p2 = r_segment.path_segment__extended_[1];

    double _dx = p2.x - p1.x;
    double _dy = p2.y - p1.y;
    double _yaw_p2_to_p1 = std::atan2(_dy, _dx);
    _yaw_p2_to_p1 = mod_2pi(_yaw_p2_to_p1);
    double _yaw_p1 = mod_2pi(p1.yaw);
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


bool ClassHawaMultiSegmentManager::checkCurrentSegFinish(const double robot_x,const double robot_y,const double robot_yaw)
{
    ClassPose2D _end = m_vector_segments_.at(m_counter_current_segment_).path_segment__original_.back();

    double _dist_to_end = compute_distance_meter(_end.x, 
                                                 _end.y, 
                                                 robot_x, 
                                                 robot_y);
    // ROS_INFO_STREAM("_dist_to_end " << _dist_to_end << "  look_ahead " << parameters_.look_ahead_distance_meter_);
    return (_dist_to_end <= 0.2);
}

void ClassHawaMultiSegmentManager::update(const double robot_x,const double robot_y,const double robot_yaw)
{
    if (m_vector_segments_.size() <= 0)
    {
        return;
    }

    if (m_finished_all_)
    {
        ROS_INFO_STREAM("update(), wait for new path.");
        return;
    }

    


    bool _this_finished = checkCurrentSegFinish( robot_x, robot_y, robot_yaw);
    ROS_INFO_STREAM("update() _this_finished " << _this_finished);
    if (_this_finished)
    {
        finishCurrentSegment();
    }

}


nav_msgs::Path ClassHawaMultiSegmentManager::getOriginalPath()
{
    return m_original_path_;
}

int ClassHawaMultiSegmentManager::getCounter()
{
    return m_counter_current_segment_;
}

#endif