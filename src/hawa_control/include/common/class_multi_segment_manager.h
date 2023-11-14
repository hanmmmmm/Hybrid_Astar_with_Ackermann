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


#ifndef HAWA_CLASS_MULTI_SEGMENT_MANAGER_H
#define HAWA_CLASS_MULTI_SEGMENT_MANAGER_H

#include <ros/console.h>
#include <vector>
#include <array>

#include "nav_msgs/Path.h"
#include "class_elemental_path2d_segment.h"
#include "tools_angles.h"


/**
 * @brief This class has the logics of converting the ros path msg into the format that is usable by 
 * my custom pure pursuit implementation. Also has the logics for other module to use the path information.  
*/
class ClassHawaMultiSegmentManager
{
private:

    int m_counter_current_segment_;

    nav_msgs::Path m_original_path_;

    std::vector<ClassPath2DSegment> m_vector_segments_;

    bool m_finished_all_;

    double m_distance_to_end_;

private:
    std::vector<int> findSingularPoints();

    void split_whole_path(std::vector<int> singular_points);

    bool estimateDirectionIsForward(ClassPath2DSegment& points);

    bool checkCurrentSegFinish(const double robot_x,const double robot_y,const double robot_yaw);

    
public:
    ClassHawaMultiSegmentManager();

    ~ClassHawaMultiSegmentManager();

    void setPath(nav_msgs::Path path_input);

    void finishCurrentSegment();

    bool doesPathExist();

    bool didFinishAll();

    ClassPath2DSegment getCurrentSegment();

    int getCounter();

    nav_msgs::Path getOriginalPath();

    void update(const double robot_x,const double robot_y,const double robot_yaw);

    double getDistToEnd();
};

/**
 * @brief Constructor function. Also initialize some member variables.
*/
ClassHawaMultiSegmentManager::ClassHawaMultiSegmentManager()
{
    m_counter_current_segment_ = 0;
    m_finished_all_ = true;
    m_distance_to_end_ = 0;
}

ClassHawaMultiSegmentManager::~ClassHawaMultiSegmentManager()
{
}

/**
 * @brief Call function when the robot reaches the end of the current segment. The counter holding the 
 * the index of the active segment will increment by 1. And if this is the last segment, then the flag
 * of finishing_the_entire_path will be updated too.  
*/
void ClassHawaMultiSegmentManager::finishCurrentSegment()
{
    m_counter_current_segment_ ++;
    ROS_INFO_STREAM("finishCurrentSegment() " << m_counter_current_segment_);
    if (m_counter_current_segment_ >= m_vector_segments_.size())
    {
        m_finished_all_ = true;
    }
}

/**
 * @brief Call this function to setup the path data. Also reinitialize the related flags. 
*/
void ClassHawaMultiSegmentManager::setPath(nav_msgs::Path path_input)
{
    m_original_path_ = path_input;

    std::vector<int> _sgpoints = findSingularPoints();
    // split_whole_path(_sgpoints, m_vector_segments_);
    
    split_whole_path(_sgpoints);

    m_finished_all_ = false;

    m_counter_current_segment_ = 0;
}

/**
 * @brief This function will go through the original long path vector, and record the indices of the 
 * waypoints where the path has large angle change. These are the points where the robot needs to 
 * switch between forward and reverse motion. 
 * @return A vector of the indices. 
*/
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
                                            quaternionToEularYaw(_ptr_poses.at(i-1).pose.orientation)};
        std::array<double, 3> _point_this = {_ptr_poses.at(i).pose.position.x,
                                            _ptr_poses.at(i).pose.position.y,
                                            quaternionToEularYaw(_ptr_poses.at(i).pose.orientation)};
        std::array<double, 3> _point_next = {_ptr_poses.at(i+1).pose.position.x,
                                            _ptr_poses.at(i+1).pose.position.y,
                                            quaternionToEularYaw(_ptr_poses.at(i+1).pose.orientation)};

        if (calcAngleByThreePoints(_point_last, _point_this, _point_next) < M_PI/2.0)
        {
            _result.push_back(i);
        }
    }
    return _result;
}

/**
 * @brief According to the vector of indices found by findSingularPoints(), split the orignal long
 * path into several segments. The extracted segments will be stored in a member variable. 
 * @param singular_points The vector of the indices where the path needs to be cut.
*/
void ClassHawaMultiSegmentManager::split_whole_path(std::vector<int> singular_points)
{
    ROS_INFO_STREAM("split_whole_path(), " << singular_points.size() << " sg points.");

    m_vector_segments_.clear();

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

    for (auto rg : _range_of_all_segments)
    {
        ClassPath2DSegment _one_segment;
        for (int ct=rg[0]; ct<rg[1]+1; ct++)
        {
            _one_segment.pushbackForOriginal(ClassPose2D(m_original_path_.poses.at(ct).pose.position.x, 
                                                         m_original_path_.poses.at(ct).pose.position.y, 
                                                         quaternionToEularYaw( m_original_path_.poses.at(ct).pose.orientation)));
            _one_segment.pushbackForExtended(ClassPose2D(m_original_path_.poses.at(ct).pose.position.x, 
                                                         m_original_path_.poses.at(ct).pose.position.y, 
                                                         quaternionToEularYaw( m_original_path_.poses.at(ct).pose.orientation)));
        }
        _one_segment.extendThePath();
        if (estimateDirectionIsForward(_one_segment))
        {
            _one_segment.setDirectionForward();
        }
        else
        {
            _one_segment.setDirectionBackward();
        }
        
        m_vector_segments_.push_back(_one_segment);
    }
    ROS_INFO_STREAM("num of segments: " << m_vector_segments_.size());
}

/**
 * @brief To get the active segment that the robot is running on.
 * @return The segment.
*/
ClassPath2DSegment ClassHawaMultiSegmentManager::getCurrentSegment()
{
    return m_vector_segments_.at(m_counter_current_segment_);
}

/**
 * @brief A function to know if there is any unfinished segment left. 
 * @return True if there's at least one segment left. False if there's no left or no path setup yet. 
*/
bool ClassHawaMultiSegmentManager::doesPathExist()
{
    ROS_INFO_STREAM("doesPathExist: " << m_vector_segments_.size() << " " <<  m_counter_current_segment_);
    return bool(m_vector_segments_.size() - m_counter_current_segment_);
}

/**
 * @brief For a given segment, this function tells if the robot will move in forward direction or reveser 
 * direction.
 * @param r_segment reference of the segment.
 * @return True if it is forward. False if it is reversing. 
*/
bool ClassHawaMultiSegmentManager::estimateDirectionIsForward(ClassPath2DSegment& r_segment)
{
    if (r_segment.m_path_extended_.size() < 2)
    {
        std::cerr << "estimateDirectionIsForward. Not enough points. " << std::endl;
    }

    ClassPose2D p1 = r_segment.m_path_extended_[0];
    ClassPose2D p2 = r_segment.m_path_extended_[1];

    double _dx = p2.x - p1.x;
    double _dy = p2.y - p1.y;
    double _yaw_p2_to_p1 = std::atan2(_dy, _dx);
    _yaw_p2_to_p1 = mod2pi(_yaw_p2_to_p1);
    double _yaw_p1 = mod2pi(p1.yaw);
    double _large = std::max(_yaw_p2_to_p1, _yaw_p1);
    double _small = std::min(_yaw_p2_to_p1, _yaw_p1);
    double _diff = _large - _small;
    if (_diff > M_PI)
    {
        _diff = 2*M_PI - _diff;
    }

    return (_diff < (M_PI/2.0));
}

/**
 * @brief With given robot pose, check if this segment can be marked as finished. The only critiria is the distance 
 * from the roobt the end of the current active segment. More robust ideas could be added in the future. 
 * @return True if finished. False if not finished. 
*/
bool ClassHawaMultiSegmentManager::checkCurrentSegFinish(const double robot_x, 
                                                         const double robot_y, 
                                                         const double robot_yaw)
{
    ClassPose2D _end = m_vector_segments_.at(m_counter_current_segment_).m_path_original_.back();

    double _dist_to_end = computeDistanceMeter(_end.x, 
                                               _end.y, 
                                               robot_x, 
                                               robot_y);
    // ROS_INFO_STREAM("_dist_to_end " << _dist_to_end << "  look_ahead " << parameters_.look_ahead_distance_meter_);
    m_distance_to_end_ = _dist_to_end;
    return (_dist_to_end <= 0.2);
}

/**
 * @brief Call this function when the setup functions in this class have done. This is one of the main functions
 * in this class. The input is the robot pose. 
*/
void ClassHawaMultiSegmentManager::update(const double robot_x, const double robot_y, const double robot_yaw)
{
    if (m_vector_segments_.size() <= 0)
    {
        return;
    }

    if (m_finished_all_)
    {
        ROS_INFO_STREAM_NAMED("ClassHawaMultiSegmentManager::update()", "Wait for new path.");
        return;
    }

    bool _this_finished = checkCurrentSegFinish( robot_x, robot_y, robot_yaw);
    ROS_INFO_STREAM("update() _this_finished " << _this_finished);
    if (_this_finished)
    {
        finishCurrentSegment();
    }
}

/**
 * @brief This function returns the original unprocessed path data.
 * @return The original path in format of nav_msgs::Path.
*/
nav_msgs::Path ClassHawaMultiSegmentManager::getOriginalPath()
{
    return m_original_path_;
}

/**
 * @brief This function returns the value of counter that is the index of the current active segment. 
 * @return The index.
*/
int ClassHawaMultiSegmentManager::getCounter()
{
    return m_counter_current_segment_;
}

/**
 * @brief This function returns the latest value of the distance between the robot and the end of the current
 * active segment.
 * @return The distance in meter.
*/
double ClassHawaMultiSegmentManager::getDistToEnd()
{
    return m_distance_to_end_;
}

/**
 * @brief This function tells if the robot has finished all segments. 
 * @return True if all finished.  False if not. 
*/
bool ClassHawaMultiSegmentManager::didFinishAll()
{
    return m_finished_all_;
}


#endif