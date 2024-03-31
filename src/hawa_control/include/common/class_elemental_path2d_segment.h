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

#ifndef CLASS_HAWA_ELEMENTAL_PATH2D_SEGMENT_H
#define CLASS_HAWA_ELEMENTAL_PATH2D_SEGMENT_H

#include <vector>
#include <deque>

#include "class_elemental_pose2d.h"
#include "nav_msgs/msg/path.hpp"


/**
 * @brief This class will be used to store a segment from the path. Segments are divided at where the
 * robot is going to switch between forward motion and backward motion. The major components are the two 
 * containers holding the waypoints and a variable holding the direction of this segment. 
*/
class ClassPath2DSegment
{
public:
    enum EnumDirection
    {
        forawrd,
        reverse
    };

private:
    EnumDirection m_direction_;
    
public:
    ClassPath2DSegment();
    ~ClassPath2DSegment();

    void pushbackForOriginal(ClassPose2D pose);
    
    void pushbackForExtended(ClassPose2D pose);

    void setDirectionForward();
    void setDirectionBackward();
    
    void clear();

    bool isForward();

    void extendThePath();

    std::deque< ClassPose2D > m_path_original_;  
    // the points from input path.

    std::deque< ClassPose2D > m_path_extended_;  
    // the original + the extra points extended at the end of the original.
    
    nav_msgs::msg::Path toRosPath();

};

ClassPath2DSegment::ClassPath2DSegment()
{
    m_direction_ = EnumDirection::forawrd;
}

ClassPath2DSegment::~ClassPath2DSegment()
{
}

/**
 * @brief Add a pose into the member variable of original_path. Add at the end. 
 * @param pose The pose to be added. Type is a custom type: ClassPose2D
*/
void ClassPath2DSegment::pushbackForOriginal(ClassPose2D pose)
{
    m_path_original_.push_back(pose);
}

/**
 * @brief Add a pose into the member variable of extended_path. Add at the end. 
 * @param pose The pose to be added. Type is a custom type: ClassPose2D
*/
void ClassPath2DSegment::pushbackForExtended(ClassPose2D pose)
{
    m_path_extended_.push_back(pose);
}

/**
 * @brief Call this to set the direction of this segment to be forawrd.
*/
void ClassPath2DSegment::setDirectionForward()
{
    m_direction_ = EnumDirection::forawrd;
}

/**
 * @brief Call this to set the direction of this segment to be reversing.
*/
void ClassPath2DSegment::setDirectionBackward()
{
    m_direction_ = EnumDirection::reverse;
}

/**
 * @brief Call this when you want to empty the values in this class. 
*/
void ClassPath2DSegment::clear()
{
    m_path_original_.clear();
    m_path_extended_.clear();
    m_direction_ = EnumDirection::forawrd;
}

/**
 * @brief Call this to know if this segment is going forward to going reverse.
 * @return True means forward. False means reverse. 
*/
bool ClassPath2DSegment::isForward()
{
    return m_direction_ == EnumDirection::forawrd;
}

/**
 * @brief Call this function after all points are added in the 'extend path'. This function will 
 * put several more points at the end. These points are in the same direction of the last two points
 * in the 'extend path'. These extra points will be needed by the pure pursuit algo 
 * to find the target point, when the robot is approaching to the end the segment. 
*/
void ClassPath2DSegment::extendThePath()
{
    double _path_size = m_path_extended_.size();
    double _dx = m_path_extended_.back().x - m_path_extended_.at(_path_size-2).x;
    double _dy = m_path_extended_.back().y - m_path_extended_.at(_path_size-2).y;
    ClassPose2D _temp(0, 0, 0);
    double _last_x = m_path_extended_.back().x;
    double _last_y = m_path_extended_.back().y;

    // the value 7 here is arbitrary. This decides how many new points are added. 
    for(int i=1; i<7; i++)
    {
        _temp.x = _last_x + _dx * i;
        _temp.y = _last_y + _dy * i;
        _temp.yaw = m_path_extended_.back().yaw;
        m_path_extended_.push_back(_temp);
    }
}

/**
 * @brief Convert the path from custom type to ROS standard navmsgs path. Becasue at this moment (2023Nov) this
 * ros path is only used for visulization, I didn't put the actual yaw for the waypoints in this path.
 * @return the converted path.
*/
nav_msgs::msg::Path ClassPath2DSegment::toRosPath()
{
    nav_msgs::msg::Path _ros_path;
    for (auto pose : m_path_original_)
    {
        geometry_msgs::msg::PoseStamped _temp;
        _temp.pose.position.x = pose.x;
        _temp.pose.position.y = pose.y;
        _temp.pose.orientation.w = 1;
        _ros_path.poses.push_back(_temp);
    }
    return _ros_path;
}


#endif