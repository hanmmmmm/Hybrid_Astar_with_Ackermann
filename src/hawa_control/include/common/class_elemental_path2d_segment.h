#ifndef CLASS_ELEMENTAL_PATH2D_SEGMENT_H
#define CLASS_ELEMENTAL_PATH2D_SEGMENT_H

#include <vector>
#include <deque>

#include "class_elemental_pose2d.h"
#include "nav_msgs/Path.h"

class ClassPath2DSegment
{
public:
    enum EnumDirection
    {
        forawrd,
        reverse
    };

    // EnumDirection direction_;
    
private:
    EnumDirection direction_;
    
public:
    ClassPath2DSegment();
    ~ClassPath2DSegment();

    void pushback_for_original(ClassPose2D pose);
    void pushfront_for_original(ClassPose2D pose);
    
    void pushback_for_extended(ClassPose2D pose);
    void pushfront_for_extended(ClassPose2D pose);

    void set_direction_forward();
    void set_direction_backward();
    
    // void at_original(int index, ClassPose2D& pose, bool& valid);
    
    // size_t size();
    
    void clear();

    bool isForward();

    void extendThePath();

    std::deque< ClassPose2D > path_segment__original_;  // the points from input path.
    std::deque< ClassPose2D > path_segment__extended_;  // the original + the extra points extended at the begining and end of the original.
    
    nav_msgs::Path toRosPath();

};

ClassPath2DSegment::ClassPath2DSegment()
{
    direction_ = EnumDirection::forawrd;
}

ClassPath2DSegment::~ClassPath2DSegment()
{
}

void ClassPath2DSegment::pushback_for_original(ClassPose2D pose)
{
    path_segment__original_.push_back(pose);
}

void ClassPath2DSegment::pushback_for_extended(ClassPose2D pose)
{
    path_segment__extended_.push_back(pose);
}

void ClassPath2DSegment::set_direction_forward()
{
    direction_ = EnumDirection::forawrd;
}

void ClassPath2DSegment::set_direction_backward()
{
    direction_ = EnumDirection::reverse;
}

// void ClassPath2DSegment::at(int index, ClassPose2D& pose, bool& valid)
// {
//     if (index >= 0 && index < path_segment.size())
//     {
//         valid = true;
//         return path_segment[index];
//     }
//     else{
//         valid = false;
//         return ClassPose2D();
//     }
// }

// size_t ClassPath2DSegment::size()
// {
//     return path_segment.size();
// }

void ClassPath2DSegment::clear()
{
    path_segment__original_.clear();
    path_segment__extended_.clear();
    direction_ = EnumDirection::forawrd;
}

bool ClassPath2DSegment::isForward()
{
    return direction_ == EnumDirection::forawrd;
}

void ClassPath2DSegment::extendThePath()
{
    double _path_size = path_segment__extended_.size();
    double _dx = path_segment__extended_.back().x - path_segment__extended_.at(_path_size-2).x;
    double _dy = path_segment__extended_.back().y - path_segment__extended_.at(_path_size-2).y;
    ClassPose2D _temp(0, 0, 0);
    double _last_x = path_segment__extended_.back().x;
    double _last_y = path_segment__extended_.back().y;
    for(int i=1; i<7; i++)
    {
        _temp.x = _last_x + _dx * i;
        _temp.y = _last_y + _dy * i;
        _temp.yaw = path_segment__extended_.back().yaw;
        path_segment__extended_.push_back(_temp);
    }
}

nav_msgs::Path ClassPath2DSegment::toRosPath()
{
    nav_msgs::Path _ros_path;
    for (auto pose : path_segment__original_)
    {
        geometry_msgs::PoseStamped _temp;
        _temp.pose.position.x = pose.x;
        _temp.pose.position.y = pose.y;
        _temp.pose.orientation.w = 1;
        _ros_path.poses.push_back(_temp);
    }
    return _ros_path;
}


#endif