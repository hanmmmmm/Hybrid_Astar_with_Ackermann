#ifndef CLASS_ELEMENTAL_PATH2D_SEGMENT_H
#define CLASS_ELEMENTAL_PATH2D_SEGMENT_H

#include <vector>
#include <deque>

#include "class_elemental_pose2d.h"

class ClassPath2DSegment
{
public:
    enum EnumDirection
    {
        forawrd,
        reverse
    };

    EnumDirection direction_;
    
private:

    
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

    std::deque< ClassPose2D > path_segment__original_;  // the points from input path.
    std::deque< ClassPose2D > path_segment__extended_;  // the original + the extra points extended at the begining and end of the original.
    
};

ClassPath2DSegment::ClassPath2DSegment()
{
    direction_ = EnumDirection::forawrd;
}

ClassPath2DSegment::~ClassPath2DSegment()
{
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

#endif