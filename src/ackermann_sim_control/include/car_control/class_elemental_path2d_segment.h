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

private:
    
public:
    ClassPath2DSegment();
    ~ClassPath2DSegment();
    std::deque< ClassPose2D > path_segment;
    EnumDirection direction_;
    void add_pose_at_back(ClassPose2D pose);
    void add_pose_at_front(ClassPose2D pose);
    ClassPose2D at(int index, bool& valid);
    size_t size();
    void clear();
};

ClassPath2DSegment::ClassPath2DSegment()
{
}

ClassPath2DSegment::~ClassPath2DSegment()
{
}

ClassPose2D ClassPath2DSegment::at(int index, bool& valid)
{
    if (index >= 0 && index < path_segment.size())
    {
        valid = true;
        return path_segment[index];
    }
    else{
        valid = false;
        return ClassPose2D();
    }
}

size_t ClassPath2DSegment::size()
{
    return path_segment.size();
}

void ClassPath2DSegment::clear()
{
    path_segment.clear();
}

#endif