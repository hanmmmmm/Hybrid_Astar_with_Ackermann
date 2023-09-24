#ifndef CLASS_PATH2D_H
#define CLASS_PATH2D_H

/**
 * @file class_path2d.h
 * @author Mingjie
 * @brief
 * @version 0.1
 * @date 2023-01-18
 *
 * @copyright Copyright (c) 2023
 *
 */

/*
TODO:
add a function for converting ros_path_msg to custom path type

*/

#include <deque>
#include <vector>

#include "class_elemental_path2d_segment.h"

class ClassPath2D
{
private:
    std::vector<ClassPath2DSegment> path_;
public:
    ClassPath2D();
    ~ClassPath2D();
    
    ClassPath2DSegment get_segment(int index);
    ClassPath2DSegment at(int index, bool &valid);
    void add_segment(ClassPath2DSegment seg);
    size_t size();
};

ClassPath2D::ClassPath2D()
{
}

ClassPath2D::~ClassPath2D()
{
}

ClassPath2DSegment ClassPath2D::at(int index, bool &valid)
{
    if (index >= 0 && index < path_.size())
    {
        valid = true;
        return path_[index];
    }
    else{
        valid = false;
        return ClassPath2DSegment();
    }
}

size_t ClassPath2D::size()
{
    return path_.size();
}

#endif