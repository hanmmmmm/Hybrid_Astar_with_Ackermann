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

namespace hawa
{

class ClassPath2D
{

private:
    std::vector<ClassPath2DSegment> path_;

public:
    ClassPath2D() {};
    ~ClassPath2D() {};
    
    ClassPath2DSegment get_segment(int index);

    ClassPath2DSegment at(int index, bool &valid);

    void add_segment(ClassPath2DSegment seg);

    size_t size();
};

} // namespace hawa

#endif