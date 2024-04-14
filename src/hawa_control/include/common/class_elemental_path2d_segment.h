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


namespace hawa
{


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

} // namespace hawa



#endif