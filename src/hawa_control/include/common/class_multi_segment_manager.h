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

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <array>

#include "nav_msgs/msg/path.hpp"
#include "class_elemental_path2d_segment.h"
#include "tools_angles.h"


namespace hawa
{

/**
 * @brief This class has the logics of converting the ros path msg into the format that is usable by 
 * my custom pure pursuit implementation. Also has the logics for other module to use the path information.  
*/
class ClassHawaMultiSegmentManager
{
private:

    int m_counter_current_segment_;

    nav_msgs::msg::Path m_original_path_;

    std::vector<ClassPath2DSegment> m_vector_segments_;

    bool m_finished_all_;

    double m_distance_to_end_;

private:
    std::vector<int> findSingularPoints();

    void split_whole_path(std::vector<int> singular_points);

    bool estimateDirectionIsForward(ClassPath2DSegment& points);

    bool checkCurrentSegFinish(const double robot_x,const double robot_y,const double robot_yaw);

    void ros_info(const std::string& str);
    void ros_warn(const std::string& str, const int t=5);

    
public:
    ClassHawaMultiSegmentManager();

    ~ClassHawaMultiSegmentManager();

    void setPath(nav_msgs::msg::Path path_input);

    void finishCurrentSegment();

    bool doesPathExist();

    bool didFinishAll();

    ClassPath2DSegment getCurrentSegment();

    int getCounter();

    nav_msgs::msg::Path getOriginalPath();

    void update(const double robot_x,const double robot_y,const double robot_yaw);

    double getDistToEnd();
};

} // namespace hawa

#endif