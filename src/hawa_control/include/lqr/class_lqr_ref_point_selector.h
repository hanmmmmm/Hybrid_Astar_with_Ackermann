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

#ifndef CLASS_LRQ_REF_POINT_SELECTOR_H_
#define CLASS_LRQ_REF_POINT_SELECTOR_H_

#include <iostream>
#include <memory>
#include <math.h>

#include "nav_msgs/msg/path.hpp"
#include "common/class_elemental_pose2d.h"
#include "common/class_elemental_path2d_segment.h"


namespace hawa
{

/**
 * @brief Class for selecting reference points for LQR controller
*/
class ClassLqrRefPointSelector
{
private:

    std::shared_ptr<ClassPath2DSegment> m_path_ptr_;
    ClassPose2D m_robot_pose_;
    ClassPose2D m_ref_pose_;
    bool m_going_forward_ = true;

private:

    int findClosestPointIndex();

    int decideRefPointIndex(const int curr_idx);

    double calcPointYaw(const int pt_idx);


public:
    ClassLqrRefPointSelector() {}
    ~ClassLqrRefPointSelector() {}

    void setPathPtr(std::shared_ptr<ClassPath2DSegment> path_ptr);

    void setRobotPose(const ClassPose2D& robot_pose);

    void resetRefPose();

    ClassPose2D getRefPose();

    bool process();

    void setGoingForward(bool going_forward);

};

} // namespace hawa


#endif