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

#include "lqr/class_lqr_ref_point_selector.h"

namespace hawa
{

/**
 * @brief Find the closest point to the current position
*/
int ClassLqrRefPointSelector::findClosestPointIndex()
{
    // Find the closest point to the current position
    double min_dist = 1e5;
    int min_idx = 0;
    for (int i = 0; i < m_path_ptr_->m_path_extended_.size(); i++)
    {
        auto& pose = m_path_ptr_->m_path_extended_[i];
        const double dx = pose.x - m_robot_pose_.x;
        const double dy = pose.y - m_robot_pose_.y;
        const double dist_sq = dx * dx + dy * dy;
        
        if (dist_sq < min_dist)
        {
            min_dist = dist_sq;
            min_idx = i;
        }
    }
    return min_idx;
}

/**
 * @brief Decide the index of the reference point
*/
int ClassLqrRefPointSelector::decideRefPointIndex(const int curr_idx)
{
    int look_ahead_dist = 3;  // num of points to look ahead, not metric distance

    int ref_idx = curr_idx + look_ahead_dist;

    if (ref_idx < 0)
    {
        ref_idx = 0;
        std::cerr << "Warning: ref_idx < 0" << std::endl;
    }
    else if (ref_idx >= m_path_ptr_->m_path_extended_.size())
    {
        ref_idx = m_path_ptr_->m_path_extended_.size() - 1;
        std::cerr << "Warning: ref_idx >= path size" << std::endl;
    }

    return ref_idx;
}

/**
 * @brief Calculate the yaw of the point
*/
double ClassLqrRefPointSelector::calcPointYaw(const int pt_idx)
{
    if (pt_idx < 0 || pt_idx > m_path_ptr_->m_path_extended_.size() - 1)
    {
        std::cerr << "Error: pt_idx out of range" << std::endl;
        return 0;
    }

    ClassPose2D ps1, ps2;

    if (m_going_forward_)
    {
        if (pt_idx == 0)
        {
            ps1 = m_path_ptr_->m_path_extended_[0];
            ps2 = m_path_ptr_->m_path_extended_[1];
        }
        else
        {
            ps1 = m_path_ptr_->m_path_extended_[pt_idx - 1];
            ps2 = m_path_ptr_->m_path_extended_[pt_idx];
        }
    }
    else
    {
        if (pt_idx == 0)
        {
            ps1 = m_path_ptr_->m_path_extended_[1];
            ps2 = m_path_ptr_->m_path_extended_[0];
        }
        else
        {
            ps1 = m_path_ptr_->m_path_extended_[pt_idx + 1];
            ps2 = m_path_ptr_->m_path_extended_[pt_idx];
        }
    }
    
    double dx = ps2.x - ps1.x;
    double dy = ps2.y - ps1.y;

    return std::atan2(dy, dx);
}

/**
 * @brief Set the path pointer
*/
void ClassLqrRefPointSelector::setPathPtr(std::shared_ptr<ClassPath2DSegment> path_ptr)
{
    m_path_ptr_ = path_ptr;
}

/**
 * @brief Set the robot pose
*/
void ClassLqrRefPointSelector::setRobotPose(const ClassPose2D& robot_pose)
{
    m_robot_pose_ = robot_pose;
}

/**
 * @brief Reset the reference pose
*/
void ClassLqrRefPointSelector::resetRefPose()
{
    m_ref_pose_.set(0, 0, 0);
}

/**
 * @brief Get the reference pose
*/
ClassPose2D ClassLqrRefPointSelector::getRefPose()
{
    return m_ref_pose_;
}

/**
 * @brief Process the reference point selection
*/
bool ClassLqrRefPointSelector::process()
{
    if (m_path_ptr_ == nullptr)
    {
        std::cerr << "Error: path_ptr is nullptr" << std::endl;
        return false;
    }

    if (m_path_ptr_->m_path_extended_.size() == 0)
    {
        std::cerr << "Error: path size is 0" << std::endl;
        return false;
    }

    resetRefPose();

    int curr_idx = findClosestPointIndex();

    int ref_idx = decideRefPointIndex(curr_idx);

    // std::cout << "curr_idx: " << curr_idx << " ref_idx: " << ref_idx << std::endl;

    m_ref_pose_ = m_path_ptr_->m_path_extended_[ref_idx];
    m_ref_pose_.yaw = calcPointYaw(ref_idx);

    return true;
}

/**
 * @brief Set the direction of the path
*/
void ClassLqrRefPointSelector::setGoingForward(bool going_forward)
{
    m_going_forward_ = going_forward;
}


} // namespace hawa