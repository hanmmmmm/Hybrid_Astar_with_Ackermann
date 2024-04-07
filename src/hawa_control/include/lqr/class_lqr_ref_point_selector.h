
#ifndef CLASS_LRQ_REF_POINT_SELECTOR_H_
#define CLASS_LRQ_REF_POINT_SELECTOR_H_

#include <iostream>
#include <memory>
#include <math.h>

#include "nav_msgs/msg/path.hpp"
#include "common/class_elemental_pose2d.h"
#include "common/class_elemental_path2d_segment.h"


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

    int findClosestPointIndex()
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

    int decideRefPointIndex(const int curr_idx)
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

    double calcPointYaw(const int pt_idx)
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


public:
    ClassLqrRefPointSelector();
    ~ClassLqrRefPointSelector() {};

    void setPathPtr(std::shared_ptr<ClassPath2DSegment> path_ptr)
    {
        m_path_ptr_ = path_ptr;
    }

    void setRobotPose(const ClassPose2D& robot_pose)
    {
        m_robot_pose_ = robot_pose;
    }

    void resetRefPose()
    {
        m_ref_pose_.set(0, 0, 0);
    }

    ClassPose2D getRefPose()
    {
        return m_ref_pose_;
    }

    bool process()
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

    void setGoingForward(bool going_forward)
    {
        m_going_forward_ = going_forward;
    }

};

ClassLqrRefPointSelector::ClassLqrRefPointSelector()
{
}



#endif