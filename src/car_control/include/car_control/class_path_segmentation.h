#ifndef CLASS_PATH_SEGMENTATION_H
#define CLASS_PATH_SEGMENTATION_H

/**
 * @file class_path_segmentation.h
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
- add the feature that decide each segment moving forward or backward.
- add the featrue for upsampling between 2 given waypoints.

*/

#include <iostream>
#include <math.h>
#include "class_elemental_path2d.h"
#include "class_elemental_path2d_segment.h"
#include "class_elemental_pose2d.h"
#include "nav_msgs/Path.h"  // ros nav_msgs


/**
 * @brief This provides a method for cutting one path into smaller pieces.
 * The key critiria is the sudden yaw changes in adjecent waypoints.
 * Usually such change means the car is swtiching bwtn forward and backward gears.
 * So the path will be cur into smaller pieces at these points.
 */
class ClassPathSegmentation
{
private:
    ClassPath2D *original_path_ptr_;
    ClassPath2D result_path_;

    double compute_yaw_change(double yaw1, double yaw2);
    double compute_yaw_of_2_points(double x1, double y1, double x2, double y2);
    double mod_2pi(double angle);

public:
    ClassPathSegmentation(ClassPath2D *in_path);
    ~ClassPathSegmentation();

    void process(bool &valid);
    ClassPath2D get_result();
};

/**
 * @brief Construct a new Class Path Segmentation:: Class Path Segmentation object
 *
 * @param in_path  The pointer of the original path to be processed.
 */
ClassPathSegmentation::ClassPathSegmentation(ClassPath2D *in_path)
    : original_path_ptr_(in_path)
{
}

/**
 * @brief Destroy the Class Path Segmentation:: Class Path Segmentation object
 *
 */
ClassPathSegmentation::~ClassPathSegmentation()
{
}


ClassPath2D ClassPathSegmentation::get_result(){
    return result_path_;
}


/**
 * @brief main function of this class.
 * Now, only works with input path being one big segment.
 * The case that input path consists of >=2 segments will be added later.
 * @param valid  Returns true if this operation is valid.
 */
void ClassPathSegmentation::process(bool &valid)
{
    if (original_path_ptr_->size() <= 0)
    {
        valid = false;
        return;
    }
    else if (original_path_ptr_->size() > 1)
    {
        result_path_ = *original_path_ptr_;
        valid = true;
        return;
    }

    bool valid_segment;
    ClassPath2DSegment old_big_segment = original_path_ptr_->at(0, valid_segment);
    if (!valid_segment)
    {
        valid = false;
        return;
    }

    ClassPath2DSegment current_seg;
    bool pose_is_valid;
    ClassPose2D initial_pose = old_big_segment.at(0, pose_is_valid);
    if (!pose_is_valid)
    {
        valid = false;
        return;
    }

    current_seg.add_pose_at_back(initial_pose); // the first pose from the input path doesn't need process, just put it into the reault path.

    bool valid_index;
    ClassPose2D p1, p2, p3;
    double yaw1, yaw2, yaw_diff;
    for (int i = 1; i < old_big_segment.size() - 2; i++)
    {
        p1 = old_big_segment.at(i - 1, valid_index);
        if (!valid_index)
        {
            valid = false;
            return;
        }
        p2 = old_big_segment.at(i, valid_index);
        if (!valid_index)
        {
            valid = false;
            return;
        }
        p3 = old_big_segment.at(i + 1, valid_index);
        if (!valid_index)
        {
            valid = false;
            return;
        }
        yaw1 = compute_yaw_of_2_points(p1.x, p1.y, p2.x, p2.y);
        yaw2 = compute_yaw_of_2_points(p2.x, p2.y, p3.x, p3.y);
        yaw_diff = compute_yaw_change(yaw1, yaw2);

        current_seg.add_pose_at_back(p2);

        if (yaw_diff > M_PI / 2.0)
        {
            result_path_.add_segment(current_seg);
            current_seg.clear();
        }
    }
}

/// @brief compute the minimal yaw change between 2 given yaw values.
/// @param yaw1
/// @param yaw2
/// @return double , radian.
double ClassPathSegmentation::compute_yaw_change(double yaw1, double yaw2)
{
    yaw1 = mod_2pi(yaw1);
    yaw2 = mod_2pi(yaw2);
    if (yaw1 == yaw2)
        return 0.0;
    double d_yaw = std::max(yaw1, yaw2) - std::min(yaw1, yaw2);
    if (d_yaw <= M_PI)
        return d_yaw;
    else
        return M_PI * 2.0 - d_yaw;
}

/// @brief compute the yaw angle of the line formed by 2 points
/// @param x1
/// @param y1
/// @param x2
/// @param y2
/// @return double , radian.
inline double ClassPathSegmentation::compute_yaw_of_2_points(double x1, double y1, double x2, double y2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    double yaw = atan2(dy, dx);
    return yaw;
}

/// @brief change angle into the range of 0-2pi
/// @param a the input angle.
/// @return the output angle.
inline double ClassPathSegmentation::mod_2pi(double a)
{
    double angle = a;
    while (angle > 2 * M_PI)
    {
        angle -= 2 * M_PI;
    }
    while (angle < 0)
    {
        angle += 2 * M_PI;
    }
    return angle;
}

#endif