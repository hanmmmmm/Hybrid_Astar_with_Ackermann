#ifndef CLASS_MODULE_PATH_SEGMENTER
#define CLASS_MODULE_PATH_SEGMENTER


#include <iostream>
#include <math.h>
// #include "class_elemental_path2d.h"
// #include "class_elemental_path2d_segment.h"
// #include "class_elemental_pose2d.h"
#include "nav_msgs/Path.h"  // ros nav_msgs

double HALF_PI = M_PI/2.0;

struct StructPose2D
{
    double x_meter;
    double y_meter;
    double yaw_rad;

    StructPose2D(double _x, double _y, double _yaw):x_meter(_x), y_meter(_y), yaw_rad(_yaw){}
};


struct StructResultOneSegment
{
    // ClassPath2D original_waypoints;
    // ClassPath2D extended_waypoints;
    std::vector<StructPose2D> original_waypoints;
    std::vector<StructPose2D> extended_waypoints;
    double original_wappoints_distance = 0.0;
    double extended_wappoints_distance = 0.0;

    void reset_values()
    {
        original_waypoints.clear();
        extended_waypoints.clear();
        original_wappoints_distance = 0.0;
        extended_wappoints_distance = 0.0;
    }
};

struct StructResultPath
{
    std::vector<StructResultOneSegment> data;
};


/// @brief change angle into the range of 0-2pi
/// @param a the input angle.
/// @return the output angle.
double mod_2pi(double _angle)
{
    while (_angle > 2 * M_PI)
    {
        _angle -= 2 * M_PI;
    }
    while (_angle < 0)
    {
        _angle += 2 * M_PI;
    }
    return _angle;
}


class ClassPathSegmenter
{
private:
    nav_msgs::Path ros_path_msg_;


private:
    double compute_yaw_change(double yaw1, double yaw2);
    double compute_yaw_of_2_points(double x1, double y1, double x2, double y2);
    double compute_distance_of_2_points(double x1, double y1, double x2, double y2);
    void extend_from_point(double _x, double _y, double _yaw, std::vector<StructPose2D> &_seg);
    bool validate_segment(const std::vector<StructPose2D> &_seg);

public:
    ClassPathSegmenter();
    ~ClassPathSegmenter();

    void set_ros_path(const nav_msgs::PathConstPtr &_msg);

    void segment(StructResultPath &_result, bool &_ok);
};

ClassPathSegmenter::ClassPathSegmenter()
{
}

ClassPathSegmenter::~ClassPathSegmenter()
{
}

void ClassPathSegmenter::set_ros_path(const nav_msgs::PathConstPtr &_msg)
{
    ros_path_msg_.header = _msg->header; 
    ros_path_msg_.poses = _msg->poses;
}

void ClassPathSegmenter::segment(StructResultPath &_result, bool &_ok)
{
    size_t path_total_waypoint_number = ros_path_msg_.poses.size();
    std::cout << "Num of points in input path : " << path_total_waypoint_number << std::endl;
    if (path_total_waypoint_number <= 3)
    {
        _ok = false;
        return;
    }
    _result.data.clear();
    StructResultOneSegment _one_seg;
    geometry_msgs::PoseStamped last_pose, this_pose, next_pose; 
    double _yaw1, _yaw2, _yaw_change;
    _one_seg.original_waypoints.push_back(StructPose2D(ros_path_msg_.poses[0].pose.position.x, \
                                                       ros_path_msg_.poses[0].pose.position.y, \
                                                       0 ));
    _one_seg.extended_waypoints.push_back(StructPose2D(ros_path_msg_.poses[0].pose.position.x, \
                                                       ros_path_msg_.poses[0].pose.position.y, \
                                                       0 ));
    for (int i = 1; i <= path_total_waypoint_number-1; i ++)
    {
        last_pose = ros_path_msg_.poses[i-1];
        this_pose = ros_path_msg_.poses[i];
        if (i != path_total_waypoint_number-1)
        {
            next_pose = ros_path_msg_.poses[i+1]; 
        }
        else
        {
            next_pose.pose.position.x = this_pose.pose.position.x + (this_pose.pose.position.x - last_pose.pose.position.x);
            next_pose.pose.position.y = this_pose.pose.position.y + (this_pose.pose.position.y - last_pose.pose.position.y);
        }
        
        _yaw1 = compute_yaw_of_2_points(last_pose.pose.position.x, last_pose.pose.position.y, \
                                                this_pose.pose.position.x, this_pose.pose.position.y);
        _yaw2 = compute_yaw_of_2_points(this_pose.pose.position.x, this_pose.pose.position.y, \
                                                next_pose.pose.position.x, next_pose.pose.position.y);
        _yaw_change = compute_yaw_change( _yaw1, _yaw2);
        _one_seg.original_wappoints_distance += compute_distance_of_2_points(this_pose.pose.position.x, this_pose.pose.position.y, \
                                                next_pose.pose.position.x, next_pose.pose.position.y);
        std::cout << "point (" << this_pose.pose.position.x << " " << this_pose.pose.position.y << ") ";

        _one_seg.original_waypoints.push_back(StructPose2D(this_pose.pose.position.x, \
                                                            this_pose.pose.position.y, \
                                                            0 ));
        _one_seg.extended_waypoints.push_back(StructPose2D(this_pose.pose.position.x, \
                                                            this_pose.pose.position.y, \
                                                            0 ));

        if (_yaw_change < HALF_PI)
        {
            std::cout << " straight. " << std::endl;
            if (i != path_total_waypoint_number-1)
            {
                continue;
            }
        }
        std::cout << " Changes. " << std::endl;

        // there's a sharp change in yaw at this waypoint.
        if (_one_seg.original_wappoints_distance > 0.01)
        {
            extend_from_point(this_pose.pose.position.x, this_pose.pose.position.y, _yaw1, _one_seg.extended_waypoints);
            _result.data.push_back(_one_seg);
        }
        _one_seg.reset_values();
        _one_seg.original_waypoints.push_back(StructPose2D(this_pose.pose.position.x, \
                                                            this_pose.pose.position.y, \
                                                            0 ));
        _one_seg.extended_waypoints.push_back(StructPose2D(this_pose.pose.position.x, \
                                                            this_pose.pose.position.y, \
                                                            0 ));
    }

    if (_result.data.size() >= 1)
    {
        _ok = true;
        return;
    }
    _ok = false;    
}


void ClassPathSegmenter::extend_from_point(double _x, double _y, double _yaw, std::vector<StructPose2D> &_seg)
{
    double unit_distance_meter = 0.1;
    double dx = unit_distance_meter * cos(_yaw);
    double dy = unit_distance_meter * sin(_yaw);
    for(int i=0; i<10; i++)
    {
        _x += dx;
        _y += dy;
        _seg.push_back(StructPose2D(_x, _y, _yaw));
    }
}

// bool ClassPathSegmenter::validate_segment(const std::vector<StructPose2D> &_seg)
// {
//     double _total_distance = 0;

//     for( StructPose2D point : _seg)
//     {

//     }

// }

inline double ClassPathSegmenter::compute_distance_of_2_points(double x1, double y1, double x2, double y2)
{
    return sqrt(std::pow((x2-x1),2) + std::pow((y2-y1),2)); 
}


/// @brief compute the minimal yaw change between 2 given yaw values.
/// @param yaw1
/// @param yaw2
/// @return double , radian.
double ClassPathSegmenter::compute_yaw_change(double yaw1, double yaw2)
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
inline double ClassPathSegmenter::compute_yaw_of_2_points(double x1, double y1, double x2, double y2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    double yaw = atan2(dy, dx);
    return yaw;
}



#endif