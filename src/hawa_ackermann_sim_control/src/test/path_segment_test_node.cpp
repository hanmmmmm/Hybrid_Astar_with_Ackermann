
#include "../../include/car_control/class_module_path_segmenter.h"

#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/Path.h"


void path_cb(const nav_msgs::PathConstPtr _msg)
{
    std::cout << "\n" << "path_cb Start." << std::endl;

    ClassPathSegmenter seger;
    seger.set_ros_path(_msg);
    StructResultPath _result;
    bool _valid;
    seger.segment(_result, _valid);

    std::cout << "Num of segments: " << _result.data.size() << std::endl;
    int ct = 0;
    for (auto seg : _result.data)
    {
        std::cout << "Seg " << ct << ". Num of points: " << seg.original_waypoints.size() << std::endl;
        ct ++;

        for (auto pt : seg.original_waypoints)
        {
            std::cout << pt.x_meter << " " << pt.y_meter << std::endl;
        }
    }

    std::cout << "path_cb Done." << std::endl;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_seg_node");

    ros::NodeHandle nh;

    ros::Subscriber path_suber_ = nh.subscribe( "/path", 1, path_cb);

    ros::AsyncSpinner s(2);
    s.start();

    ros::waitForShutdown();

    return 0;
}






