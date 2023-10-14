#ifndef CLASS_CAR_BODY_BOX_MARKER
#define CLASS_CAR_BODY_BOX_MARKER

#include <string>

#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

class ClassCarBox2D
{
private:
    double rectangle_width_, rectangle_length_;
    double dist_robot_base_to_end_;
public:
    ClassCarBox2D();
    ~ClassCarBox2D();

    void set_frame_id(std::string id_str);

    visualization_msgs::Marker box_2d_marker_;

    visualization_msgs::Marker get_marker_msg( );
};

ClassCarBox2D::ClassCarBox2D()
{
    rectangle_width_ = 0.3;
    rectangle_length_ = 0.3;
    dist_robot_base_to_end_ = 0.05;
}

ClassCarBox2D::~ClassCarBox2D()
{
}

void ClassCarBox2D::set_frame_id(std::string id_str)
{
    box_2d_marker_.header.frame_id = "base_link";  //id_str;
    box_2d_marker_.color.r = 0.0;
    box_2d_marker_.color.g = 0.0;
    box_2d_marker_.color.b = 1.0;
    box_2d_marker_.color.a = 1.0;
    box_2d_marker_.type = box_2d_marker_.LINE_STRIP;
    box_2d_marker_.lifetime.fromSec(5);
    box_2d_marker_.scale.x = 0.03;
    box_2d_marker_.scale.y = 0.03;
    box_2d_marker_.scale.z = 0.03;

    box_2d_marker_.pose.orientation.w = 1.0;

    box_2d_marker_.pose.position.x = - dist_robot_base_to_end_;
    box_2d_marker_.pose.position.y = - rectangle_width_ / 2.0;
}



visualization_msgs::Marker ClassCarBox2D::get_marker_msg( )
{
    double _width_half = rectangle_width_ / 2.0 ;
    geometry_msgs::Point _p1;
    _p1.x = 0; // - ( dist_robot_base_to_end_ );
    _p1.y = 0;

    geometry_msgs::Point _p2;
    _p2.x = 0;
    _p2.y = rectangle_width_;

    geometry_msgs::Point _p3;
    _p3.x = rectangle_length_;
    _p3.y = rectangle_width_;

    geometry_msgs::Point _p4;
    _p4.x = rectangle_length_ + 0.3;
    _p4.y = _width_half + 0.03;

    geometry_msgs::Point _p5;
    _p5.x = rectangle_length_ + 0.3;
    _p5.y = _width_half - 0.03;

    geometry_msgs::Point _p6;
    _p6.x = rectangle_length_;
    _p6.y = 0;

    box_2d_marker_.points.push_back(_p1);
    box_2d_marker_.points.push_back(_p2);
    box_2d_marker_.points.push_back(_p3);
    box_2d_marker_.points.push_back(_p4);
    box_2d_marker_.points.push_back(_p5);
    box_2d_marker_.points.push_back(_p6);

    return box_2d_marker_;
}


#endif