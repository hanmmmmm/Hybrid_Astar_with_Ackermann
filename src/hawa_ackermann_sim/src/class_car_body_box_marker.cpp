
#include "hawa_ackermann_sim/class_car_body_box_marker.h"



namespace hawa
{

ClassCarBox2D::ClassCarBox2D()
{
    m_rectangle_width_ = 0.3;
    m_rectangle_length_ = 0.3;
    m_dist_robot_base_to_end_ = 0.05;
}

ClassCarBox2D::~ClassCarBox2D()
{
}

/**
 * @brief Set the frame id of the box. Also set some other visulization parameters. 
 * @param id_str The name of the frame. 
*/
void ClassCarBox2D::setFrameID(std::string id_str)
{
    auto _temp = id_str;
    m_box_2d_marker_.header.frame_id = "base_link";  //id_str;
    m_box_2d_marker_.color.r = 0.0;
    m_box_2d_marker_.color.g = 0.0;
    m_box_2d_marker_.color.b = 1.0;
    m_box_2d_marker_.color.a = 1.0;
    m_box_2d_marker_.type = m_box_2d_marker_.LINE_STRIP;
    // m_box_2d_marker_.lifetime.fromSec(5);
    m_box_2d_marker_.scale.x = 0.03;
    m_box_2d_marker_.scale.y = 0.03;
    m_box_2d_marker_.scale.z = 0.03;

    m_box_2d_marker_.pose.orientation.w = 1.0;

    m_box_2d_marker_.pose.position.x = - m_dist_robot_base_to_end_;
    m_box_2d_marker_.pose.position.y = - m_rectangle_width_ / 2.0;

    m_box_2d_marker_.action = m_box_2d_marker_.MODIFY;
}

/**
 * @brief Get the marker message that's ready to be publsihed. 
 * @return the visualization_msgs::Marker message.
*/
visualization_msgs::msg::Marker ClassCarBox2D::getMarkerMsg( )
{
    double _width_half = m_rectangle_width_ / 2.0 ;
    geometry_msgs::msg::Point _p1;
    _p1.x = 0; // - ( m_dist_robot_base_to_end_ );
    _p1.y = 0;

    geometry_msgs::msg::Point _p2;
    _p2.x = 0;
    _p2.y = m_rectangle_width_;

    geometry_msgs::msg::Point _p3;
    _p3.x = m_rectangle_length_;
    _p3.y = m_rectangle_width_;

    geometry_msgs::msg::Point _p4;
    _p4.x = m_rectangle_length_ + 0.3;
    _p4.y = _width_half + 0.03;

    geometry_msgs::msg::Point _p5;
    _p5.x = m_rectangle_length_ + 0.3;
    _p5.y = _width_half - 0.03;

    geometry_msgs::msg::Point _p6;
    _p6.x = m_rectangle_length_;
    _p6.y = 0;

    m_box_2d_marker_.points.clear();
    m_box_2d_marker_.points.push_back(_p1);
    m_box_2d_marker_.points.push_back(_p2);
    m_box_2d_marker_.points.push_back(_p3);
    m_box_2d_marker_.points.push_back(_p4);
    m_box_2d_marker_.points.push_back(_p5);
    m_box_2d_marker_.points.push_back(_p6);
    m_box_2d_marker_.points.push_back(_p1);

    return m_box_2d_marker_;
}


} // namespace hawa
