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


/**
 * @file class_car_body_box_marker.h
 * @author Mingjie
 * @brief This is a class for generating the box around robot base. This box is only for 
 * visulization in RVIZ.
 * @version 0.3
 * @date 2023-11-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#ifndef HAWA_CLASS_CAR_BODY_BOX_MARKER
#define HAWA_CLASS_CAR_BODY_BOX_MARKER

#include <string>

#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

class ClassCarBox2D
{
private:
    double m_rectangle_width_, m_rectangle_length_;
    double m_dist_robot_base_to_end_;
public:
    ClassCarBox2D();
    ~ClassCarBox2D();

    void setFrameID(std::string id_str);

    visualization_msgs::Marker m_box_2d_marker_;

    visualization_msgs::Marker getMarkerMsg( );
};

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
    m_box_2d_marker_.header.frame_id = "base_link";  //id_str;
    m_box_2d_marker_.color.r = 0.0;
    m_box_2d_marker_.color.g = 0.0;
    m_box_2d_marker_.color.b = 1.0;
    m_box_2d_marker_.color.a = 1.0;
    m_box_2d_marker_.type = m_box_2d_marker_.LINE_STRIP;
    m_box_2d_marker_.lifetime.fromSec(5);
    m_box_2d_marker_.scale.x = 0.03;
    m_box_2d_marker_.scale.y = 0.03;
    m_box_2d_marker_.scale.z = 0.03;

    m_box_2d_marker_.pose.orientation.w = 1.0;

    m_box_2d_marker_.pose.position.x = - m_dist_robot_base_to_end_;
    m_box_2d_marker_.pose.position.y = - m_rectangle_width_ / 2.0;
}

/**
 * @brief Get the marker message that's ready to be publsihed. 
 * @return the visualization_msgs::Marker message.
*/
visualization_msgs::Marker ClassCarBox2D::getMarkerMsg( )
{
    double _width_half = m_rectangle_width_ / 2.0 ;
    geometry_msgs::Point _p1;
    _p1.x = 0; // - ( m_dist_robot_base_to_end_ );
    _p1.y = 0;

    geometry_msgs::Point _p2;
    _p2.x = 0;
    _p2.y = m_rectangle_width_;

    geometry_msgs::Point _p3;
    _p3.x = m_rectangle_length_;
    _p3.y = m_rectangle_width_;

    geometry_msgs::Point _p4;
    _p4.x = m_rectangle_length_ + 0.3;
    _p4.y = _width_half + 0.03;

    geometry_msgs::Point _p5;
    _p5.x = m_rectangle_length_ + 0.3;
    _p5.y = _width_half - 0.03;

    geometry_msgs::Point _p6;
    _p6.x = m_rectangle_length_;
    _p6.y = 0;

    m_box_2d_marker_.points.push_back(_p1);
    m_box_2d_marker_.points.push_back(_p2);
    m_box_2d_marker_.points.push_back(_p3);
    m_box_2d_marker_.points.push_back(_p4);
    m_box_2d_marker_.points.push_back(_p5);
    m_box_2d_marker_.points.push_back(_p6);

    return m_box_2d_marker_;
}


#endif