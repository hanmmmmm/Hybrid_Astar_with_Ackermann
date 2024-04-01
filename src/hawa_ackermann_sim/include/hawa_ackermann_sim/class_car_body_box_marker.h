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


#ifndef HAWA_CLASS_CAR_BODY_BOX_MARKERRR
#define HAWA_CLASS_CAR_BODY_BOX_MARKERRR

#include <string>

#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
// #include "geometry_msgs/msg/polygon_stamped.hpp"


namespace hawa
{
    
/**
 * @brief This class is for generating the box around robot base. This box is only for 
 * visulization in RVIZ.
 * 
 */
class ClassCarBox2D
{
private:
    double m_rectangle_width_, m_rectangle_length_;  // the abosulte width and length of the box.
    double m_dist_robot_base_to_end_;
public:
    ClassCarBox2D();
    ~ClassCarBox2D();

    void setFrameID(std::string id_str);

    visualization_msgs::msg::Marker m_box_2d_marker_;
    visualization_msgs::msg::Marker getMarkerMsg( );

    // geometry_msgs::msg::PolygonStamped m_box_2d_marker_;
    // geometry_msgs::msg::PolygonStamped getMarkerMsg( );
};


}

#endif