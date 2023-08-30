#ifndef CLASS_CORRIDOR_H
#define CLASS_CORRIDOR_H


#include <iostream>
#include <array>
#include <vector>
#include <string>

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "struct_simple_pose.h"

using std::cout, std::endl;
using std::array, std::vector;
using std::string;


/*
TODO:
record the number of markers; if latest array shorter than previous, then delete the extra ones.

*/


class ClassCorridor
{
private:
    vector<StructRectangle> m_result_;

    visualization_msgs::MarkerArray m_msg_;
    size_t m_max_num_boxes_;

public:
    ClassCorridor();
    ~ClassCorridor();

    void add_one_box(double xmin, double xmax, double ymin, double ymax);
    void clear_content();
    bool get_vector_boxes(vector<StructRectangle>& cont);
    bool get_ros_marker_format( const string frameid, visualization_msgs::MarkerArray& r_marker_array, const double offset_x_metric, const double offset_y_metric);
};

ClassCorridor::ClassCorridor()
{
    m_result_.clear();
    m_max_num_boxes_ = 0;
}

ClassCorridor::~ClassCorridor()
{
}

void ClassCorridor::add_one_box(double xmin, double xmax, double ymin, double ymax)
{
    m_result_.push_back(StructRectangle{xmax,ymax,xmin,ymin});
}

void ClassCorridor::clear_content()
{
    m_result_.clear();
}

bool ClassCorridor::get_vector_boxes(vector<StructRectangle>& cont)
{
    cont = m_result_;
    return true;
}

bool ClassCorridor::get_ros_marker_format(const string frameid, visualization_msgs::MarkerArray& r_marker_array, const double offset_x_metric, const double offset_y_metric)
{
    // m_msg_.markers.clear();

    size_t _max = std::max(m_max_num_boxes_, m_result_.size());
    m_msg_.markers.resize(_max);

    int _ct = 0;
    // for (auto rect : m_result_)
    for (int i=0; i<_max; i++)
    {
        auto rect = m_result_[i];

        if (_ct < m_result_.size())
        {
            // visualization_msgs::Marker _temp_line;
            visualization_msgs::Marker * _temp_line = &(m_msg_.markers[_ct]);
            _temp_line->header.frame_id = frameid;
            _temp_line->id = _ct;
            _ct ++;
            _temp_line->type = visualization_msgs::Marker::LINE_STRIP;
            _temp_line->action = visualization_msgs::Marker::MODIFY;
            _temp_line->pose.orientation.w = 1.0;
            _temp_line->scale.x = 0.015;
            _temp_line->color.a = 1.0;
            // _temp_line->color.r = 1.0;
            // _temp_line->color.b = 0.2;
            geometry_msgs::Point _temp_point;
            _temp_point.x = rect.x_dw + offset_x_metric;
            _temp_point.y = rect.y_dw + offset_y_metric;
            _temp_line->points.push_back(_temp_point);
            _temp_point.x = rect.x_up + offset_x_metric;
            _temp_point.y = rect.y_dw + offset_y_metric;
            _temp_line->points.push_back(_temp_point);
            _temp_point.x = rect.x_up + offset_x_metric;
            _temp_point.y = rect.y_up + offset_y_metric;
            _temp_line->points.push_back(_temp_point);
            _temp_point.x = rect.x_dw + offset_x_metric;
            _temp_point.y = rect.y_up + offset_y_metric;
            _temp_line->points.push_back(_temp_point);
            _temp_point.x = rect.x_dw + offset_x_metric;
            _temp_point.y = rect.y_dw + offset_y_metric;
            _temp_line->points.push_back(_temp_point);

            // cout << "rectangle x " << rect.x_dw << " " << rect.x_up;
            // cout << " " << rect.y_dw << " " << rect.y_up << endl;

        }
        else
        {
            // this is for removing the extra markers. When latest markerarray is ahorter
            // than the previous one, some markers will be left in rviz.
            // but these two lines are not removing it at all. May need fix or this is bug in rviz.
            m_msg_.markers[_ct].action = visualization_msgs::Marker::DELETE;
            m_msg_.markers[_ct].color.a = 0.0;
        }
        
    }

    

    m_max_num_boxes_ = std::max(m_max_num_boxes_, m_result_.size());

    r_marker_array = m_msg_;
    // cout << "Add " << _ct << " rectangles." << endl;
    return true;
}


#endif