#ifndef CLASS_CORRIDOR_GENERATOR_H
#define CLASS_CORRIDOR_GENERATOR_H

// Work Flow:
//
// receive map data
// receive path points
// build empty result container
// loop through each point in path
//   find the corridor box limits
//   append in result
// return result

// find the corridor box limits:
//
// convert metric_xy to grid xy
// decide how many grids width/height to check
// decide the order of indices of rows and columns to check
// check each grids following the order.
// if grid is obstacle, stop extending in this direction.
// convert grid limits to metric limits.

#include <iostream>
#include <vector>

#include "../car_pathplan/class_custom_path.h"
#include "../car_pathplan/class_gridmap_2d_handler.h"
#include "../car_pathplan/class_corridor.h"


class ClassCorridorGenerator
{
private:
    ClassCustomPathContainer m_waypoints_;
    ClassGridMap2DHandler m_gridmap_handler_;
    ClassCorridor m_result_as_corridor;
    std::vector<double> m_result_as_vector_;

    double m_max_bbox_half_size_meter_;

public:
    ClassCorridorGenerator();

    ~ClassCorridorGenerator();

    bool set_map_data(
        const int map_width_grid,
        const int map_height_grid,
        vector<int8_t> &map,
        const double grid_resolution,
        const int8_t plan_threshold);

    void set_points_data(const ClassCustomPathContainer &data);

    bool generate_bboxs();

    void get_result(ClassCorridor& cont);

    void get_result_as_vector(std::vector<double> cont);
};

ClassCorridorGenerator::ClassCorridorGenerator()
{
    m_max_bbox_half_size_meter_ = 0.25;
}

ClassCorridorGenerator::~ClassCorridorGenerator()
{
}

void ClassCorridorGenerator::set_points_data(const ClassCustomPathContainer &data)
{
    m_waypoints_ = data;
}

bool ClassCorridorGenerator::set_map_data(
    const int map_width_grid,
    const int map_height_grid,
    vector<int8_t> &map,
    const double grid_resolution,
    const int8_t plan_threshold)
{
    if (!m_gridmap_handler_.set_grid_map_ptr(&map))
        return false;
    if (!m_gridmap_handler_.set_grid_width_height(map_width_grid, map_height_grid))
        return false;
    if (!m_gridmap_handler_.set_planning_obstacle_threshold(plan_threshold))
        return false;
    if (!m_gridmap_handler_.set_grid_meter_ratio(grid_resolution))
        return false;
}

bool ClassCorridorGenerator::generate_bboxs()
{
    if (m_waypoints_.number_of_points() < 1)
    {
        return false;
    }

    m_result_as_corridor.clear_content();
    m_result_as_vector_.clear();

    unsigned int _number_grid_width_half = m_max_bbox_half_size_meter_ / m_gridmap_handler_.get_resolution();
    // cout << "_number_grid_width_half  " << _number_grid_width_half << endl;

    for (auto pt : m_waypoints_.get_path())
    {
        double _x = pt[0];
        double _y = pt[1];
        StructPoseGrid _center_grid;
        m_gridmap_handler_.convert_fine_pose_to_grid(StructPoseReal(_x, _y, 0), _center_grid);
        if (!m_gridmap_handler_.check_grid_within_map(_center_grid.x, _center_grid.y))
            return false;
        if (!m_gridmap_handler_.check_grid_clear(_center_grid.x, _center_grid.y))
            return false;

        // cout << "waypoint okay." << _x << "  " << _y << endl;

        int _x_max = _center_grid.x;
        int _x_min = _center_grid.x;
        int _y_max = _center_grid.y;
        int _y_min = _center_grid.y;

        bool _x_max_done = false;
        bool _x_min_done = false;
        bool _y_max_done = false;
        bool _y_min_done = false;

        for (int stepct = 0; stepct < _number_grid_width_half; stepct++)
        {
            
            if (!_y_min_done)
            {
                _y_min --;
                if (!(m_gridmap_handler_.check_grid_within_map(_x_min, _y_min) || m_gridmap_handler_.check_grid_within_map(_x_max, _y_min)))
                {
                    _y_min ++;
                    _y_min_done = true;
                }
                for (int xcheck = _x_min; xcheck <= _x_max; xcheck ++)
                {
                    if (!m_gridmap_handler_.check_grid_clear(xcheck, _y_min))
                    {
                        _y_min ++;
                        _y_min_done = true;
                        break;
                    }
                }
            }
            if (!_x_min_done)
            {
                _x_min --;
                if (!(m_gridmap_handler_.check_grid_within_map(_x_min, _y_max) || m_gridmap_handler_.check_grid_within_map(_x_min, _y_min)))
                {
                    _x_min ++;
                    _x_min_done = true;
                }
                for (int ycheck = _y_min; ycheck <= _y_max; ycheck ++)
                {
                    if (!m_gridmap_handler_.check_grid_clear(_x_min, ycheck))
                    {
                        _x_min ++;
                        _x_min_done = true;
                        break;
                    }
                }
            }
            
        }
        for (int stepct = 0; stepct < _number_grid_width_half+1; stepct++)
        {
            if (!_x_max_done)
            {
                _x_max ++;
                if (!(m_gridmap_handler_.check_grid_within_map(_x_max, _y_max) || m_gridmap_handler_.check_grid_within_map(_x_max, _y_min)))
                {
                    _x_max --;
                    _x_max_done = true;
                }
                for (int ycheck = _y_min; ycheck <= _y_max; ycheck ++)
                {
                    if (!m_gridmap_handler_.check_grid_clear(_x_max, ycheck))
                    {
                        _x_max --;
                        _x_max_done = true;
                        break;
                    }
                }
            }
            
            if (!_y_max_done)
            {
                _y_max ++;
                if (!(m_gridmap_handler_.check_grid_within_map(_x_min, _y_max) || m_gridmap_handler_.check_grid_within_map(_x_max, _y_max)))
                {
                    _y_max --;
                    _y_max_done = true;
                }
                for (int xcheck = _x_min; xcheck <= _x_max; xcheck ++)
                {
                    if (!m_gridmap_handler_.check_grid_clear(xcheck, _y_max))
                    {
                        _y_max --;
                        _y_max_done = true;
                        break;
                    }
                }
            }
        }
        double _x_max_metric, _x_min_metric, _y_max_metric, _y_min_metric;
        m_gridmap_handler_.convert_grid_pose_to_fine(_x_min, _y_min, _x_min_metric, _y_min_metric);
        m_gridmap_handler_.convert_grid_pose_to_fine(_x_max, _y_max, _x_max_metric, _y_max_metric);

        // cout << "corridor " << _x_min_metric << " " << _x_max_metric << " " << _y_min_metric << " " << _y_max_metric << endl;

        m_result_as_corridor.add_one_box(_x_min_metric, _x_max_metric, _y_min_metric, _y_max_metric);
        
        m_result_as_vector_.push_back(_x_min_metric);
        m_result_as_vector_.push_back(_x_max_metric);
        m_result_as_vector_.push_back(_y_min_metric);
        m_result_as_vector_.push_back(_y_max_metric);
        
    }
}


void ClassCorridorGenerator::get_result(ClassCorridor& cont)
{
    cont = m_result_as_corridor;
}

void ClassCorridorGenerator::get_result_as_vector(std::vector<double> cont)
{
    cont = m_result_as_vector_;
}


#endif