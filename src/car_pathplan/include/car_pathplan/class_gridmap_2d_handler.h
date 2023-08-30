#ifndef CLASS_GRIDMAP_2D_HANDLER
#define CLASS_GRIDMAP_2D_HANDLER

/**
 * @file class_gridmap_2d_handler.h
 * @author Mingjie
 * @brief This is a class for handling all operations on gridmap.
 * @version 0.1
 * @date 2023-06-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <iostream>
#include <vector>
#include <array>
#include <math.h>

#include "struct_simple_pose.h"

using std::array;
using std::cerr;
using std::cout;
using std::endl;
using std::string;
using std::vector;

/**
 * @brief The class for storing gridmap and perform actions about it.
 * Remeber to set the requied parameters !!
 */
class ClassGridMap2DHandler
{
private:
    vector<int8_t> *grid_map_1D_ptr_;

    int grid_map_width_, grid_map_height_;
    bool valid_width_is_set_, valid_height_is_set_;

    int8_t planning_obstacle_threshold_value_;
    // int8_t validate_obstacle_threshold_value_;

    double K_ratio_fine_to_grid_xy_;  // convert meter to grid_index

public:
    ClassGridMap2DHandler();
    ~ClassGridMap2DHandler();

    enum EnumMode
    {
        plan,
        validate
    }enum_mode_;

    enum EnumCollosionCheckType
    {
        end_points_only,
        all_grids,
        spaced
    }enum_collosion_type_;

    int m_number_of_angle_layers_;

    int convert_twoD_to_oneD(const int x, const int y);
    
    bool convert_fine_pose_to_grid(double fine_x, double fine_y, int &grid_x, int &grid_y);
    
    bool convert_fine_pose_to_grid(const StructPoseReal realpose, StructPoseGrid& gridpose);

    bool convert_grid_pose_to_fine(int grid_x, int grid_y, double &fine_x, double &fine_y);


    bool check_grid_within_map(int x, int y);
    
    bool check_grid_clear(int x, int y);
    
    bool set_grid_width_height(int width, int height);

    bool set_planning_obstacle_threshold(int8_t thd);
    
    bool set_grid_map_ptr(vector<int8_t> * ptr_);
    bool set_grid_meter_ratio(double xy);

    double get_resolution();

    size_t get_map_size();
};


/**
 * @brief Construct a new Class Grid Map Handler:: Class Grid Map Handler object
 *
 */
ClassGridMap2DHandler::ClassGridMap2DHandler()
{
    valid_width_is_set_ = false;
    valid_height_is_set_ = false;

    K_ratio_fine_to_grid_xy_ = 0.1;
    // K_ratio_fine_to_grid_yaw_ = M_PI / 9.0;

    // m_number_of_angle_layers_ = int(M_PI*2.0/K_ratio_fine_to_grid_yaw_)+1;
}


/**
 * @brief Destroy the Class Grid Map Handler:: Class Grid Map Handler object
 *
 */
ClassGridMap2DHandler::~ClassGridMap2DHandler()
{
}


/**
 * @brief 
 * 
 * @param ptr_ 
 */
bool ClassGridMap2DHandler::set_grid_map_ptr(vector<int8_t> * ptr_)
{
    if(ptr_->size() > 1)
    {
        grid_map_1D_ptr_ = ptr_;
        return true;
    }
    else{
        return false;
    }
    
}


/**
 * @brief Because the map is saved in 1D vector, its width and height need to saved.
 *
 * @param width
 * @param height
 */
bool ClassGridMap2DHandler::set_grid_width_height(int width, int height)
{
    if (width > 1 && width < 123456)
    {
        grid_map_width_ = width;
        valid_width_is_set_ = true;
    }
    else
    {
        valid_width_is_set_ = false;
        cerr << "Invalid width is assigned: " << width << endl;
    }
    if (height > 1 && height < 123456)
    {
        grid_map_height_ = height;
        valid_height_is_set_ = true;
    }
    else
    {
        valid_height_is_set_ = false;
        cerr << "Invalid height is assigned: " << height << endl;
    }
    if( valid_height_is_set_ && valid_width_is_set_)
    {
        return true;
    }
    else
    {
        return false;
    }
}


/**
 * @brief 
 * 
 * @param xy 
 * @param angle 
 * @return true 
 * @return false 
 */
bool ClassGridMap2DHandler::set_grid_meter_ratio(double xy)
{
    K_ratio_fine_to_grid_xy_ = xy;

    return true;
}


/**
 * @brief    set the value of obstacle_threshold for plan mode.
 * @param thd   the threshold you want. Range:(0,100). Higher = Stricter.
 */
bool ClassGridMap2DHandler::set_planning_obstacle_threshold(int8_t thd)
{
    if (thd < 0)
    {
        thd = 0;
        cerr << "function: <ClassGridMap2DHandler::set_planning_obstacle_threshold>\nGiven value smaller than 0. Set as 0." << endl;
        return false;
    }
    else if (thd > 100)
    {
        thd = 100;
        cerr << "function: <ClassGridMap2DHandler::set_planning_obstacle_threshold>\nGiven value larger than 100. Set as 100." << endl;
        return false;
    }
    planning_obstacle_threshold_value_ = thd;
    return true;
}


/**
 * @brief
 * check if a grid is clear or it's obstacle.
 * @param x , int. grid index.
 * @param y , int. grid index.
 * @return true , if it's clear.
 * @return false , if it's obstacle or error happened.
 */
bool ClassGridMap2DHandler::check_grid_clear(int x, int y)
{
    int index_1d = convert_twoD_to_oneD(x, y);

    if (0 <= index_1d && index_1d < (grid_map_1D_ptr_->size()))
    {
        if ((*grid_map_1D_ptr_)[index_1d] > planning_obstacle_threshold_value_)
        {
            return false;
        }
        return true;
    }
    else
    {
        cerr << "function <ClassGridMap2DHandler::check_grid_clear>\nindex_1d out of map size." << endl;
        try
        {
            cerr << index_1d << " in " << grid_map_1D_ptr_->size() << " xy:" << x << " " << y << endl;
        }
        catch (...)
        {
            cout << "try failed." << endl;
        }
        return false;
    }
}


/**
 * @brief   check if a grid in inside of grid map
 * @param x     int
 * @param y     int
 * @return true , if inside or on the edges.
 * @return false , if outside.
 */
inline bool ClassGridMap2DHandler::check_grid_within_map(int x, int y)
{
    if ( x < 0 || x >= grid_map_width_ || y < 0 || y >= grid_map_height_)
    {
        return false;
    }
    return true;
}


/**
 * @brief
 * convert pose (x,y,yaw) in meter & radian to grid.
 * @return true , if the values look okay.
 * @return false , if the values look unreasonably large/small.
 */
bool ClassGridMap2DHandler::convert_fine_pose_to_grid(double fine_x, double fine_y, int &grid_x, int &grid_y)
{
    grid_x = int(fine_x / K_ratio_fine_to_grid_xy_);
    grid_y = int(fine_y / K_ratio_fine_to_grid_xy_);
    return true;
}


/**
 * @brief 
 * 
 * @param realpose 
 * @param gridpose 
 * @return true 
 * @return false 
 */
bool ClassGridMap2DHandler::convert_fine_pose_to_grid(const StructPoseReal realpose, StructPoseGrid& gridpose)
{
    gridpose.x = int(realpose.x / K_ratio_fine_to_grid_xy_);
    gridpose.y = int(realpose.y / K_ratio_fine_to_grid_xy_);
    return true;
}



inline bool ClassGridMap2DHandler::convert_grid_pose_to_fine(int grid_x, int grid_y, double &fine_x, double &fine_y)
{
    fine_x = grid_x * K_ratio_fine_to_grid_xy_;
    fine_y = grid_y * K_ratio_fine_to_grid_xy_;
}

/**
 * @brief
 * The grid is 2D in logic, but the grid_map is saved in 1D vector.
 * So need this function to convert 2D grid (x,y) into 1D index.
 * @param x
 * @param y
 * @return int
 */
inline int ClassGridMap2DHandler::convert_twoD_to_oneD(const int x, const int y)
{
    return y * grid_map_width_ + x;
}


inline size_t ClassGridMap2DHandler::get_map_size()
{
    return grid_map_1D_ptr_->size();
}


inline double ClassGridMap2DHandler::get_resolution()
{
    return K_ratio_fine_to_grid_xy_;
}

#endif