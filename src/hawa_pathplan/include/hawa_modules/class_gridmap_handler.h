#ifndef CLASS_GRIDMAP_HANDLER
#define CLASS_GRIDMAP_HANDLER

/**
 * @file class_gridmap_handler.h
 * @author Mingjie
 * @brief This is a class for handling all operations on gridmap.
 * @version 0.1
 * @date 2023-01-17
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <iostream>
#include <queue>
#include <unordered_map>
#include <map>
#include <math.h>

#include <algorithm>
#include <set>
#include <chrono>

// #include "array_hasher.h"

// #include "struct_simple_pose.h"

#include "../utils/hawa_data_containers.h"

using std::array;
using std::cerr;
using std::cout;
using std::endl;
using std::string;
using std::vector;

using std::chrono::high_resolution_clock;

/**
 * @brief The class for storing gridmap and perform actions about it.
 * Remeber to set the requied parameters !!
 */
class ClassGridMapHandler
{
private:
    vector<int8_t> *grid_map_1D_ptr_;

    int grid_map_width_, grid_map_height_;
    bool valid_width_is_set_, valid_height_is_set_;

    int8_t planning_obstacle_threshold_value_;
    int8_t validate_obstacle_threshold_value_;

    double K_ratio_fine_to_grid_xy_;  // convert meter to grid_index
    double K_ratio_fine_to_grid_yaw_; // convert radian to grid_index

public:
    ClassGridMapHandler();
    ~ClassGridMapHandler();

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
    int convert_3D_to_oneD(const int x, const int y,  const int yaw);
    bool convert_fine_pose_to_grid(double fine_x, double fine_y, double fine_yaw, int &grid_x, int &grid_y, int &grid_yaw);
    bool convert_fine_pose_to_grid(array<double, 3> fine_xyyaw, int &grid_x, int &grid_y, int &grid_yaw);
    bool convert_fine_pose_to_grid(double fine_x, double fine_y, int &grid_x, int &grid_y);
    bool convert_grid_pose_to_fine(int grid_x, int grid_y, double &fine_x, double &fine_y);
    bool convert_fine_pose_to_grid(const StructPoseReal realpose, StructPoseGrid& gridpose);
    bool check_grid_within_map(int x, int y);
    bool check_pose_within_map(double x, double y);
    bool check_grid_clear(int x, int y, EnumMode m);
    // bool check_pose_clear(double x, double y, EnumMode m);
    bool check_line_clear__by_grid(int x1, int y1, int x2, int y2, EnumMode m, EnumCollosionCheckType co);
    bool check_line_clear__by_meter(double x1, double y1, double x2, double y2, EnumMode m, EnumCollosionCheckType co);
    bool set_grid_width_height(int width, int height);
    bool set_planning_obstacle_threshold(int8_t thd);
    bool set_validate_obstacle_threshold(int8_t thd);
    bool set_grid_map_ptr(vector<int8_t> * ptr_);
    bool set_grid_meter_ratio(double xy, double angle);

    int getMapLength();
};

/**
 * @brief Construct a new Class Grid Map Handler:: Class Grid Map Handler object
 *
 */
ClassGridMapHandler::ClassGridMapHandler()
{
    valid_width_is_set_ = false;
    valid_height_is_set_ = false;

    K_ratio_fine_to_grid_xy_ = 0.1;
    K_ratio_fine_to_grid_yaw_ = M_PI / 9.0;

    m_number_of_angle_layers_ = int(M_PI*2.0/K_ratio_fine_to_grid_yaw_)+1;
}

/**
 * @brief Destroy the Class Grid Map Handler:: Class Grid Map Handler object
 *
 */
ClassGridMapHandler::~ClassGridMapHandler()
{
}

/**
 * @brief 
 * 
 * @param ptr_ 
 */
bool ClassGridMapHandler::set_grid_map_ptr(vector<int8_t> * ptr_)
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

int ClassGridMapHandler::getMapLength()
{
    return grid_map_1D_ptr_->size();
}


/**
 * @brief Because the map is saved in 1D vector, its width and height need to saved.
 *
 * @param width
 * @param height
 */
bool ClassGridMapHandler::set_grid_width_height(int width, int height)
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
bool ClassGridMapHandler::set_grid_meter_ratio(double xy, double angle)
{
    K_ratio_fine_to_grid_xy_ = xy;
    K_ratio_fine_to_grid_yaw_= angle;
    m_number_of_angle_layers_ = int(M_PI*2.0/K_ratio_fine_to_grid_yaw_)+1;
    return true;
}


/**
 * @brief   set the value of obstacle_threshold for validating mode.
 * @param thd   the threshold you want. Range:(0,100). Higher = Stricter.
 */
bool ClassGridMapHandler::set_validate_obstacle_threshold(int8_t thd)
{
    if (thd < 0)
    {
        thd = 0;
        cerr << "function: <ClassGridMapHandler::set_validate_obstacle_threshold>\nGiven value smaller than 0. Set as 0." << endl;
        return false;
    }
    else if (thd > 100)
    {
        thd = 100;
        cerr << "function: <ClassGridMapHandler::set_validate_obstacle_threshold>\nGiven value larger than 100. Set as 100." << endl;
        return false;
    }
    validate_obstacle_threshold_value_ = thd;
    return true;
}

/**
 * @brief    set the value of obstacle_threshold for plan mode.
 * @param thd   the threshold you want. Range:(0,100). Higher = Stricter.
 */
bool ClassGridMapHandler::set_planning_obstacle_threshold(int8_t thd)
{
    if (thd < 0)
    {
        thd = 0;
        cerr << "function: <ClassGridMapHandler::set_planning_obstacle_threshold>\nGiven value smaller than 0. Set as 0." << endl;
        return false;
    }
    else if (thd > 100)
    {
        thd = 100;
        cerr << "function: <ClassGridMapHandler::set_planning_obstacle_threshold>\nGiven value larger than 100. Set as 100." << endl;
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
 * @param m , EnumMode. The threshold value is different in plan mode and validate mode.
 * @return true , if it's clear.
 * @return false , if it's obstacle or error happened.
 */
bool ClassGridMapHandler::check_grid_clear(int x, int y, EnumMode m)
{
    int8_t threshold;
    if (m == EnumMode::plan)
        threshold = planning_obstacle_threshold_value_;
    if (m == EnumMode::validate)
        threshold = validate_obstacle_threshold_value_;

    int index_1d = convert_twoD_to_oneD(x, y);

    // if ( std::abs(rs_path->path_steps[pointi][0] - rs_path->path_steps[pointi-1][0]) +
    //         std::abs(rs_path->path_steps[pointi][1] - rs_path->path_steps[pointi-1][1])  > 0.4  ){
    //         continue;
    //         }

    if (0 <= index_1d && index_1d < (grid_map_1D_ptr_->size()))
    {
        if ((*grid_map_1D_ptr_)[index_1d] > threshold)
        {
            return false;
        }
        return true;
    }
    else
    {
        cerr << "function <ClassGridMapHandler::check_grid_clear>\nindex_1d out of map size." << endl;
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

// /**
//  * @brief
//  * check if a pose if clear from obstacle collision.
//  * @param x , meter.
//  * @param y , meter.
//  * @param m , mode: plan, validate.
//  * @return true
//  * @return false
//  */
// bool ClassGridMapHandler::check_pose_clear(double x, double y, EnumMode m)
// {
//     int grid_x, grid_y;
//     convert_fine_pose_to_grid(x, y, grid_x, grid_y);
//     return check_grid_clear(grid_x, grid_y, m);
// }

/**
 * @brief
 *
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @param m
 * @return true
 * @return false
 */
bool ClassGridMapHandler::check_line_clear__by_grid(int x1, int y1, int x2, int y2, EnumMode m, EnumCollosionCheckType co)
{
    if (co == EnumCollosionCheckType::end_points_only)
    {
        return check_grid_clear(x1, y1, m) && check_grid_clear(x2, y2, m);
    }
    else if (co == EnumCollosionCheckType::all_grids)
    {
        // use bresenham to find the grids in the middle, and check each of them.
        return check_grid_clear(x1, y1, m) && check_grid_clear(x2, y2, m);
    }
    else if (co == EnumCollosionCheckType::spaced)
    {
        // similar to above, but check only some grids.
        return check_grid_clear(x1, y1, m) && check_grid_clear(x2, y2, m);
    }
    else
    {
        return check_grid_clear(x1, y1, m) && check_grid_clear(x2, y2, m);
    }
}

/**
 * @brief
 *
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @param m
 * @return true
 * @return false
 */
bool ClassGridMapHandler::check_line_clear__by_meter(double x1, double y1, double x2, double y2, EnumMode m, EnumCollosionCheckType co)
{
    int x1_grid, y1_grid, x2_grid, y2_grid;
    convert_fine_pose_to_grid(x1, y1, x1_grid, y1_grid);
    convert_fine_pose_to_grid(x2, y2, x2_grid, y2_grid);
    return check_line_clear__by_grid(x1_grid, y1_grid, x2_grid, y2_grid, m, co);
}

/**
 * @brief
 * convert pose (x,y,yaw) in meter & radian to grid.
 * @return true , if the values look okay.
 * @return false , if the values look unreasonably large/small.
 */
bool ClassGridMapHandler::convert_fine_pose_to_grid(double fine_x, double fine_y, double fine_yaw, int &grid_x, int &grid_y, int &grid_yaw)
{
    if (K_ratio_fine_to_grid_xy_ > 0.001 && K_ratio_fine_to_grid_yaw_ > 0.001)
    {
        grid_x = int(fine_x / K_ratio_fine_to_grid_xy_);
        grid_y = int(fine_y / K_ratio_fine_to_grid_xy_);
        grid_yaw = int(fine_yaw / K_ratio_fine_to_grid_yaw_);
        return true;
    }
    else
    {
        cout << "K_ratio_fine_to_grid_ seems too small to be reasonable: " << K_ratio_fine_to_grid_xy_ << " , " << K_ratio_fine_to_grid_yaw_ << endl;
        return false;
    }
}

/**
 * @brief
 * convert pose (x,y,yaw) in meter & radian to grid.
 * @return true , if the values look okay.
 * @return false , if the values look unreasonably large/small.
 */
bool ClassGridMapHandler::convert_fine_pose_to_grid(array<double, 3> fine_xyyaw, int &grid_x, int &grid_y, int &grid_yaw)
{
    return convert_fine_pose_to_grid(fine_xyyaw[0], fine_xyyaw[1], fine_xyyaw[2], grid_x, grid_y, grid_yaw);
}

/**
 * @brief 
 * 
 * @param realpose 
 * @param gridpose 
 * @return true 
 * @return false 
 */
bool ClassGridMapHandler::convert_fine_pose_to_grid(const StructPoseReal realpose, StructPoseGrid& gridpose)
{
    // cout << K_ratio_fine_to_grid_xy_ << endl;

    if (K_ratio_fine_to_grid_xy_ > 0.001 && K_ratio_fine_to_grid_yaw_ > 0.001)
    {
        gridpose.x = int(realpose.x / K_ratio_fine_to_grid_xy_);
        gridpose.y = int(realpose.y / K_ratio_fine_to_grid_xy_);
        gridpose.yaw = int(realpose.yaw / K_ratio_fine_to_grid_yaw_);
        return true;
    }
    else
    {
        cout << "K_ratio_fine_to_grid_ seems too small to be reasonable: " << K_ratio_fine_to_grid_xy_ << " , " << K_ratio_fine_to_grid_yaw_ << endl;
        return false;
    }
}

/**
 * @brief
 * convert pose (x,y,yaw) in meter & radian to grid.
 * @return true , if the values look okay.
 * @return false , if the values look unreasonably large/small.
 */
bool ClassGridMapHandler::convert_fine_pose_to_grid(double fine_x, double fine_y, int &grid_x, int &grid_y)
{
    int dummy;
    return convert_fine_pose_to_grid(fine_x, fine_y, 0, grid_x, grid_y, dummy);
}

/**
 * @brief check if a pose in inside of grid map. The pose is converted to grid actually.
 *
 * @param x
 * @param y
 * @return true
 * @return false
 */
inline bool ClassGridMapHandler::check_pose_within_map(double x, double y)
{
    int grid_x, grid_y;
    convert_fine_pose_to_grid(x, y, grid_x, grid_y);
    return check_grid_within_map(grid_x, grid_y);
}


inline bool ClassGridMapHandler::convert_grid_pose_to_fine(int grid_x, int grid_y, double &fine_x, double &fine_y)
{
    fine_x = grid_x * K_ratio_fine_to_grid_xy_;
    fine_y = grid_y * K_ratio_fine_to_grid_xy_;

    return true;
}

/**
 * @brief
 * The grid is 2D in logic, but the grid_map is saved in 1D vector.
 * So need this function to convert 2D grid (x,y) into 1D index.
 * @param x
 * @param y
 * @return int
 */
inline int ClassGridMapHandler::convert_twoD_to_oneD(const int x, const int y)
{
    return y * grid_map_width_ + x;
}

/**
 * @brief
 * The grid is 2D in logic, but the grid_map is saved in 1D vector.
 * So need this function to convert 2D grid (x,y) into 1D index.
 * @param x
 * @param y
 * @return int
 */
inline int ClassGridMapHandler::convert_3D_to_oneD(const int x, const int y,  const int yaw)
{
    return yaw * grid_map_1D_ptr_->size() + y * grid_map_width_ + x;
}

/**
 * @brief   check if a grid in inside of grid map
 * @param x     int
 * @param y     int
 * @return true , if inside or on the edges.
 * @return false , if outside.
 */
inline bool ClassGridMapHandler::check_grid_within_map(int x, int y)
{
    if ( x < 0 ){
        return false;
    }
    else if (x >= grid_map_width_)
    {
        return false;
    }
    else if (y < 0)
    {
        return false;
    }
    else if (y >= grid_map_height_)
    {
        return false;
    }
    return true;
    // if (x >= 0 && x < grid_map_width_)
    // {
    //     if (y >= 0 && y < grid_map_height_)
    //     {
    //         return true;
    //     }
    // }
    // return false;
}

#endif