
#include "class_gridmap_handler.h"


namespace hawa
{

/**
 * @brief Construct a new Class Grid Map Handler:: Class Grid Map Handler object
 *
 */
ClassGridMapHandler::ClassGridMapHandler()
{
    m_FLAG_valid_width_is_set_ = false;
    m_FLAG_valid_height_is_set_ = false;

    m_ratio_fine_to_grid_xy_ = 0.1;
    m_ratio_fine_to_grid_yaw_ = M_PI / 9.0;

    m_number_of_angle_layers_ = int(M_PI*2.0/m_ratio_fine_to_grid_yaw_)+1;
}

/**
 * @brief Destroy the Class Grid Map Handler:: Class Grid Map Handler object
 */
ClassGridMapHandler::~ClassGridMapHandler()
{
    delete m_ptr_grid_map_1D_;
}

/**
 * @brief Assign the map data by giving the pointer. 
 * @param ptr_map Pointer to the 1d vector. 
 */
bool ClassGridMapHandler::setGridMapPtr(std::vector<int8_t> * ptr_map)
{
    if(ptr_map->size() > 1)
    {
        m_ptr_grid_map_1D_ = ptr_map;
        return true;
    }
    else{
        return false;
    }
}

/**
 * @brief Get the total number of grids in the map.
 * @return The size.
*/
int ClassGridMapHandler::getMapLength()
{
    return m_ptr_grid_map_1D_->size();
}


/**
 * @brief Because the map is saved in 1D vector, its width and height are saved separately..
 * @param width The width of the gridmap.
 * @param height The height of the gridmap.
 * @return False if any value out of regular ranges. True if nothing wrong.
 */
bool ClassGridMapHandler::setGridWidthHeight(int width, int height)
{
    if (width > 1 && width < 123456)
    {
        m_grid_map_width_ = width;
        m_FLAG_valid_width_is_set_ = true;
    }
    else
    {
        m_FLAG_valid_width_is_set_ = false;
        RCLCPP_INFO(rclcpp::get_logger("ClassGridMapHandler"), "Invalid width is assigned: %d", width);
        // ROS_WARN_STREAM_THROTTLE(20, "Invalid width is assigned: " << width);
    }
    if (height > 1 && height < 123456)
    {
        m_grid_map_height_ = height;
        m_FLAG_valid_height_is_set_ = true;
    }
    else
    {
        m_FLAG_valid_height_is_set_ = false;
        RCLCPP_INFO(rclcpp::get_logger("ClassGridMapHandler"), "Invalid height is assigned: %d", height);
        // ROS_WARN_STREAM_THROTTLE(20, "Invalid height is assigned: " << height);
    }
    if( m_FLAG_valid_height_is_set_ && m_FLAG_valid_width_is_set_)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief Set the ratio between the metric dimension and grid dimension.
 * @param xy The ratio in x and y directions. 
 * @param angle The ratio used by converting the angles/yaw.
 * @return Return is not used.
 */
bool ClassGridMapHandler::setGridMeterRatio(double xy, double angle)
{
    m_ratio_fine_to_grid_xy_ = xy;
    m_ratio_fine_to_grid_yaw_= angle;
    m_number_of_angle_layers_ = int(M_PI*2.0/m_ratio_fine_to_grid_yaw_)+1;
    return true;
}


/**
 * @brief Set the value of obstacle_threshold for validating mode.
 * @param thd The threshold you want. Range:(0,100). Higher = Stricter.
 */
bool ClassGridMapHandler::setValidateObstacleThreshold(int8_t thd)
{
    if (thd < 0)
    {
        thd = 0;
        RCLCPP_INFO(rclcpp::get_logger("ClassGridMapHandler"), "ClassGridMapHandler::setValidateObstacleThreshold() value smaller than 0");
        // ROS_WARN_STREAM_THROTTLE(20, "ClassGridMapHandler::setValidateObstacleThreshold() value smaller than 0");
        return false;
    }
    else if (thd > 100)
    {
        thd = 100;
        RCLCPP_INFO(rclcpp::get_logger("ClassGridMapHandler"), "ClassGridMapHandler::setValidateObstacleThreshold() value larger than 100");
        // ROS_WARN_STREAM_THROTTLE(20, "ClassGridMapHandler::setValidateObstacleThreshold() value larger than 100");
        return false;
    }
    m_validate_obstacle_threshold_value_ = thd;
    return true;
}

/**
 * @brief Set the value of obstacle_threshold for plan mode.
 * @param thd The threshold you want. Range:(0,100). Higher = Stricter.
 */
bool ClassGridMapHandler::setPlanningObstacleThreshold(int8_t thd)
{
    if (thd < 0)
    {
        thd = 0;
        RCLCPP_INFO(rclcpp::get_logger("ClassGridMapHandler"), "ClassGridMapHandler::setPlanningObstacleThreshold() value smaller than 0");
        // ROS_WARN_STREAM_THROTTLE(20, "ClassGridMapHandler::setPlanningObstacleThreshold() value smaller than 0");
        return false;
    }
    else if (thd > 100)
    {
        thd = 100;
        RCLCPP_INFO(rclcpp::get_logger("ClassGridMapHandler"), "ClassGridMapHandler::setPlanningObstacleThreshold() value larger than 100");
        // ROS_WARN_STREAM_THROTTLE(20, "ClassGridMapHandler::setPlanningObstacleThreshold() value larger than 100");
        return false;
    }
    m_planning_obstacle_threshold_value_ = thd;
    return true;
}

/**
 * @brief check if a grid is clear or it's obstacle.
 * @param x grid index.
 * @param y grid index.
 * @param mode EnumMode. The threshold value is different in plan mode and validate mode.
 * @return true , if it's clear.
 * @return false , if it's obstacle or error happened.
 */
bool ClassGridMapHandler::checkGridClear(int x, int y, EnumMode mode)
{
    int8_t _threshold;
    if (mode == EnumMode::plan)
        _threshold = m_planning_obstacle_threshold_value_;
    if (mode == EnumMode::validate)
        _threshold = m_validate_obstacle_threshold_value_;

    int _index_1d = convert2DTo1D(x, y);

    if (0 <= _index_1d && _index_1d < int(m_ptr_grid_map_1D_->size()))
    {
        if ((*m_ptr_grid_map_1D_).at(_index_1d) > _threshold)
        {
            return false;
        }
        return true;
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("ClassGridMapHandler"), "function <ClassGridMapHandler::checkGridClear>\nindex_1d out of map size.");
        // ROS_WARN_STREAM_THROTTLE(30, "function <ClassGridMapHandler::checkGridClear>\nindex_1d out of map size.");
        return false;
    }
}

/**
 * @brief Check if a line is clear from any obstacle collision. 
 * @param x1 X of end point 1.
 * @param y1 Y of end point 1.
 * @param x2 X of end point 2.
 * @param y2 Y of end point 2.
 * @param mode Decide which mode it should use. 
 * @param coll Decide which type of checking should use.
 * @return true if this line does not collide with any thing.
 */
bool ClassGridMapHandler::checkLineClearByGrid(int x1, int y1, 
                                               int x2, int y2, 
                                               EnumMode mode, EnumCollosionCheckType coll)
{
    if (coll == EnumCollosionCheckType::end_points_only)
    {
        return checkGridClear(x1, y1, mode) && checkGridClear(x2, y2, mode);
    }
    else if (coll == EnumCollosionCheckType::all_grids)
    {
        // use bresenham to find the grids in the middle, and check each of them.
        return checkGridClear(x1, y1, mode) && checkGridClear(x2, y2, mode);
    }
    else if (coll == EnumCollosionCheckType::spaced)
    {
        // similar to above, but check only some grids.
        return checkGridClear(x1, y1, mode) && checkGridClear(x2, y2, mode);
    }
    else
    {
        return checkGridClear(x1, y1, mode) && checkGridClear(x2, y2, mode);
    }
}

/**
 * @brief Check if a line is clear from any obstacle collision. 
 * @param x1 X of end point 1.
 * @param y1 Y of end point 1.
 * @param x2 X of end point 2.
 * @param y2 Y of end point 2.
 * @param mode Decide which mode it should use. 
 * @param coll Decide which type of checking should use.
 * @return true if this line does not collide with any thing.
 */
bool ClassGridMapHandler::checkLineClearByReal(double x1, double y1, 
                                               double x2, double y2, 
                                               EnumMode mode, EnumCollosionCheckType coll)
{
    int _x1_grid, _y1_grid, _x2_grid, _y2_grid;
    convertFinePoseToGrid(x1, y1, _x1_grid, _y1_grid);
    convertFinePoseToGrid(x2, y2, _x2_grid, _y2_grid);
    return checkLineClearByGrid(_x1_grid, _y1_grid, _x2_grid, _y2_grid, mode, coll);
}

/**
 * @brief convert metric pose (x,y,yaw) to grid.
 * @param fine_x The metric x.
 * @param fine_y The metric y.
 * @param fine_yaw The metric yaw.
 * @param r_grid_x Reference to the gridwise x.
 * @param r_grid_y Reference to the gridwise y.
 * @param r_grid_yaw Reference to the gridwise yaw.
 * @return true, if the values look okay. false, if the values look unreasonably large/small.
 */
bool ClassGridMapHandler::convertFinePoseToGrid(double fine_x, double fine_y, double fine_yaw, 
                                                int &r_grid_x, int &r_grid_y, int &r_grid_yaw)
{
    if (m_ratio_fine_to_grid_xy_ > 0.001 && m_ratio_fine_to_grid_yaw_ > 0.001)
    {
        r_grid_x = int(fine_x / m_ratio_fine_to_grid_xy_);
        r_grid_y = int(fine_y / m_ratio_fine_to_grid_xy_);
        r_grid_yaw = int(fine_yaw / m_ratio_fine_to_grid_yaw_);
        return true;
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("ClassGridMapHandler"), "K_ratio_fine_to_grid_ seems too small %f, %f", 
                                        m_ratio_fine_to_grid_xy_, m_ratio_fine_to_grid_yaw_);
        // ROS_WARN_STREAM_ONCE("K_ratio_fine_to_grid_ seems too small " 
        //                     <<  m_ratio_fine_to_grid_xy_ << " , " << m_ratio_fine_to_grid_yaw_);
        return false;
    }
}

/**
 * @brief convert metric pose (x,y,yaw) to grid.
 * @param fine_xyyaw The metric pose. The type is std array.
 * @param r_grid_x Reference to the gridwise x.
 * @param r_grid_y Reference to the gridwise y.
 * @param r_grid_yaw Reference to the gridwise yaw.
 */
bool ClassGridMapHandler::convertFinePoseToGrid(std::array<double, 3> fine_xyyaw, 
                                                int &r_grid_x, int &r_grid_y, int &r_grid_yaw)
{
    return convertFinePoseToGrid(fine_xyyaw[0], fine_xyyaw[1], fine_xyyaw[2], r_grid_x, r_grid_y, r_grid_yaw);
}

/**
 * @brief convert metric pose (x,y,yaw) to grid wise coordinate. The input and output are custom defined
 * types. 
 * @param realpose The metric pose.
 * @param gridpose The gridwise pose.
 */
bool ClassGridMapHandler::convertFinePoseToGrid(const StructPoseReal realpose, StructPoseGrid& r_gridpose)
{
    // cout << m_ratio_fine_to_grid_xy_ << endl;

    return convertFinePoseToGrid(realpose.x, realpose.y, realpose.yaw, r_gridpose.x, r_gridpose.y, r_gridpose.yaw);

    // if (m_ratio_fine_to_grid_xy_ > 0.001 && m_ratio_fine_to_grid_yaw_ > 0.001)
    // {
    //     gridpose.x = int(realpose.x / m_ratio_fine_to_grid_xy_);
    //     gridpose.y = int(realpose.y / m_ratio_fine_to_grid_xy_);
    //     gridpose.yaw = int(realpose.yaw / m_ratio_fine_to_grid_yaw_);
    //     return true;
    // }
    // else
    // {
    //     ROS_WARN_STREAM_ONCE("")
    //     cout << "K_ratio_fine_to_grid_ seems too small to be reasonable: " << m_ratio_fine_to_grid_xy_ << " , " << m_ratio_fine_to_grid_yaw_ << endl;
    //     return false;
    // }
}

/**
 * @brief convert pose (x,y,yaw) in meter & radian to grid wise coordinate. This version assumes that the yaw
 * is 0.
 * @param fine_x Metric x coordinate.
 * @param fine_y Metric y coordinate.
 * @param r_grid_x Grid wise x coordinate.
 * @param r_grid_y Grid wise x coordinate.
 * @return true, if the values look okay. false, if the values look unreasonably large/small.
 */
bool ClassGridMapHandler::convertFinePoseToGrid(double fine_x, double fine_y, int &r_grid_x, int &r_grid_y)
{
    int _dummy;
    return convertFinePoseToGrid(fine_x, fine_y, 0, r_grid_x, r_grid_y, _dummy);
}

/**
 * @brief check if a pose in inside of grid map. The pose is converted to grid-wise actually.
 * @param x The real world metric x coordinate.
 * @param y The real world metric y coordinate.
 * @return true, if inside or on the edges. false, if outside.
 */
inline bool ClassGridMapHandler::checkPoseWithinMap(double x, double y)
{
    int _grid_x, _grid_y;
    convertFinePoseToGrid(x, y, _grid_x, _grid_y);
    return checkGridWithinMap(_grid_x, _grid_y);
}

/**
 * @brief Convert the given grid pose to metric fine pose. 
 * @param grid_x grid wise x.
 * @param grid_y grid wise y.
 * @param r_fine_x reference to the metric x.
 * @param r_fine_y reference to the metric y.
 * @return The return is not used for now. 
*/
inline bool ClassGridMapHandler::convertGridPoseToFine(const int grid_x, const int grid_y, 
                                                       double &r_fine_x, double &r_fine_y)
{
    r_fine_x = grid_x * m_ratio_fine_to_grid_xy_;
    r_fine_y = grid_y * m_ratio_fine_to_grid_xy_;
    return true;
}

/**
 * @brief The grid is 3D in logic, but the grid_map is saved in 1D vector.
 * So need this function to convert 3D grid (x,y,yaw) into 1D index.
 * @param x The x in coordinate.
 * @param y The y in coordinate.
 * @param yaw The index of yaw in coordinate.
 * @return The index in 1D. 
 */
int ClassGridMapHandler::convert3DTo1D(const int x, const int y,  const int yaw)
{
    return yaw * m_ptr_grid_map_1D_->size() + y * m_grid_map_width_ + x;
}

/**
 * @brief Check if the given grid in inside of grid map.
 * @param x The x in coordinate.
 * @param y The y in coordinate.
 * @return true , if inside or on the edges. false , if outside.
 */
bool ClassGridMapHandler::checkGridWithinMap(int x, int y)
{
    if ( x < 0 )
        return false;
    
    else if (x >= m_grid_map_width_)
        return false;
    
    else if (y < 0)
        return false;
    
    else if (y >= m_grid_map_height_)
        return false;
    
    return true;
}

// /**
//  * @brief The grid is 3D in logic, but the grid_map is saved in 1D vector.
//  * So need this function to convert 3D grid (x,y,yaw) into 1D index.
//  * @param x The x in coordinate.
//  * @param y The y in coordinate.
//  * @param yaw The index of yaw in coordinate.
//  * @return The index in 1D. 
//  */
// inline int ClassGridMapHandler::convert3DTo1D(const int x, const int y,  const int yaw)
// {
//     return yaw * m_ptr_grid_map_1D_->size() + y * m_grid_map_width_ + x;
// }

// /**
//  * @brief Check if the given grid in inside of grid map.
//  * @param x The x in coordinate.
//  * @param y The y in coordinate.
//  * @return true , if inside or on the edges. false , if outside.
//  */
// inline bool ClassGridMapHandler::checkGridWithinMap(int x, int y)
// {
//     if ( x < 0 )
//         return false;
    
//     else if (x >= m_grid_map_width_)
//         return false;
    
//     else if (y < 0)
//         return false;
    
//     else if (y >= m_grid_map_height_)
//         return false;
    
//     return true;
// }

/**
 * @brief The grid is 2D in logic, but the grid_map is saved in 1D vector.
 * So need this function to convert 2D grid (x,y) into 1D index.
 * @param x The x in coordinate.
 * @param y The y in coordinate.
 * @return The index in 1D. 
 */
inline int ClassGridMapHandler::convert2DTo1D(const int x, const int y)
{
    return y * m_grid_map_width_ + x;
}


} // namespace hawa

