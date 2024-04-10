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
 * @file class_gridmap_handler.h
 * @author Mingjie
 * @brief This class is for handling all operations on gridmap.
 * @version 0.2
 * @date 2023-11-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef CLASS_GRIDMAP_HANDLER
#define CLASS_GRIDMAP_HANDLER

#include "common_includes.h"

#include "custom_data_types.h"


namespace hawa
{

/**
 * @brief The class for storing gridmap and perform actions about it.
 * Remeber to set the required parameters !!
 */
class ClassGridMapHandler
{
private:
    std::vector<int8_t> *m_ptr_grid_map_1D_;

    int m_grid_map_width_, m_grid_map_height_;
    bool m_FLAG_valid_width_is_set_, m_FLAG_valid_height_is_set_;

    int8_t m_obstacle_threshold_value_;

    double m_ratio_fine_to_grid_xy_;  // convert meter to grid_index
    double m_ratio_fine_to_grid_yaw_; // convert radian to grid_index

    double m_origin_offset_x_, m_origin_offset_y_;

public:
    ClassGridMapHandler();
    ~ClassGridMapHandler();

    int m_number_of_angle_layers_;

    int convert2DTo1D(const int x, const int y);
    int convert3DTo1D(const int x, const int y,  const int yaw);
    
    bool convertFinePoseToGrid(double fine_x, double fine_y, double fine_yaw, 
                               int &r_grid_x, int &r_grid_y, int &r_grid_yaw);

    bool convertFinePoseToGrid(std::array<double, 3> fine_xyyaw, 
                               int &r_grid_x, int &r_grid_y, int &r_grid_yaw);

    bool convertFinePoseToGrid(double fine_x, double fine_y, 
                               int &r_grid_x, int &r_grid_y);

    bool convertGridPoseToFine(int grid_x, int grid_y, 
                               double &r_fine_x, double &r_fine_y);

    bool convertFinePoseToGrid(const StructPoseReal realpose, StructPoseGrid& r_gridpose);
    
    bool checkGridWithinMap(int x, int y);

    bool checkPoseWithinMap(double x, double y);
    bool checkGridClear(int x, int y);
    
    bool setGridWidthHeight(int width, int height);
    bool setPlanningObstacleThreshold(int8_t thd);
    bool setValidateObstacleThreshold(int8_t thd);
    bool setGridMapPtr(std::vector<int8_t> * ptr_);
    bool setGridMeterRatio(double xy, double angle);

    void setOriginOffset(double x, double y);
    void getOriginOffset(double &x, double &y);

    int getMapLength();
};


} // namespace hawa


#endif