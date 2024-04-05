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
 * @file class_path_validator.h
 * @author Mingjie
 * @brief
 * @version 0.1
 * @date 2023-01-18
 *
 * @copyright Copyright (c) 2023
 *
 */

/**
 * @note This feature is not used and it is still in developemnt. 2023-11-22.
*/

#ifndef CLASS_PATH_VALIDATOR
#define CLASS_PATH_VALIDATOR

#include "common_includes.h"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/path.hpp"

#include "class_gridmap_handler.h"
#include "hybrid_astar_tools.h"


namespace hawa
{
    
/**
 * @brief A class for verify if the given path is valid or not.
 *
 */
class ClassPathValidator
{
private:

    struct 
    {
        double x = 0;
        double y = 0;
        double yaw = 0;
    } m_robot_pose_;

    nav_msgs::msg::Path::SharedPtr m_path_ptr_;

    ClassGridMapHandler m_gridmap_handler_;
    MapOccThreshold m_map_occ_threshold_;
    

    double m_distance_tolerance_;
    double m_angle_tolerance_;

    static const double k_pi2_;

    bool checkPathIsClear();
    bool checkRobotNearPath();

    double mod_2pi(double angle);

    double euclidean_distance(double x1, double y1, double x2, double y2);
    double min_angle_diffrence(double a1, double a2);

    double quaternionToYaw(const geometry_msgs::msg::Quaternion& q_msg);


public:
    ClassPathValidator();
    ~ClassPathValidator();

    double m_yaw_angle_bin_size_;
    
    void setRobotPose(const geometry_msgs::msg::TransformStamped& robot_pose);
    void setPath(const nav_msgs::msg::Path& path);
    void setMap(nav_msgs::msg::OccupancyGrid * ptr_map);

    // void setup_para(double distance_tole, double angle_tole);
    
    bool validate();
};

}



#endif
