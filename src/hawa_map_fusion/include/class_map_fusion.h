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
 * @file class_map_fusion.h
 * @author Mingjie
 * @brief This class receives the raw gridmap and other range sensors, then add the sensor data into the 
 * gridmap and publish the result into a new topic. 
 * @version 0.2
 * @date 2023-11-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */



#ifndef HAWA_CLASS_MAP_FUSION_NODE_H
#define HAWA_CLASS_MAP_FUSION_NODE_H

#include <algorithm>
#include <future>
#include <mutex>

// #include <boost/property_tree/ptree.hpp>
// #include <boost/property_tree/json_parser.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


#include "class_inflation_samples.h"
#include "class_tool_timer.h"
#include "class_tool_general.h"



namespace hawa
{

using namespace std::chrono_literals;

/**
 * @brief This class receives the raw gridmap and other range sensors, then add the sensor data into the 
 * gridmap and publish the result into a new topic. 
*/
class ClassMapFusion : public rclcpp::Node
{

private:

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_suber_map_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_suber_scan_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr m_suber_depth_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_puber_map_;
    rclcpp::TimerBase::SharedPtr m_ros_timer_;
    
    std::string m_topic_name_map_subscribed_;
    std::string m_topic_name_scan_subscribed_;
    std::string m_topic_name_depth_subscribed_;
    std::string m_topic_name_map_published_;

    nav_msgs::msg::OccupancyGrid m_map_msg_;
    sensor_msgs::msg::LaserScan m_scan_msg_;
    sensor_msgs::msg::PointCloud m_depth_msg_;

    std::string m_map_frame_;
    std::string m_lidar_frame_;
    std::string m_depth_frame_;

    std::mutex m_map_mutex_;
    std::mutex m_scan_mutex_;
    std::mutex m_depth_mutex_;

    bool FLAG_received_map_;
    bool FLAG_enable_scan_;
    bool FLAG_enable_depth_;

    void mapCallback(const nav_msgs::msg::OccupancyGrid &msg);
    void scanCallback(const sensor_msgs::msg::LaserScan &msg);
    void depthCallback(const sensor_msgs::msg::PointCloud &msg);

    void mergeData();

private:

    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_;

    geometry_msgs::msg::TransformStamped m_lidar_to_base_tf_;

    std::unique_ptr<ClassInflationSamples> m_inflation_ptr_;
    int8_t m_inflation_radius_;

    void loadParameters();

    void getLidarPoseInMapFrame();
    void convertRealToGrid(const double x_meter, const double y_meter, int &r_x_index, int &r_y_index);
    int convertGrid2dTo1d(const int xgrid, const int ygrid);

    void putLidarIntoGrids(nav_msgs::msg::OccupancyGrid &r_grids, sensor_msgs::msg::LaserScan &r_scan);

    void inflateOneCell(nav_msgs::msg::OccupancyGrid *ptr_map_msg, 
                        std::unique_ptr<ClassInflationSamples>& ptr_inflation, 
                        const int center_x, 
                        const int center_y);

public:
    ClassMapFusion();
    ~ClassMapFusion();
};

} // namespace hawa

# endif 

