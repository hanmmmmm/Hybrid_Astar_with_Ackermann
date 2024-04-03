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
#include <atomic>
#include <future>
#include <thread>
#include <mutex>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

// #include "ros/ros.h"
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


using namespace std::chrono_literals;
// using std::placeholders::_1;

/**
 * @brief This class receives the raw gridmap and other range sensors, then add the sensor data into the 
 * gridmap and publish the result into a new topic. 
*/
class ClassMapFusion : public rclcpp::Node
{
private:

    // ros::NodeHandle m_nh_;
    
    // things for map data
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_suber_map_;
    std::string m_topic_name_map_subscribed_;
    nav_msgs::msg::OccupancyGrid m_map_msg_;
    std::string m_map_frame_;
    std::mutex m_map_mutex_;
    bool m_received_map_;

    // things for lidar data
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_suber_scan_;
    std::string m_topic_name_scan_subscribed_;
    sensor_msgs::msg::LaserScan m_scan_msg_;
    std::string m_lidar_frame_;
    std::mutex m_scan_mutex_;
    bool m_enable_scan_;

    // things for depth camera data
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr m_suber_depth_;
    std::string m_topic_name_depth_subscribed_;
    sensor_msgs::msg::PointCloud m_depth_msg_;
    std::string m_depth_frame_; 
    std::mutex m_depth_mutex_;
    bool m_depth_enabled_;

    // publish the result
    std::string m_topic_name_map_published_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_puber_map_;
    rclcpp::TimerBase::SharedPtr m_ros_timer_;

    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_;

    geometry_msgs::msg::TransformStamped m_lidar_to_base_tf_;

    ClassInflationSamples m_inflation_samples; 

private:

    void loadParameters();

    void mapCallback(const nav_msgs::msg::OccupancyGrid &msg);
    void scanCallback(const sensor_msgs::msg::LaserScan &msg);
    void depthCallback(const sensor_msgs::msg::PointCloud &msg);

    void mergeData();

    void getLidarPoseInMapFrame();
    void convertRealToGrid(const double x_meter, const double y_meter, int &r_x_index, int &r_y_index);
    int convertGrid2dTo1d(const int xgrid, const int ygrid);

    void putLidarIntoGrids(nav_msgs::msg::OccupancyGrid &r_grids, sensor_msgs::msg::LaserScan &r_scan);

    void inflateOneCell(nav_msgs::msg::OccupancyGrid *ptr_map_msg, 
                        ClassInflationSamples *ptr_inflation, 
                        const int center_x, 
                        const int center_y);

public:
    ClassMapFusion();
    ~ClassMapFusion();
};


ClassMapFusion::ClassMapFusion(): Node("map_fusion_node")
{
    loadParameters();

    m_inflation_samples.setRadius(5);
    m_inflation_samples.prepareSampleByRadius();

    m_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_);

    m_puber_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        m_topic_name_map_published_, 10
    );

    m_suber_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        m_topic_name_map_subscribed_, 1, std::bind(&ClassMapFusion::mapCallback, this, std::placeholders::_1)
    );

    m_suber_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        m_topic_name_scan_subscribed_, 2, std::bind(&ClassMapFusion::scanCallback, this, std::placeholders::_1)
    );

    m_suber_depth_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
        m_topic_name_depth_subscribed_, 10, std::bind(&ClassMapFusion::depthCallback, this, std::placeholders::_1)
    );

    // m_puber_map_  = m_nh_.advertise<nav_msgs::OccupancyGrid>( m_topic_name_map_published_, 10);

    // m_suber_map_ = m_nh_.subscribe( m_topic_name_map_subscribed_ , 1, &ClassMapFusion::mapCallback,  this);
    // m_suber_scan_= m_nh_.subscribe( m_topic_name_scan_subscribed_ , 2, &ClassMapFusion::scanCallback, this);
    // m_suber_depth_= m_nh_.subscribe( m_topic_name_depth_subscribed_ , 10, &ClassMapFusion::depthCallback, this);

    // m_ros_timer_ = m_nh_.createTimer( ros::Duration(0.1), &ClassMapFusion::mergeData, this );

    m_ros_timer_ = this->create_wall_timer( 100ms, std::bind(&ClassMapFusion::mergeData, this) );

    m_enable_scan_ = false;
    m_depth_enabled_= false;
    m_received_map_ = false; 
    
    RCLCPP_INFO(this->get_logger(), "ClassMapFusion init done.");
}

ClassMapFusion::~ClassMapFusion()
{
}

/**
 * @brief This should be executed during the initialization. At this moment, these names are hardcoded
 * in strings, but they should be loaded from configuration json files for easier configuration. 
*/
void ClassMapFusion::loadParameters()
{
    m_topic_name_map_subscribed_ = "/map";
    m_topic_name_depth_subscribed_ = "/depth";
    m_topic_name_scan_subscribed_ = "/scan";

    m_map_frame_ = "/map";
    m_lidar_frame_ = "/laser";
    m_depth_frame_ = "/depth_link";

    m_topic_name_map_published_ = "/map_fusion";

}

/**
 * @brief Ros callback for the original gridmap. 
*/
void ClassMapFusion::mapCallback(const nav_msgs::msg::OccupancyGrid &msg)
{
    m_map_mutex_.lock();
    m_map_msg_ = msg;
    m_received_map_ = true;
    m_map_mutex_.unlock();
}

/**
 * @brief Ros callback for the lidar scan messages. It's just passed to a variable to store the data. Other 
 * processings for this data are happening in mergeData(), not in this callback.
*/
void ClassMapFusion::scanCallback(const sensor_msgs::msg::LaserScan &msg)
{
    if (! m_enable_scan_)
    {
        return;
    }
    m_scan_mutex_.lock();
    m_scan_msg_ = msg;
    m_scan_mutex_.unlock(); 
}

/**
 * @brief Ros callback for the depth camera data. At this moment, there is no depth camera message so 
 * nothing is implemented here. The details are to be added in the futrue. 
*/
void ClassMapFusion::depthCallback(const sensor_msgs::msg::PointCloud &msg)
{
    auto dummy = msg;
}

/**
 * @brief This is the main function in this class. This would be called by the ros timer. It generates the 
 * final grid map that contains the original gridmap, any other sensor data, and inflation. 
*/
void ClassMapFusion::mergeData( )
{
    if( ! m_received_map_ ) return;

    ClassHawaTimer _timer;
    _timer.startNow();

    m_map_mutex_.lock();
    if (m_enable_scan_) m_scan_mutex_.lock();
    if (m_depth_enabled_) m_depth_mutex_.lock();

    nav_msgs::msg::OccupancyGrid _new_map_to_pub;
    _new_map_to_pub.data = m_map_msg_.data;
    _new_map_to_pub.header = m_map_msg_.header;
    _new_map_to_pub.info = m_map_msg_.info;

    if (m_enable_scan_)
    {
        putLidarIntoGrids(_new_map_to_pub, m_scan_msg_);
    }
    else
    {
        // inflate the static section in the map
        int _index_1D;
        for(int x_ind = 0; x_ind<int(_new_map_to_pub.info.width); x_ind++)
        {
            for(int y_ind = 0; y_ind<int(_new_map_to_pub.info.height); y_ind++)
            {
                _index_1D = convertGrid2dTo1d(x_ind, y_ind);
                if (m_map_msg_.data.at(_index_1D) > 70)
                {
                    inflateOneCell(&_new_map_to_pub, &m_inflation_samples, x_ind, y_ind);
                }
            }
        }
    }

    m_puber_map_->publish( _new_map_to_pub );

    m_map_mutex_.unlock();
    if (m_enable_scan_) m_scan_mutex_.unlock();
    if (m_depth_enabled_) m_depth_mutex_.unlock();

    _timer.endTiming();

    // ROS_DEBUG_STREAM_THROTTLE(30, "map fusion "<< int(_timer.getDuration()*1000.0) << " ms");
}

/**
 * @brief Call this function when need the relative pose between lidar frame and robot base frame. 
*/
void ClassMapFusion::getLidarPoseInMapFrame()
{
    try{
        // m_tf_listener.lookupTransform(m_map_frame_, m_lidar_frame_, ros::Time(0), m_lidar_to_base_tf_);
        m_lidar_to_base_tf_ = m_tf_buffer_->lookupTransform(m_map_frame_, m_lidar_frame_, rclcpp::Time(0));
    }
    catch (tf2::TransformException ex){
        std::stringstream _ss;
        _ss << "getLidarPoseInMapFrame: " << ex.what();
        RCLCPP_ERROR(this->get_logger(), _ss.str().c_str());
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    }
}

/**
 * @brief Convert the given 2 dimensional coordinate into a 1 dimensional coordinate. Because the grid map
 * is stored as a 1 dimensional vector.
 * @param xgrid The x part of the 2D coordinate.
 * @param ygrid The y part of the 2D coordinate.
 * @return The 1 dimensional coordinate. 
*/
inline int ClassMapFusion::convertGrid2dTo1d(const int xgrid, const int ygrid)
{
    return ygrid * m_map_msg_.info.width + xgrid;
}

/**
 * @brief Convert pose x-y from meter to grid map cell index; values of x and y should be pose in reference
 * to frame_id of the map msg. The poses values usually come from real world metric coordinate, but many logics
 * happen in the virtual occupancy grid coordinate.
 * @param x_meter The x part of the 2D metric coordinate.
 * @param y_meter The y part of the 2D metric coordinate.
 * @param r_x_index The x part of the 2D grid coordinate.
 * @param r_x_index The y part of the 2D grid coordinate.
 */
void ClassMapFusion::convertRealToGrid(const double x_meter, const double y_meter, int &r_x_index, int &r_y_index)
{
    float _resolution = m_map_msg_.info.resolution; // meter/grid 
    double _x_meter_to_origin = x_meter - m_map_msg_.info.origin.position.x;
    double _y_meter_to_origin = y_meter - m_map_msg_.info.origin.position.y;
    r_x_index = _x_meter_to_origin / _resolution;
    r_y_index = _y_meter_to_origin / _resolution;
}

/**
 * @brief Use grid (x,y) as the center, inflate the grids around it, using the occupancy matrix from the file 
 * class_inflation_sample. 
 * @param ptr_map_msg Pointer to the occupancy grid to be filled. 
 * @param ptr_inflation Pointer to the inflation information manager object. 
 * @param x The center of this inflation. Grid-wise.
 * @param y The center of this inflation. Grid-wise.
 */
void ClassMapFusion::inflateOneCell(nav_msgs::msg::OccupancyGrid *ptr_map_msg, 
                                    ClassInflationSamples *ptr_inflation, 
                                    const int center_x, 
                                    const int center_y
                                    )
{
    int _radius = ptr_inflation->getRadius();

    int x_start_in_map = std::max(center_x - _radius, 0);
    int x_end_in_map   = std::min(center_x + _radius + 1, int(ptr_map_msg->info.width));
    int x_start_in_sample= std::max(_radius - center_x, 0);
    // int x_end_in_sample  = std::min(_radius + int(ptr_map_msg->info.width) - 1 - center_x, _radius * 2 + 1);

    int y_start_in_map = std::max(center_y - _radius, 0);
    int y_end_in_map   = std::min(center_y + _radius + 1, int(ptr_map_msg->info.height));
    int y_start_in_sample= std::max(_radius - center_y, 0);
    // int y_end_in_sample  = std::min(_radius + int(ptr_map_msg->info.height) - 1 - center_y, _radius * 2 + 1);

    int index_1D;
    int x_counter = 0;
    int y_counter = 0;
    for( int y_ind = y_start_in_map; y_ind<y_end_in_map; y_ind++ )
    {
        x_counter = 0;
        for( int x_ind = x_start_in_map; x_ind<x_end_in_map; x_ind++ )
        {
            index_1D = convertGrid2dTo1d(x_ind, y_ind);
            int _temp_y = y_start_in_sample + y_counter;
            int _temp_x = x_start_in_sample + x_counter;
            if(ptr_inflation->m_inflate_sample_.at(_temp_y).at(_temp_x) > 0)
            {
                ptr_map_msg->data.at(index_1D) = std::max(ptr_inflation->m_inflate_sample_.at(_temp_y).at(_temp_x), 
                                                          ptr_map_msg->data.at(index_1D));
            }
            x_counter ++ ;
        }
        y_counter ++ ;
    }
}

/**
 * @brief Add the lidar scan data into the grid map. 
 * @param r_grids Reference to the grid map data.
 * @param r_scan Reference to the lidar scan data.
*/
void ClassMapFusion::putLidarIntoGrids(nav_msgs::msg::OccupancyGrid &r_grids, sensor_msgs::msg::LaserScan &r_scan)
{
    getLidarPoseInMapFrame(); 

    double _angle = r_scan.angle_min;

    double _angle_inc = r_scan.angle_increment; 

    tf2::Transform _point_tf_in_lidar, _point_tf_in_map, _lidar_pose;

    tf2::fromMsg(m_lidar_to_base_tf_.transform, _lidar_pose);
    
    double _x, _y, _x_in_map, _y_in_map;
    int _x_index, _y_index, _index_1D;
    for( float range : r_scan.ranges )
    {
        try
        {
            if( range > 20.0) 
            {
                _angle += _angle_inc ;  
                continue;
            }
            _x = range * std::sin( -_angle + M_PI/2.0 );
            _y = range * std::cos( -_angle + M_PI/2.0 );

            _point_tf_in_lidar.setOrigin(tf2::Vector3(_x, _y, 0));
            // _point_tf_in_map = m_lidar_to_base_tf_ * _point_tf_in_lidar;
            _point_tf_in_map = _lidar_pose * _point_tf_in_lidar;

            _x_in_map = _point_tf_in_map.getOrigin().x();
            _y_in_map = _point_tf_in_map.getOrigin().y();
            
            convertRealToGrid(_x_in_map, _y_in_map, _x_index, _y_index);
            _index_1D = convertGrid2dTo1d(_x_index, _y_index);
            inflateOneCell(&r_grids, &m_inflation_samples, _x_index, _y_index);
        }
        catch (tf2::TransformException ex)
        {
            std::stringstream _ss;
            _ss << "getLidarPoseInMapFrame: " << ex.what();
            RCLCPP_ERROR(this->get_logger(), _ss.str().c_str());
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
        _angle += _angle_inc ; 
    }
}




# endif 

