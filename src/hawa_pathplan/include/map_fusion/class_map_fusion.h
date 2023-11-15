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

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"

#include "class_inflation_samples.h"
#include "../utils/hawa_timer_class.h"
#include "../utils/hawa_tools.h"


/**
 * @brief This class receives the raw gridmap and other range sensors, then add the sensor data into the 
 * gridmap and publish the result into a new topic. 
*/
class ClassMapFusion
{
private:

    ros::NodeHandle m_nh_;
    
    // things for map data
    ros::Subscriber m_suber_map_;
    std::string m_topic_name_map_subscribed_;
    nav_msgs::OccupancyGrid m_map_msg_;
    std::string m_map_frame_;
    std::mutex m_map_mutex_;
    bool m_received_map_;

    // things for lidar data
    ros::Subscriber m_suber_scan_;
    std::string m_topic_name_scan_subscribed_;
    sensor_msgs::LaserScan m_scan_msg_;
    std::string m_lidar_frame_;
    std::mutex m_scan_mutex_;
    bool m_enable_scan_;

    // things for depth camera data
    ros::Subscriber m_suber_depth_;
    std::string m_topic_name_depth_subscribed_;
    sensor_msgs::PointCloud m_depth_msg_;
    std::string m_depth_frame_; 
    std::mutex m_depth_mutex_;
    bool m_depth_enabled_;

    // publish the result
    std::string m_topic_name_map_published_;
    ros::Publisher m_puber_map_;
    ros::Timer m_ros_timer_;

    tf::TransformListener tf_listener;
    tf::StampedTransform lidar_to_base_tf_;

    struct inflationInfo
    {
        std::vector<std::vector<int8_t>> inflate_sample;
        int inflate_radius = 5 ;  // grid_wise
    };

    inflationInfo inflation_;
    ClassInflationSamples inflat_options; 

private:

    void loadParameters();

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void depthCallback(const sensor_msgs::PointCloud::ConstPtr &msg);

    void mergeData( const ros::TimerEvent &event );

    void get_lidar_pose_in_map_frame();
    void convertRealToGrid(const double x_meter, const double y_meter, int& x_index, int& y_index);
    int convertGrid2dTo1d(const int xgrid, const int ygrid);

    void inflateOneCell(std::vector<int8_t> &grids, const int x, const int y, const int map_width, const int map_height, const std::vector<std::vector<int8_t>> &inflate_sample_in, const int radius);

public:
    ClassMapFusion(const ros::NodeHandle nh_in_);
    ~ClassMapFusion();
};


ClassMapFusion::ClassMapFusion(const ros::NodeHandle nh_in_): m_nh_{nh_in_}
{
    loadParameters();

    m_puber_map_  = m_nh_.advertise<nav_msgs::OccupancyGrid>( m_topic_name_map_published_, 10);

    m_suber_map_ = m_nh_.subscribe( m_topic_name_map_subscribed_ , 1, &ClassMapFusion::mapCallback,  this);
    m_suber_scan_= m_nh_.subscribe( m_topic_name_scan_subscribed_ , 1, &ClassMapFusion::scanCallback, this);
    m_suber_depth_= m_nh_.subscribe( m_topic_name_depth_subscribed_ , 10, &ClassMapFusion::depthCallback, this);

    m_ros_timer_ = m_nh_.createTimer( ros::Duration(0.1), &ClassMapFusion::mergeData, this );

    m_enable_scan_ = false;
    m_depth_enabled_= false;
    m_received_map_ = false; 

    // build_inflate_sample(inflation_.inflate_sample, inflation_.inflate_radius);
    inflat_options.get_sample_by_radius(inflation_.inflate_sample, inflation_.inflate_radius);

    std::cout << "ClassMapFusion  init done" << std::endl;
}

ClassMapFusion::~ClassMapFusion()
{
}

/**
 * 
*/
void ClassMapFusion::loadParameters(){
    m_topic_name_map_subscribed_ = "/map";
    m_topic_name_depth_subscribed_ = "/depth";
    m_topic_name_scan_subscribed_ = "/scan";

    m_map_frame_ = "/map";
    m_lidar_frame_ = "/laser";
    m_depth_frame_ = "/depth_link";

    m_topic_name_map_published_ = "/map_fusion";

}

/**
 * 
*/
void ClassMapFusion::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg){
    m_map_mutex_.lock();
    m_map_msg_.data = msg->data;
    m_map_msg_.header = msg->header;
    m_map_msg_.info = msg->info;
    m_received_map_ = true;
    m_map_mutex_.unlock();
}

/**
 * 
*/
void ClassMapFusion::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    if (! m_enable_scan_)
    {
        return;
    }
    m_scan_mutex_.lock();
    m_scan_msg_ = *msg;
    m_scan_mutex_.unlock(); 
}

/**
 * 
*/
void ClassMapFusion::depthCallback(const sensor_msgs::PointCloud::ConstPtr &msg)
{

}

/**
 * 
*/
void ClassMapFusion::mergeData( const ros::TimerEvent &event )
{
    if( ! m_received_map_ ) return;

    ClassHawaTimer _timer;
    _timer.startNow();

    m_map_mutex_.lock();
    if (m_enable_scan_) m_scan_mutex_.lock();
    if (m_depth_enabled_) m_depth_mutex_.lock();

    nav_msgs::OccupancyGrid _new_map_to_pub;
    _new_map_to_pub.data = m_map_msg_.data;
    _new_map_to_pub.header = m_map_msg_.header;
    _new_map_to_pub.info = m_map_msg_.info;

    // if use scan, then get lidar->map tf,     
    // loop through each point in lidar, compute xy position, convert to gridmap posistion, 
    // update the gridmap

    if (m_enable_scan_){
        get_lidar_pose_in_map_frame(); 

        // std::cout << "lidar_to_base_tf_ " << std::endl;
        // std::cout << lidar_to_base_tf_.getOrigin().x() << " " << lidar_to_base_tf_.getOrigin().y() << " " << lidar_to_base_tf_.getOrigin().z() << std::endl;
        // std::cout << lidar_to_base_tf_.getRotation().x() << " " << lidar_to_base_tf_.getRotation().y() << " " << lidar_to_base_tf_.getRotation().z() << " " << lidar_to_base_tf_.getRotation().w() << std::endl;

        double angle = m_scan_msg_.angle_min;

        double angle_inc = m_scan_msg_.angle_increment; 

        tf::Transform point_tf_in_lidar;
        tf::Transform point_tf_in_map ;
        double x, y, x_in_map, y_in_map;
        int x_index, y_index, index_1D;
        for( float range : m_scan_msg_.ranges )
        {
            try{
                if( range > 20.0) {
                    angle += angle_inc ;  
                    continue;
                }

                x = range * sin( -angle + M_PI/2.0 );
                y = range * cos( -angle + M_PI/2.0 );

                point_tf_in_lidar.setOrigin(tf::Vector3(x,y,0));
                point_tf_in_map = lidar_to_base_tf_ * point_tf_in_lidar;

                x_in_map = point_tf_in_map.getOrigin().x();
                y_in_map = point_tf_in_map.getOrigin().y();
                
                convertRealToGrid(x_in_map, y_in_map, x_index, y_index);
                index_1D = convertGrid2dTo1d(x_index, y_index);
                // if (_new_map_to_pub.data[index_1D] < 90) _new_map_to_pub.data[index_1D] = 100; 
                // _new_map_to_pub.data[index_1D] = 100; 
                inflateOneCell( _new_map_to_pub.data, x_index, y_index, _new_map_to_pub.info.width, _new_map_to_pub.info.height, inflation_.inflate_sample, inflation_.inflate_radius);
            }
            catch (tf::TransformException ex){
                // ROS_ERROR("\nget_lidar_pose_in_map_frame: \n%s",ex.what());
                // ros::Duration(0.5).sleep();
            }
            angle += angle_inc ; 
        }
    }
    else
    {
        // inflate the static section in the map
        int _index_1D;
        for(int x_ind = 0; x_ind<_new_map_to_pub.info.width; x_ind++)
        {
            for(int y_ind = 0; y_ind<_new_map_to_pub.info.height; y_ind++)
            {
                _index_1D = convertGrid2dTo1d(x_ind, y_ind);
                if (m_map_msg_.data.at(_index_1D) > 70)
                {
                    inflateOneCell(_new_map_to_pub.data, 
                                   x_ind, 
                                   y_ind, 
                                   _new_map_to_pub.info.width, 
                                   _new_map_to_pub.info.height, 
                                   inflation_.inflate_sample, 
                                   inflation_.inflate_radius
                                   );
                }
            }
        }
    }

    m_puber_map_.publish( _new_map_to_pub );

    m_map_mutex_.unlock();
    if (m_enable_scan_) m_scan_mutex_.unlock();
    if (m_depth_enabled_) m_depth_mutex_.unlock();

    _timer.endTiming();

    ROS_DEBUG_STREAM_THROTTLE(30, "map fusion "<< int(_timer.getDuration()*1000.0) << " ms");
}



void ClassMapFusion::get_lidar_pose_in_map_frame(){
    try{
        tf_listener.lookupTransform( m_map_frame_, m_lidar_frame_, ros::Time(0), lidar_to_base_tf_);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("\nget_lidar_pose_in_map_frame: \n%s",ex.what());
      ros::Duration(0.2).sleep();
    }
}

/**
 * @brief 
*/
inline int ClassMapFusion::convertGrid2dTo1d(const int xgrid, const int ygrid)
{
    return ygrid * m_map_msg_.info.width + xgrid;
}

/**
 * @brief Convert pose x-y from meter to grid map cell index; xy should be pose in ref to frame_id of this map 
 */
void ClassMapFusion::convertRealToGrid(const double x_meter, const double y_meter, int& x_index, int& y_index)
{
    float _resolution = m_map_msg_.info.resolution; // m/cell 
    double _x_meter_to_origin = x_meter - m_map_msg_.info.origin.position.x;
    double _y_meter_to_origin = y_meter - m_map_msg_.info.origin.position.y;
    x_index = _x_meter_to_origin / _resolution;
    y_index = _y_meter_to_origin / _resolution;
}

/**
 * @brief Change the map data grids, around position (x,y) , using the inflation_sample_grids. 
 */
void ClassMapFusion::inflateOneCell( std::vector<int8_t>& grids, 
                                       const int x, 
                                       const int y, 
                                       const int map_width, 
                                       const int map_height, 
                                       const std::vector<std::vector<int8_t>>& inflate_sample_in , 
                                       const int radius  )
{
    // prepare the index ranges 
    int x_start_in_map, x_end_in_map, y_start_in_map, y_end_in_map;
    int x_start_in_smpl, x_end_in_smpl, y_start_in_smpl, y_end_in_smpl;

    x_start_in_map = std::max( x-radius, 0 );
    x_end_in_map   = std::min( x+radius+1, map_width );
    x_start_in_smpl= std::max( radius-x, 0 );
    x_end_in_smpl  = std::min( radius + map_width -1-x, radius*2+1 );

    y_start_in_map = std::max( y-radius, 0 );
    y_end_in_map   = std::min( y+radius+1, map_height );
    y_start_in_smpl= std::max( radius-y, 0 );
    y_end_in_smpl  = std::min( radius + map_height -1-y, radius*2+1 );

    // std::cout << x << " " << y << " ; " << map_width << " " << map_height << " ; " << x_start_in_map << " ; " << x_end_in_map << std::endl; 

    // // This validation section has bugs for the grids on the left and bottom edges of the map. So I comment it out 
    // // for now. 
    // // validate
    // int width_in_map = x_end_in_map - x_start_in_map;
    // int height_in_map= y_end_in_map - y_start_in_map;
    // int width_in_smpl  = x_end_in_smpl - x_start_in_smpl;
    // int height_in_smpl = y_end_in_smpl - y_start_in_smpl;
    // if( width_in_map != width_in_smpl )
    // {
    //     std::cout << "inflation: width dont match. " << width_in_map << " " << width_in_smpl << std::endl;
    //     std::cout << "x " << x << " y " << y << " radius " << radius << " map_width " << map_width 
    //     << " map_height " << map_height << std::endl;
    //     return;
    // }
    // if( height_in_map != height_in_smpl )
    // {
    //     std::cout << "inflation: height dont match." << height_in_map << " " << height_in_smpl << std::endl;
    //     return;
    // }
    // // validate Done

    // filling the map data vector. 
    int index_1D;
    int x_counter = 0;
    int y_counter = 0;
    for( int y_ind = y_start_in_map; y_ind<y_end_in_map; y_ind++ ){
        x_counter = 0;
        for( int x_ind = x_start_in_map; x_ind<x_end_in_map; x_ind++ ){
            index_1D = convertGrid2dTo1d(x_ind, y_ind);
            if( inflate_sample_in[y_start_in_smpl+y_counter][x_start_in_smpl + x_counter] > 0 ){
                grids[index_1D] = std::max( inflate_sample_in[y_start_in_smpl+y_counter][x_start_in_smpl + x_counter], grids[index_1D]);
            }
            x_counter ++ ;
        }
        y_counter ++ ;
    }

}

# endif 

