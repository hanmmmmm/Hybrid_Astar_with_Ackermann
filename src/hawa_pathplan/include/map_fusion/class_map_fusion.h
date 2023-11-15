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

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Odometry.h"

#include "tf/transform_listener.h"
#include <algorithm>
#include <atomic>
#include <future>
#include <thread>
#include <chrono>
#include <mutex>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "class_inflation_samples.h"

using std::chrono::high_resolution_clock;


class ClassMapFusion
{
private:
    
    std::string map_subscribed_topic_name_;
    std::string depth_subscribed_topic_name_;
    std::string scan_subscribed_topic_name_;
    std::string sonic_subscribed_topic_name_;
    ros::Subscriber suber_map_;
    ros::Subscriber suber_depth_;
    ros::Subscriber suber_scan_;
    ros::Subscriber suber_sonic_;

    std::string depth_enable_subscribed_topic_name_;
    std::string scan_enable_subscribed_topic_name_;
    std::string sonic_enable_subscribed_topic_name_;
    ros::Subscriber suber_depth_enable_;
    ros::Subscriber suber_scan_enable_;
    ros::Subscriber suber_sonic_enable_;

    bool scan_enabled_, depth_enabled_, sonic_enabled_;
    bool map_received_; 


    std::string map_published_topic_name_;
    ros::Publisher map_puber_;

    ros::NodeHandle nh_;
    tf::TransformListener tf_listener;

    ros::Timer  periodic_merge_data_;
    float merge_data_time_interval_;

    std::mutex map_mutex_, scan_mutex_, depth_mutex_, sonic_mutex_;

    std::string map_frame_, lidar_frame_, depth_frame_, sonic_frame_; 

    tf::StampedTransform lidar_to_base_tf_;

    //  member functions
    // void get_tf_laser_to_base(std::array<float, 3> &the_tf);


    struct inflationInfo
    {
        std::vector<std::vector<int8_t>> inflate_sample;
        int inflate_radius = 4 ;  // grid_wise
    };

    inflationInfo inflation_;
    ClassInflationSamples inflat_options; 

    void load_parameters();

    float mod_2pi(float angle);

    nav_msgs::OccupancyGrid map_msg_;
    sensor_msgs::LaserScan  scan_msg_;
    sensor_msgs::PointCloud depth_msg_;

    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void depth_callback(const sensor_msgs::PointCloud::ConstPtr &msg);

    void merge_data( const ros::TimerEvent &event );

    void get_lidar_pose_in_map_frame();
    void helper_xy_meter_to_grid(const double x_meter, const double y_meter, int& x_index, int& y_index);
    int helper_grid2D_to_1D(const int xgrid, const int ygrid);
    double helper_get_time();

    void inflate_one_cell(std::vector<int8_t> &grids, const int x, const int y, const int map_width, const int map_height, const std::vector<std::vector<int8_t>> &inflate_sample_in, const int radius);

public:
    ClassMapFusion(const ros::NodeHandle nh_in_);
    ~ClassMapFusion();
};

ClassMapFusion::ClassMapFusion(const ros::NodeHandle nh_in_): nh_{nh_in_}
{
    load_parameters();

    map_puber_  = nh_.advertise<nav_msgs::OccupancyGrid>( map_published_topic_name_, 10);

    suber_map_ = nh_.subscribe( map_subscribed_topic_name_ , 1, &ClassMapFusion::map_callback,  this);
    suber_scan_= nh_.subscribe( scan_subscribed_topic_name_ , 1, &ClassMapFusion::scan_callback, this);
    suber_depth_= nh_.subscribe( depth_subscribed_topic_name_ , 10, &ClassMapFusion::depth_callback, this);

    periodic_merge_data_ = nh_.createTimer( ros::Duration(merge_data_time_interval_), &ClassMapFusion::merge_data, this );

    scan_enabled_ = false;
    depth_enabled_= false;
    sonic_enabled_= false;
    map_received_ = false; 

    // build_inflate_sample(inflation_.inflate_sample, inflation_.inflate_radius);
    inflat_options.get_sample_by_radius(inflation_.inflate_sample, inflation_.inflate_radius);

    std::cout << "ClassMapFusion  init done" << std::endl;
}

ClassMapFusion::~ClassMapFusion()
{
}

void ClassMapFusion::load_parameters(){
    map_subscribed_topic_name_ = "/map";
    depth_subscribed_topic_name_ = "/depth";
    scan_subscribed_topic_name_ = "/scan";
    sonic_subscribed_topic_name_ = "/sonic";
    map_frame_ = "/map";
    lidar_frame_ = "/laser";
    depth_frame_ = "/depth_link";
    sonic_frame_ = "/sonic_link";

    map_published_topic_name_ = "/map_fusion";

    merge_data_time_interval_ = 0.1;

    inflation_.inflate_radius = 4;

}

void ClassMapFusion::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg){
    map_mutex_.lock();
    // std::cout << "map_callback start" << std::endl;
    map_msg_.data = msg->data;
    map_msg_.header = msg->header;
    map_msg_.info = msg->info;
    map_received_ = true;
    // std::cout << "map_callback end" << std::endl;
    map_mutex_.unlock();
}


void ClassMapFusion::scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg){
    if (scan_enabled_){
        scan_mutex_.lock();
        // std::cout << "scan_callback start" << std::endl;
        scan_msg_ = *msg;
        // std::cout << "scan_callback end" << std::endl;
        scan_mutex_.unlock(); 
    }
}

void ClassMapFusion::depth_callback(const sensor_msgs::PointCloud::ConstPtr &msg){

}

void ClassMapFusion::merge_data( const ros::TimerEvent &event ){

    if( ! map_received_ ) return;

    // std::cout << "merge_data start" << std::endl;

    double t1 = helper_get_time();

    map_mutex_.lock();
    if (scan_enabled_) scan_mutex_.lock();
    if (depth_enabled_) depth_mutex_.lock();
    if (sonic_enabled_) sonic_mutex_.lock(); 

    nav_msgs::OccupancyGrid map_to_pub;
    map_to_pub.data = map_msg_.data;
    map_to_pub.header = map_msg_.header;
    map_to_pub.info = map_msg_.info;

    // if use scan, then get lidar->map tf,     
    // loop through each point in lidar, compute xy position, convert to gridmap posistion, 
    // update the gridmap

    if (scan_enabled_){
        get_lidar_pose_in_map_frame(); 

        // std::cout << "lidar_to_base_tf_ " << std::endl;
        // std::cout << lidar_to_base_tf_.getOrigin().x() << " " << lidar_to_base_tf_.getOrigin().y() << " " << lidar_to_base_tf_.getOrigin().z() << std::endl;
        // std::cout << lidar_to_base_tf_.getRotation().x() << " " << lidar_to_base_tf_.getRotation().y() << " " << lidar_to_base_tf_.getRotation().z() << " " << lidar_to_base_tf_.getRotation().w() << std::endl;

        double angle = scan_msg_.angle_min;

        double angle_inc = scan_msg_.angle_increment; 

        tf::Transform point_tf_in_lidar;
        tf::Transform point_tf_in_map ;
        double x, y, x_in_map, y_in_map;
        int x_index, y_index, index_1D;
        for( float range : scan_msg_.ranges ){

            try{
                if( range > 20.0) {
                    angle += angle_inc ;  
                    continue;
                }

                x = range * sin( -angle + M_PI/2.0 );
                y = range * cos( -angle + M_PI/2.0 );

                point_tf_in_lidar.setOrigin(tf::Vector3(x,y,0));
                point_tf_in_map =   lidar_to_base_tf_ * point_tf_in_lidar;

                x_in_map = point_tf_in_map.getOrigin().x();
                y_in_map = point_tf_in_map.getOrigin().y();
                
                helper_xy_meter_to_grid(x_in_map, y_in_map, x_index, y_index);
                index_1D = helper_grid2D_to_1D(x_index, y_index);
                // if (map_to_pub.data[index_1D] < 90) map_to_pub.data[index_1D] = 100; 
                // map_to_pub.data[index_1D] = 100; 
                inflate_one_cell( map_to_pub.data, x_index, y_index, map_to_pub.info.width, map_to_pub.info.height, inflation_.inflate_sample, inflation_.inflate_radius);
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
        int index_1D;
        for(int x_ind = 0; x_ind<map_to_pub.info.width; x_ind++)
        {
            for(int y_ind = 0; y_ind<map_to_pub.info.height; y_ind++)
            {
                index_1D = helper_grid2D_to_1D(x_ind, y_ind);
                if (map_msg_.data[index_1D] > 80)
                {
                    inflate_one_cell( map_to_pub.data, x_ind, y_ind, map_to_pub.info.width, map_to_pub.info.height, inflation_.inflate_sample, inflation_.inflate_radius);
                }
                
            }
        }
    }

    // std::cout << "merged scan" << std::endl;

    // std::cout << "inflation done" << std::endl;

    map_puber_.publish( map_to_pub );

    // std::cout << "publish done" << std::endl;

    map_mutex_.unlock();
    if (scan_enabled_) scan_mutex_.unlock();
    if (depth_enabled_) depth_mutex_.unlock();
    if (sonic_enabled_) sonic_mutex_.unlock(); 


    double t2 = helper_get_time(); 

    std::cout << "function merge_data " << int((t2-t1)*1000.0) << " ms" << std::endl;


}



void ClassMapFusion::get_lidar_pose_in_map_frame(){
    try{
        tf_listener.lookupTransform( map_frame_, lidar_frame_, ros::Time(0), lidar_to_base_tf_);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("\nget_lidar_pose_in_map_frame: \n%s",ex.what());
      ros::Duration(0.2).sleep();
    }
}


int ClassMapFusion::helper_grid2D_to_1D(const int xgrid, const int ygrid)
{
    int width  = map_msg_.info.width;
    int index = ygrid * width + xgrid;
    return index;
}


/**
 * @brief 
 * convert pose x-y from meter to grid map cell index; xy should be pose in ref to frame_id of this map 
 */
void ClassMapFusion::helper_xy_meter_to_grid(const double x_meter, const double y_meter, int& x_index, int& y_index)
{
    float reso = map_msg_.info.resolution; // m/cell 
    double x_meter_to_origin = x_meter - map_msg_.info.origin.position.x;
    double y_meter_to_origin = y_meter - map_msg_.info.origin.position.y;
    x_index = x_meter_to_origin / reso;
    y_index = y_meter_to_origin / reso;
}

/**
 * @brief 
 * return time in seconds, epoch time. 
 */
double ClassMapFusion::helper_get_time()
{
    return high_resolution_clock::now().time_since_epoch().count()/1000000000.0; 
}



/**
 * @brief Change the map data grids, around position (x,y) , using the inflation_sample_grids. 
 */
void ClassMapFusion::inflate_one_cell( std::vector<int8_t>& grids, 
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

    // validate
    int width_in_map = x_end_in_map - x_start_in_map;
    int height_in_map= y_end_in_map - y_start_in_map;
    int width_in_smpl  = x_end_in_smpl - x_start_in_smpl;
    int height_in_smpl = y_end_in_smpl - y_start_in_smpl;
    if( width_in_map != width_in_smpl ){
        std::cout << "inflation: width dont match." << std::endl;
        return;
    }
    if( height_in_map != height_in_smpl ){
        std::cout << "inflation: height dont match." << std::endl;
        return;
    }
    // validate Done

    // filling the map data vector. 
    int index_1D;
    int x_counter = 0;
    int y_counter = 0;
    for( int y_ind = y_start_in_map; y_ind<y_end_in_map; y_ind++ ){
        x_counter = 0;
        for( int x_ind = x_start_in_map; x_ind<x_end_in_map; x_ind++ ){
            index_1D = helper_grid2D_to_1D(x_ind, y_ind);
            if( inflate_sample_in[y_start_in_smpl+y_counter][x_start_in_smpl + x_counter] > 0 ){
                grids[index_1D] = std::max( inflate_sample_in[y_start_in_smpl+y_counter][x_start_in_smpl + x_counter], grids[index_1D]);
            }
            x_counter ++ ;
        }
        y_counter ++ ;
    }

}

# endif 

