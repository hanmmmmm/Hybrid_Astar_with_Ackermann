
#include "class_map_fusion.h"


namespace hawa
{

ClassMapFusion::ClassMapFusion(): Node("map_fusion_node")
{
    loadParameters();

    m_inflation_ptr_ = std::make_unique<ClassInflationSamples>(m_inflation_min_intensity_, m_inflation_radius_);

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

    m_ros_timer_ = this->create_wall_timer( 100ms, std::bind(&ClassMapFusion::mergeData, this) );

    FLAG_received_map_ = false;
    
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
    typedef boost::property_tree::ptree ptree;

    std::string cfg_path = "/home/jf/Hybrid_Astar_with_Ackermann/hawa_cfg.json";

    std::cout << "cfg path: >" << cfg_path << "< " << std::endl;

    ptree root;
    boost::property_tree::read_json(cfg_path, root);

    ptree topics = root.get_child("ros_topics");

    m_topic_name_map_subscribed_ = topics.get<std::string>("map_from_slam");            //"/map";
    m_topic_name_depth_subscribed_ = topics.get<std::string>("depth_topic");                 //"/depth";
    m_topic_name_scan_subscribed_ = topics.get<std::string>("lidar_topic");   //"/scan";
    m_topic_name_map_published_ = topics.get<std::string>("map_from_map_processor");   //"/map_fusion";

    ptree frames = root.get_child("frames");

    m_map_frame_ = frames.get<std::string>("map_frame"); 
    m_lidar_frame_ = frames.get<std::string>("lidar_frame");      // "/laser";
    m_depth_frame_ = frames.get<std::string>("depth_frame");      // "/depth";

    ptree map_processor = root.get_child("map_processor");

    m_inflation_radius_ = map_processor.get<int8_t>("inflation_size_INT");
    m_inflation_min_intensity_ = map_processor.get<int8_t>("min_intensity_INT");
    m_obstacle_thershold_ = map_processor.get<int8_t>("obstacle_threshold_INT");
    m_lidar_range_max_ = map_processor.get<double>("lidar_range_max_FLOAT");
    FLAG_enable_scan_ = map_processor.get<bool>("include_lidar_BOOL");
    FLAG_enable_depth_ = map_processor.get<bool>("include_depth_BOOL");

}

/**
 * @brief Ros callback for the original gridmap. 
*/
void ClassMapFusion::mapCallback(const nav_msgs::msg::OccupancyGrid &msg)
{
    m_map_mutex_.lock();
    m_map_msg_ = msg;
    FLAG_received_map_ = true;
    m_map_mutex_.unlock();
}

/**
 * @brief Ros callback for the lidar scan messages. It's just passed to a variable to store the data. Other 
 * processings for this data are happening in mergeData(), not in this callback.
*/
void ClassMapFusion::scanCallback(const sensor_msgs::msg::LaserScan &msg)
{
    if (! FLAG_enable_scan_)
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
    if( ! FLAG_received_map_ ) return;

    ClassHawaTimer _timer;
    _timer.startNow();

    m_map_mutex_.lock();
    if (FLAG_enable_scan_) m_scan_mutex_.lock();
    if (FLAG_enable_depth_) m_depth_mutex_.lock();

    // ------------- prepare the container for the new map to publish -------------
    nav_msgs::msg::OccupancyGrid _new_map_to_pub;
    _new_map_to_pub.data = m_map_msg_.data;
    _new_map_to_pub.header = m_map_msg_.header;
    _new_map_to_pub.info = m_map_msg_.info;

    // ------------- inflate the static section in the map -------------
    
    int _index_1D;
    for(int x_ind = 0; x_ind<int(_new_map_to_pub.info.width); x_ind++)
    {
        for(int y_ind = 0; y_ind<int(_new_map_to_pub.info.height); y_ind++)
        {
            _index_1D = convertGrid2dTo1d(x_ind, y_ind);
            if (m_map_msg_.data.at(_index_1D) > m_obstacle_thershold_)
            {
                inflateOneCell(&_new_map_to_pub, m_inflation_ptr_, x_ind, y_ind);
            }
        }
    }

    // ------------- add the sensor data into the map -------------
    
    if (FLAG_enable_scan_)
    {
        putLidarIntoGrids(_new_map_to_pub, m_scan_msg_);
    }

    // ------------- publish the new map and release the mutex -------------

    m_puber_map_->publish( _new_map_to_pub );

    m_map_mutex_.unlock();
    if (FLAG_enable_scan_) m_scan_mutex_.unlock();
    if (FLAG_enable_depth_) m_depth_mutex_.unlock();

    _timer.endTiming();
}

/**
 * @brief Call this function when need the relative pose between lidar frame and robot base frame. 
*/
void ClassMapFusion::getLidarPoseInMapFrame()
{
    try{
        m_lidar_to_base_tf_ = m_tf_buffer_->lookupTransform(m_map_frame_, m_lidar_frame_, rclcpp::Time(0));
    }
    catch (tf2::TransformException const& ex){
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
                                    std::unique_ptr<ClassInflationSamples>& ptr_inflation, 
                                    const int center_x, 
                                    const int center_y
                                    )
{
    int _radius = m_inflation_radius_;

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
    auto inflate_sample = ptr_inflation->getInflationSample();
    for( int y_ind = y_start_in_map; y_ind<y_end_in_map; y_ind++ )
    {
        x_counter = 0;
        for( int x_ind = x_start_in_map; x_ind<x_end_in_map; x_ind++ )
        {
            index_1D = convertGrid2dTo1d(x_ind, y_ind);
            int _temp_y = y_start_in_sample + y_counter;
            int _temp_x = x_start_in_sample + x_counter;
            auto val = inflate_sample.at(_temp_y).at(_temp_x);
            if( val > 0)
            {
                ptr_map_msg->data.at(index_1D) = std::max(val, ptr_map_msg->data.at(index_1D));
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
    int _x_index, _y_index;
    // int _index_1D;
    for( float range : r_scan.ranges )
    {
        try
        {
            if( range > m_lidar_range_max_) 
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
            // _index_1D = convertGrid2dTo1d(_x_index, _y_index);
            inflateOneCell(&r_grids, m_inflation_ptr_, _x_index, _y_index);
        }
        catch (tf2::TransformException  const& ex)
        {
            std::stringstream _ss;
            _ss << "getLidarPoseInMapFrame: " << ex.what();
            RCLCPP_ERROR(this->get_logger(), _ss.str().c_str());
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
        _angle += _angle_inc ; 
    }
}

}
