
#include "class_hybrid_astar.h"


namespace hawa
{


/**
 * @brief Constructor function.
 */
ClassHybridAStar::ClassHybridAStar()
{
    loadParameters();

    // m_topic_name_searching_published_ = "/planner_searching";

    // rclcpp::Node::SharedPtr _node = rclcpp::Node::make_shared("temp_hybrid_astar_node");

    // m_publisher_searching_  = _node->create_publisher<std_msgs::msg::Bool>(
    //     m_topic_name_searching_published_, 10
    // );

    m_timer_and_counter_.resetVals();

    m_motion_model_.prepare_model();

    m_gridmap_handler_ptr_ = std::make_shared<ClassGridMapHandler>();
}

/**
 * @brief destructor.
 */
ClassHybridAStar::~ClassHybridAStar()
{
}

/**
 * @brief Load parameters from the json configuration files.
 * @return Currently always returns true.
 */
bool ClassHybridAStar::loadParameters()
{
    // std::string _path = ros::package::getPath("hawa_pathplan");
    // std::string _path = ament_index_cpp::get_package_share_directory("hawa_pathplan"); // this requires the cfg to locate in ros2_ws/install/... which is inconvenient.
    std::string _path = "/home/jf/Hybrid_Astar_with_Ackermann/src/hawa_pathplan";
    std::cout << "pkg path: >" << _path << "< " << std::endl;

    boost::property_tree::ptree root;
    boost::property_tree::read_json(_path + "/cfg/hybrid_astar_cfg.json", root);

    m_parameters_.setTimeOutMs(root.get<int>("searching_time_limt_ms"));

    m_motion_model_.parameters.turning_radius = root.get<double>("searching_turning_radius_meter");

    boost::property_tree::ptree step_length = root.get_child("searching_step_length_meter");
    m_motion_model_.parameters.step_length_forward = step_length.get<double>("forward");
    m_motion_model_.parameters.step_length_backward = step_length.get<double>("backward");

    boost::property_tree::ptree obstacle_threshold = root.get_child("gridmap_obstacle_threshold");
    m_map_occ_threshold_.plan = obstacle_threshold.get<int8_t>("planning");
    m_map_occ_threshold_.vali = obstacle_threshold.get<int8_t>("validate");

    m_rs_search_trigger_.interval = root.get<int>("reedshepp_every_n_nodes");

    m_parameters_.setYawBinSize(root.get<double>("yaw_angle_grid_bin_size_rad"));

    boost::property_tree::ptree action_cost_multipler = root.get_child("action_cost_multipler");
    m_motion_model_.cost_multipler.forward = action_cost_multipler.get<double>("forward_action");
    m_motion_model_.cost_multipler.backward = action_cost_multipler.get<double>("backward_action");
    m_motion_model_.cost_multipler.mild_steer = action_cost_multipler.get<double>("steer_action");
    m_motion_model_.cost_multipler.fast_steer = action_cost_multipler.get<double>("aggresive_steer");

    return true;
}

/**
 * @brief call this from external to set the data of the map.
 * @param ptr_map_data pointer of the OccupancyGrid msg.
 * @return True if setup is complete. False if something failed.
 */
bool ClassHybridAStar::setMap(nav_msgs::msg::OccupancyGrid *ptr_map_data)
{
    if (!m_gridmap_handler_ptr_->setGridMapPtr(&(ptr_map_data->data)))
        return false;
    if (!m_gridmap_handler_ptr_->setGridWidthHeight(ptr_map_data->info.width, ptr_map_data->info.height))
        return false;
    if (!m_gridmap_handler_ptr_->setPlanningObstacleThreshold(m_map_occ_threshold_.plan))
        return false;
    if (!m_gridmap_handler_ptr_->setValidateObstacleThreshold(m_map_occ_threshold_.vali))
        return false;
    if (!m_gridmap_handler_ptr_->setGridMeterRatio(ptr_map_data->info.resolution, m_parameters_.yaw_angle_bin_size))
        return false;
    m_gridmap_handler_ptr_->setOriginOffset(ptr_map_data->info.origin.position.x, ptr_map_data->info.origin.position.y);
    return true;
}

/**
 * @brief call this from external to set the start and goal poses.
 * @param startpose the pose where we want the path planning to start from. Usually this is the robot pose.
 * @param goalpose the pose where we want the path planning to finish.
 * @return Currently always returns true.
 */
bool ClassHybridAStar::setStartGoalPoses(StructPoseReal startpose, StructPoseReal goalpose)
{
    m_important_poses_.start_pose = startpose;
    m_important_poses_.goal_pose = goalpose;

    m_gridmap_handler_ptr_->convertFinePoseToGrid(m_important_poses_.start_pose, m_important_poses_.start_grid);
    m_gridmap_handler_ptr_->convertFinePoseToGrid(m_important_poses_.goal_pose, m_important_poses_.goal_grid);

    std::cout << std::setprecision(2) << std::fixed << "hybrid astar start state:\n"
                                          << m_important_poses_.start_pose.x << " " 
                                          << m_important_poses_.start_pose.y << " "
                                          << m_important_poses_.start_pose.yaw << "\n"
                                          << m_important_poses_.start_grid.x << " "
                                          << m_important_poses_.start_grid.y << " " 
                                          << m_important_poses_.start_grid.yaw << std::endl;

    std::cout << std::setprecision(2) << std::fixed << "hybrid astar goal state:\n"
                                          << m_important_poses_.goal_pose.x << " " 
                                          << m_important_poses_.goal_pose.y << " "
                                          << m_important_poses_.goal_pose.yaw << "\n"
                                          << m_important_poses_.goal_grid.x << " "
                                          << m_important_poses_.goal_grid.y << " " 
                                          << m_important_poses_.goal_grid.yaw << std::endl;
    return true;
}

/**
 * @brief call this from external to get ready for searching. Make sure the map and poses
 * were already set before calling this function.
 * @param timeout_ms the searching will terminates and considered as failure, if the total time
 * elapsed exceeds this value.
 * @return True if setup is complete and ready fro searching. False if something failed.
 */
bool ClassHybridAStar::setup()
{
    
    m_gridinfo_vector_.clear();
    m_gridinfo_vector_.resize(m_gridmap_handler_ptr_->getMapLength() * m_gridmap_handler_ptr_->m_number_of_angle_layers_);

    while (m_opennodes_pq_.size() > 0)
        m_opennodes_pq_.pop();

    m_result_path_.clear();
    m_path_RS_section_.clear();

    m_flags_.resetVals();
    m_rs_search_trigger_.resetVals();
    m_timer_and_counter_.resetVals();

    if (!checkStartAndGoalAccessible())
    {
        return false;
    }

    setupTheFirstGrid();

    std::cout << "ClassHybridAStar::setup() Done." << std::endl;

    return true;
}

/**
 * @brief This is the main function of this hybrid astar class. Call this after all the setup steps are
 * finished.
 * @return True if a valid path is found. False if the searching failed for any reason.
 */
bool ClassHybridAStar::search()
{
    std::cout << "ClassHybridAStar::search() start." << std::endl;

    ClassHawaTimer _timer;
    _timer.startNow();

    int _search_iteration_count = 0;
    while (!m_flags_.reach_goal)
    {
        // std_msgs::msg::Bool msg;
        // msg.data = true;
        // m_publisher_searching_->publish(msg);

        // check if there's any valid nodes to search.
        if (m_opennodes_pq_.size() == 0)
        {
            m_flags_.trapped = true;
            std::cerr << "ClassHybridAStar trapped. Tried " << _search_iteration_count << " times" << std::endl;
            return false; // there is no more OPEN node to search
        }

        exploreOneNode();

        _search_iteration_count++;

        if (m_flags_.found_rs_solution)
        {
            std::cout << "search success, found_rs_solution, tried " << _search_iteration_count << " times" << std::endl;
            break;
        }

        if (int(_timer.getDurationNonStop() * 1000.0) >= m_parameters_.time_out_ms)
        {
            // ROS_WARN_STREAM("ClassHybridAStar search timeout, failed, tried " << _search_iteration_count << " times");
            std::cerr << "ClassHybridAStar search timeout, failed, tried " << _search_iteration_count << " times" << std::endl;
            m_flags_.timeout = true;
            return false;
        }
    }

    _timer.endTiming();

    // ROS_INFO_STREAM("ClassHybridAStar::search() Found the goal." << int(_timer.getDuration() * 1000.0) << "ms. Tried " << _search_iteration_count << " iterations. RS_search "
    //                                                              << m_timer_and_counter_.count__rs_search_.getVal() << " times.");
    std::cout << "ClassHybridAStar::search() Found the goal." << int(_timer.getDuration() * 1000.0) << "ms. Tried " << _search_iteration_count << " iterations. RS_search "
              << m_timer_and_counter_.count__rs_search_.getVal() << " times." << std::endl;

    // ROS_DEBUG_STREAM("ClassHybridAStar::search() finish.");
    std::cout << "ClassHybridAStar::search() finish." << std::endl;

    return true;
}

/**
 * @brief This function will try to find if there is any ReedsShepp curve that will connect
 * the r_curr_grid and the goal pose, and also not collide with any obstacle.
 * @param r_curr_grid The pose where the RS curve should start. It's a reference variable.
 * @return True when a valid solution found. False for any other case.
 */
bool ClassHybridAStar::tryFindReedsSheppCurve(const StructPoseGrid &r_curr_grid)
{
    m_timer_and_counter_.count__rs_search_.increment();

    ClassHawaTimer _timer;
    _timer.startNow();

    try
    {
        int _curr_grid_index_1d = m_gridmap_handler_ptr_->convert3DTo1D(r_curr_grid.x, r_curr_grid.y, r_curr_grid.yaw);
        
        std::array<double, 3> _rs_start_pose_array = ClassUtilsConverters::cvtStructPoseReal2Array3( 
                                                        m_gridinfo_vector_.at(_curr_grid_index_1d).real_pose );
        
        std::array<double, 3> _rs_goal_pose_array = ClassUtilsConverters::cvtStructPoseReal2Array3( m_important_poses_.goal_pose );

        StructPoseReal _rs_start_pose(_rs_start_pose_array.at(0), _rs_start_pose_array.at(1), _rs_start_pose_array.at(2));
        StructPoseReal _rs_goal_pose(_rs_goal_pose_array.at(0), _rs_goal_pose_array.at(1), _rs_goal_pose_array.at(2));

        m_RS_curve_finder_.setup(_rs_start_pose, _rs_goal_pose);
        m_RS_curve_finder_.search();
    }
    catch (const std::exception &e)
    {
        // ROS_WARN_STREAM_NAMED("ClassHybridAStar::tryFindReedsSheppCurve", 
        //                       "m_RS_curve_finder_ try_catch exception:" << e.what());
        std::cerr << "ClassHybridAStar::tryFindReedsSheppCurve m_RS_curve_finder_ try_catch exception:" << e.what() << std::endl;
    }

    _timer.endTiming();
    m_timer_and_counter_.sec__rs_search += _timer.getDuration();

    if (m_RS_curve_finder_.m_vector_path_results_.size() == 0)
        return false;

    _timer.startNow();

    bool _found_usable_RS_path = checkRSSearchResult();
    if (_found_usable_RS_path)
    {
        m_important_poses_.grid_last_incremental_step = r_curr_grid;
    }

    _timer.endTiming();
    m_timer_and_counter_.sec__rs_collision_check += _timer.getDuration();

    return _found_usable_RS_path;
}

/**
 * @brief This is one post_processing step of tryFindReedsSheppCurve. This is where the
 * collision checking actually happens. It will loop through each RS curve and check if
 * any collision happens.
 * @return True when there is a valid curve. False when no any curve works.
 */
bool ClassHybridAStar::checkRSSearchResult()
{
    ClassHawaTimer _timer;
    _timer.startNow();

    bool _found_it = false;
    for (auto one_rs_path : m_RS_curve_finder_.m_vector_path_results_)
    {
        if (_found_it)
        {
            break;
        }
        if ((!one_rs_path.valid) || (one_rs_path.path_steps.size() <= 0))
        {
            continue;
        }

        bool _this_path_is_usable = true;
        m_path_RS_section_.clear();

        StructPoseGrid _nb_grid;
        StructPoseReal _nb_real;

        for (auto step : one_rs_path.path_steps)
        {
            if (!_this_path_is_usable)
                break;

            try
            {
                _nb_real.x = step[0];
                _nb_real.y = step[1];
                _nb_real.yaw = mod2pi(step[2]);

                m_gridmap_handler_ptr_->convertFinePoseToGrid(_nb_real, _nb_grid);

                if (m_gridmap_handler_ptr_->checkGridWithinMap(_nb_grid.x, _nb_grid.y))
                {
                    if (!m_gridmap_handler_ptr_->checkGridClear(_nb_grid.x, _nb_grid.y))
                    {
                        _this_path_is_usable = false;
                    }
                }
                m_path_RS_section_.push_back(_nb_real);
            }
            catch (const std::exception &e)
            {
                std::cerr << "ClassHybridAStar::checkRSSearchResult() exception:" << e.what() << std::endl;
            }
        }
        _found_it = _this_path_is_usable;
    }
    return _found_it;
}

/**
 * @brief This function should be called right before exploring the neighbour grids around
 * the currently interested grid. Some perliminary examination will happen and the open list
 * priority queue will be updated.
 * @param doable True if the exploration can start. False if it's impossible to explore.
 * @param grid_to_explore Store the information of this interested grid.
 */
void ClassHybridAStar::prepareForExplore(bool *doable, GridInfo *grid_to_explore)
{
    NodeinfoForPQ _curr_node = m_opennodes_pq_.top();

    int _curr_node_grid_1d = m_gridmap_handler_ptr_->convert3DTo1D(_curr_node.self_grid.x,
                                                                   _curr_node.self_grid.y,
                                                                   _curr_node.self_grid.yaw);
    if (m_gridinfo_vector_.at(_curr_node_grid_1d).grid_type != AStarGridType::Open)
    {
        if (m_opennodes_pq_.size())
        {
            m_opennodes_pq_.pop();
        }
        *doable = false;
    }
    else
    {
        *doable = true;
    }

    m_gridinfo_vector_.at(_curr_node_grid_1d).grid_type = AStarGridType::Closed;
    m_opennodes_pq_.pop();

    *grid_to_explore = m_gridinfo_vector_.at(_curr_node_grid_1d);
}

/**
 * @brief This is one of the key functions in this class. When this is called,
 * it will find the grid whose cost is the smallest in the open list, also
 * update the open list priority queue. Then the 8 neighour grids around this
 * grid will be updated too.
 */
void ClassHybridAStar::exploreOneNode()
{
    ClassHawaTimer _timer;
    _timer.startNow();

    bool _this_node_is_doable = false;
    GridInfo _current_grid;

    prepareForExplore(&_this_node_is_doable, &_current_grid);

    if (!_this_node_is_doable)
        return;

    if (m_rs_search_trigger_.check_match())
    {
        m_flags_.found_rs_solution = tryFindReedsSheppCurve(_current_grid.self_grid);
        if (m_flags_.found_rs_solution)
            return;
    }
    m_rs_search_trigger_.increment_counter();

    double _curr_theta = _current_grid.real_pose.yaw;
    double _cos_theta = std::cos(_curr_theta);
    double _sin_theta = std::sin(_curr_theta);

    for (auto mm : m_motion_model_.motions)
    {
        GridInfo _neigbr_grid;
        double _step_dx = _cos_theta * mm.dx + _sin_theta * mm.dy;
        double _step_dy = _sin_theta * mm.dx + _cos_theta * mm.dy;
        _neigbr_grid.real_pose.x = _current_grid.real_pose.x + _step_dx;
        _neigbr_grid.real_pose.y = _current_grid.real_pose.y + _step_dy;
        _neigbr_grid.real_pose.yaw = mod2pi(_curr_theta + mm.dyaw);
        // _neigbr_grid.steer_type = count;
        _neigbr_grid.motion_type = mm.motion_type;

        m_gridmap_handler_ptr_->convertFinePoseToGrid(_neigbr_grid.real_pose, _neigbr_grid.self_grid);

        bool _outside_of_map = !m_gridmap_handler_ptr_->checkGridWithinMap(_neigbr_grid.self_grid.x, _neigbr_grid.self_grid.y);
        bool _grid_inaccessible = !m_gridmap_handler_ptr_->checkGridClear(_neigbr_grid.self_grid.x, _neigbr_grid.self_grid.y);
        if (_outside_of_map || _grid_inaccessible)
        {
            continue;
        }

        int _neighbor_grid_index_1d = m_gridmap_handler_ptr_->convert3DTo1D(_neigbr_grid.self_grid.x,
                                                                       _neigbr_grid.self_grid.y,
                                                                       _neigbr_grid.self_grid.yaw);
        double _new_g_cost = _current_grid.gcost + 
                            mm.edge_cost * estimateChangingMotionTypeCost(_current_grid.motion_type, mm.motion_type);
        double _new_f_cost = _new_g_cost + computeHCostEuclidean(_neigbr_grid.real_pose, m_important_poses_.goal_pose);

        bool _is_new = m_gridinfo_vector_.at(_neighbor_grid_index_1d).grid_type == AStarGridType::NewGrid;
        bool _is_open = m_gridinfo_vector_.at(_neighbor_grid_index_1d).grid_type == AStarGridType::Open;
        bool _better_new_parent = _new_f_cost < m_gridinfo_vector_.at(_neighbor_grid_index_1d).fcost;

        if (_is_new || (_is_open && _better_new_parent))
        {
            m_gridinfo_vector_.at(_neighbor_grid_index_1d).grid_type = AStarGridType::Open;
            m_gridinfo_vector_.at(_neighbor_grid_index_1d).parent_grid = _current_grid.self_grid;
            m_gridinfo_vector_.at(_neighbor_grid_index_1d).self_grid = _neigbr_grid.self_grid;
            m_gridinfo_vector_.at(_neighbor_grid_index_1d).real_pose = _neigbr_grid.real_pose;
            m_gridinfo_vector_.at(_neighbor_grid_index_1d).motion_type = _neigbr_grid.motion_type;
            m_gridinfo_vector_.at(_neighbor_grid_index_1d).gcost = _new_g_cost;
            m_gridinfo_vector_.at(_neighbor_grid_index_1d).fcost = _new_f_cost;

            m_opennodes_pq_.push(NodeinfoForPQ(
                m_gridinfo_vector_.at(_neighbor_grid_index_1d).fcost,
                m_gridinfo_vector_.at(_neighbor_grid_index_1d).self_grid));
        }
    }
    _timer.endTiming();
    m_timer_and_counter_.sec__grid_search += _timer.getDuration();
}

/**
 * @brief call this from external to obtain the data of the result path. This should be called
 * after the searching finishes and returns true.
 * @param r_path the reference variable to store the data.
 */
void ClassHybridAStar::getFinalHybridAstarPath(ClassCustomPathContainer &r_path)
{
    // ROS_DEBUG_STREAM("ClassHybridAStar::getFinalHybridAstarPath() start.");
    std::cout << "ClassHybridAStar::getFinalHybridAstarPath() start." << std::endl;
    r_path.clear_points();

    StructPoseGrid temp_step = m_important_poses_.grid_last_incremental_step;
    if (m_flags_.trapped || m_flags_.timeout)
    {
        temp_step = m_important_poses_.min_cost_node;
    }

    while (temp_step != m_important_poses_.start_grid)
    {
        int _temp_step_index_1d = m_gridmap_handler_ptr_->convert3DTo1D(temp_step.x, temp_step.y, temp_step.yaw);
        std::array<double, 3> _temp_step_pose = ClassUtilsConverters::cvtStructPoseReal2Array3( m_gridinfo_vector_[_temp_step_index_1d].real_pose );

        r_path.pushfront(_temp_step_pose);
        temp_step = m_gridinfo_vector_[_temp_step_index_1d].parent_grid;

        if (temp_step.x == 0 && temp_step.y == 0)
        {
            break;
        }
    }
    int _temp_step_index_1d = m_gridmap_handler_ptr_->convert3DTo1D(temp_step.x, temp_step.y, temp_step.yaw);
    // this one should be the starting point.
    r_path.pushfront( ClassUtilsConverters::cvtStructPoseReal2Array3( m_gridinfo_vector_[_temp_step_index_1d].real_pose ) ); 

    if (!(m_flags_.trapped || m_flags_.timeout))
    {
        for (auto point : m_path_RS_section_)
        {
            r_path.pushback( ClassUtilsConverters::cvtStructPoseReal2Array3(point) );
        }
    }

    // ROS_DEBUG_STREAM("ClassHybridAStar::getFinalHybridAstarPath() Done.");
    std::cerr << "ClassHybridAStar::getFinalHybridAstarPath() Done." << std::endl;
}

/**
 * @brief Check if either the start grid or the goal grid is inside obstacles.
 * Return true when both grids are clear. Return false when either grid is not
 * accessible.
 */
bool ClassHybridAStar::checkStartAndGoalAccessible()
{
    bool _temp = true;

    if (!m_gridmap_handler_ptr_->checkGridClear(m_important_poses_.start_grid.x, m_important_poses_.start_grid.y))
    {
        std::cerr << "!! Start grid is in obstacle." << std::endl;
        _temp = false;
    }
    if (!m_gridmap_handler_ptr_->checkGridClear(m_important_poses_.goal_grid.x, m_important_poses_.goal_grid.y))
    {
        std::cerr << "!! Goal grid is in obstacle." << std::endl;
        _temp = false;
    }

    return _temp;
}

/**
 * @brief This is the last step during the setup of ClassHybridAStar. When the map data, robot
 * data and goal data are ready, this function would be called to initialize the containers for
 * the searching.
 */
void ClassHybridAStar::setupTheFirstGrid()
{
    int _start_grid_index_1d = m_gridmap_handler_ptr_->convert3DTo1D(m_important_poses_.start_grid.x,
                                                                     m_important_poses_.start_grid.y,
                                                                     m_important_poses_.start_grid.yaw);
    m_gridinfo_vector_.at(_start_grid_index_1d).fcost = 0;
    m_gridinfo_vector_.at(_start_grid_index_1d).gcost = 0;
    m_gridinfo_vector_.at(_start_grid_index_1d).grid_type = AStarGridType::Open;
    m_gridinfo_vector_.at(_start_grid_index_1d).parent_grid = m_important_poses_.start_grid;
    m_gridinfo_vector_.at(_start_grid_index_1d).real_pose = m_important_poses_.start_pose;
    m_gridinfo_vector_.at(_start_grid_index_1d).self_grid = m_important_poses_.start_grid;

    NodeinfoForPQ _start_grid_nodeinfo(
        m_gridinfo_vector_.at(_start_grid_index_1d).fcost,
        m_gridinfo_vector_.at(_start_grid_index_1d).self_grid);

    m_opennodes_pq_.push(_start_grid_nodeinfo);
}

}

