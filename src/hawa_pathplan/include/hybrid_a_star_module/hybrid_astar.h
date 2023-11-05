#ifndef HYBRID_ASTAR_H
#define HYBRID_ASTAR_H

#include <iostream>
#include <queue>
// #include <unordered_map>
// #include <map>
#include <math.h>

#include <algorithm>
#include <set>
#include <ros/package.h>

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"

// #include "../car_pathplan/struct_simple_pose.h"

#include "array_hasher.h"

#include "enum_fail_reasons.h"

#include "Reeds_Shepp_curve/reedsshepp.h"

#include "../car_pathplan/class_gridmap_handler.h"
#include "../car_pathplan/class_custom_path.h"

#include "../utils/hawa_data_containers.h"
#include "../utils/hawa_conversion_tools.h"
#include "../utils/hawa_tools.h"

#include "hybrid_astar_tools.h"
#include "struct_motion_primitives.h"

#include "class_astar_light_version.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::array;
using std::deque;
using std::queue;
using std::priority_queue;

class ClassHybridAStar
{
private:
    ClassGridMapHandler m_gridmap_handler_;

    ReedsSheppClass m_RS_curve_finder_;

    MotionModel m_motion_model_;

    vector<GridInfo> m_gridinfo_vector_;

    priority_queue<NodeinfoForPQ, vector<NodeinfoForPQ>, CostCompareMethod> m_opennodes_pq_;

    StructPoseGrid m_start_grid_;
    StructPoseReal m_start_pose_;

    StructPoseGrid m_goal_grid_;
    StructPoseReal m_goal_pose_;

    StructPoseGrid m_grid_last_incremental_step_;

    StructPoseGrid m_min_cost_node_;

    deque<StructPoseReal> m_result_path_;  // container for storing the final complete path.
    vector<StructPoseReal> m_path_RS_section_;  // container for storing the path segments found by ReedsShepp. 

    StructCounterForRSSearch m_rs_search_trigger_;

    int m_time_out_ms_;

    double m_dummy_min_cost_value_;

    // bool FLAG_reach_goal_;
    // bool FLAG_trapped_;
    // bool FLAG_inside_obstacle_;
    // bool FLAG_found_rs_solution_;
    // bool FLAG_timeout_;

    StructFlags m_flags_;

    TimingAndCounter m_timer_and_counter_;
    
    MapOccThreshold m_map_occ_threshold_;

    double m_yaw_angle_bin_size_;

private:

    bool checkStartAndGoalAccessible();

    void setupTheFirstGrid();

    void explore_one_node(); 

    double estimate_steering_cost(const int last_steer, const int new_steer);

    bool try_RS_curve(const StructPoseGrid &r_curr_grid); // try find some ReedsShepp curves from the given pose to goal.

    // void get_closest_path_to_goal(vector<StructPoseReal> &r_path); // when path-finding fails, then find an alternate path that is close to goal.


public:
    ClassHybridAStar();
    ~ClassHybridAStar();

    bool load_parameters();

    bool setMap(nav_msgs::OccupancyGrid* map_data);

    bool setStartGoalPoses(StructPoseReal startpose, StructPoseReal goalpose);

    bool setup(const int timeout_ms);

    bool search();

    // void get_path(std::deque<array<double, 3>> &r_path);
    void get_path(ClassCustomPathContainer &r_path);
};

/// @brief Constructor function. 
ClassHybridAStar::ClassHybridAStar()
{
    m_dummy_min_cost_value_ = std::numeric_limits<double>::max();

    load_parameters();

    m_timer_and_counter_.resetVals();

    m_motion_model_.prepare_model();

}

ClassHybridAStar::~ClassHybridAStar()
{
}

bool ClassHybridAStar::load_parameters()
{
    string _path = ros::package::getPath("car_pathplan");

    cout << "pkg path: >" << _path << "< " << endl;

    boost::property_tree::ptree root;
    boost::property_tree::read_json(_path + "/cfg/hybrid_astar_cfg.json", root);

    m_time_out_ms_ = root.get<int>("searching_time_limt_ms");

    m_motion_model_.parameters.turning_radius = root.get<double>("searching_turning_radius_meter");

    boost::property_tree::ptree step_length = root.get_child("searching_step_length_meter");
    m_motion_model_.parameters.step_length_forward = step_length.get<double>("forward");
    m_motion_model_.parameters.step_length_backward = step_length.get<double>("backward");

    boost::property_tree::ptree obstacle_threshold = root.get_child("gridmap_obstacle_threshold");
    m_map_occ_threshold_.plan = obstacle_threshold.get<int8_t>("planning");
    m_map_occ_threshold_.vali = obstacle_threshold.get<int8_t>("validate");

    m_rs_search_trigger_.interval = root.get<int>("reedshepp_every_n_nodes");

    m_yaw_angle_bin_size_ = root.get<double>("yaw_angle_grid_bin_size_rad");

    boost::property_tree::ptree action_cost_multipler = root.get_child("action_cost_multipler");
    m_motion_model_.cost_multipler.forward = action_cost_multipler.get<double>("forward_action");
    m_motion_model_.cost_multipler.backward = action_cost_multipler.get<double>("backward_action");
    m_motion_model_.cost_multipler.mild_steer = action_cost_multipler.get<double>("steer_action");
    m_motion_model_.cost_multipler.fast_steer = action_cost_multipler.get<double>("aggresive_steer");

    return true;
}


bool ClassHybridAStar::setMap(nav_msgs::OccupancyGrid* map_data)
{
    if (!m_gridmap_handler_.set_grid_map_ptr(&(map_data->data)))
        return false;
    if (!m_gridmap_handler_.set_grid_width_height(map_data->info.width, map_data->info.height))
        return false;
    if (!m_gridmap_handler_.set_planning_obstacle_threshold(m_map_occ_threshold_.plan))
        return false;
    if (!m_gridmap_handler_.set_validate_obstacle_threshold(m_map_occ_threshold_.vali))
        return false;
    if (!m_gridmap_handler_.set_grid_meter_ratio(map_data->info.resolution, m_yaw_angle_bin_size_))
        return false;
    return true;
}

bool ClassHybridAStar::setStartGoalPoses(StructPoseReal startpose, StructPoseReal goalpose)
{
    m_start_pose_ = startpose;
    m_goal_pose_ = goalpose;

    m_gridmap_handler_.convert_fine_pose_to_grid(m_start_pose_, m_start_grid_);
    m_gridmap_handler_.convert_fine_pose_to_grid(m_goal_pose_, m_goal_grid_);

    ROS_DEBUG_STREAM(std::setprecision(2) << std::fixed << "hybrid astar start state:\n" 
        << m_start_pose_.x << " "<< m_start_pose_.y << " "<< m_start_pose_.yaw << "\n"
        << m_start_grid_.x << " "<< m_start_grid_.y << " "<< m_start_grid_.yaw);

    ROS_DEBUG_STREAM(std::setprecision(2) << std::fixed << "hybrid astar goal state:\n" 
        << m_goal_pose_.x << " "<< m_goal_pose_.y << " "<< m_goal_pose_.yaw << "\n"
        << m_goal_grid_.x << " "<< m_goal_grid_.y << " "<< m_goal_grid_.yaw);
    
    return true;
}


bool ClassHybridAStar::setup(
    const int timeout_ms)
{

    // Reset the containers

    m_gridinfo_vector_.clear();
    m_gridinfo_vector_.resize( m_gridmap_handler_.getMapLength() * m_gridmap_handler_.m_number_of_angle_layers_);

    while (m_opennodes_pq_.size()>0)
        m_opennodes_pq_.pop();

    m_result_path_.clear();
    m_path_RS_section_.clear();
    
    m_flags_.resetVals();
    m_rs_search_trigger_.resetVals();
    m_timer_and_counter_.resetVals();

    if (! checkStartAndGoalAccessible())
    {
        return false;
    }

    setupTheFirstGrid();

    ROS_INFO_STREAM("ClassHybridAStar::setup() Done.");

    return true;
}

bool ClassHybridAStar::search()
{
    // cout << "ClassHybridAStar::search()  Looking for path " << endl;

    ROS_INFO_STREAM("ClassHybridAStar::search() start.");

    double time1 = helper_get_time();

    StructPoseGrid grid_pose;
    StructPoseReal fine_pose;

    double t1, t2;

    t1 = helper_get_time();

    int search_iteration_count = 0;
    while (!FLAG_reach_goal_)
    {
        // cout << "search_iteration_count " << search_iteration_count << endl;

        // check if there's any valid nodes to search.
        if (m_opennodes_pq_.size() == 0)
        {
            FLAG_trapped_ = true;
            ROS_WARN_STREAM("ClassHybridAStar search trapped, failed, tried " << search_iteration_count << " times" << endl);
            return false; // there is no OPEN node to search
        }

        explore_one_node();

        search_iteration_count++;

        if (FLAG_found_rs_solution_)
        {
            ROS_INFO_STREAM("search success, found_rs_solution, tried " << search_iteration_count << " times" << endl);
            break;
        }

        double time2 = helper_get_time();
        

        if( int((time2-time1)*1000.0) >= m_time_out_ms_ )
        {
            ROS_WARN_STREAM("ClassHybridAStar search timeout, failed, tried " << search_iteration_count << " times");
            FLAG_timeout_ = true;
            // find a path closest to the goal.
            return true;
        }

    }

    t2 = helper_get_time();

    ROS_INFO_STREAM("ClassHybridAStar::search() Found the goal." << int((t2 - t1)*1000.0) <<  
        "ms. Tried " << search_iteration_count << " iterations. RS_search " << m_timer_and_counter_.times__rs_search_
         << " times.");

    ROS_DEBUG_STREAM("ClassHybridAStar::search() finish.");

    return true;
}



bool ClassHybridAStar::try_RS_curve(const StructPoseGrid &curr_grid)
{
    m_timer_and_counter_.times__rs_search_ ++;

    double t1 = helper_get_time();
    try
    {
        int _curr_grid_index_1d = m_gridmap_handler_.convert_3D_to_oneD(curr_grid.x, curr_grid.y, curr_grid.yaw);
        // array<double, 3> rs_start_pose = m_clos_grids_[curr_grid].real_pose.to_array3();
        array<double, 3> rs_start_pose = m_gridinfo_vector_[_curr_grid_index_1d].real_pose.to_array3();
        array<double, 3> rs_goal_pose = m_goal_pose_.to_array3();

        // cout << "ClassHybridAStar::try_RS_curve() start " << rs_start_pose[0] << " " << rs_start_pose[1] << " " << rs_start_pose[2] << endl;
        // cout << "ClassHybridAStar::try_RS_curve() goal " << rs_goal_pose[0] << " " << rs_goal_pose[1] << " " << rs_goal_pose[2] << endl;

        m_RS_curve_finder_.setup(rs_start_pose, rs_goal_pose);
        m_RS_curve_finder_.search();

        if (m_RS_curve_finder_.results_.size() == 0)
        {
            // cout << "ClassHybridAStar::try_RS_curve() Found " << m_RS_curve_finder_.results_.size() << " RS paths" << endl;
            return false;
        }
        // cout << "ClassHybridAStar::try_RS_curve() Found " << m_RS_curve_finder_.results_.size()
        //           << "  RS paths. Now check collision of them." << endl;
    }
    catch (const std::exception &e)
    {
        cout << "ClassHybridAStar::try_RS_curve() Exception at 1." << endl;
        std::cerr << e.what() << '\n';
    }

    double t2 = helper_get_time();
    m_timer_and_counter_.sec__rs_search += (t2 - t1);

    t1 = helper_get_time();

    int try_count = -1;
    bool found_clear_RS_path = false;
    for (auto one_rs_path : m_RS_curve_finder_.results_)
    {
        try_count ++;
        // cout << "try count: " << try_count << endl;

        if (found_clear_RS_path)
        {
            break;
        }
        
        if (!one_rs_path.valid)
        {
            continue;
        }

        if (one_rs_path.path_steps.size() <= 0)
        {
            continue;
        }

        // cout << "check pass " << endl;

        bool path_is_clear = true;
        m_path_RS_section_.clear();

        StructPoseGrid nb_grid;
        StructPoseReal nb_real;

        for (auto step : one_rs_path.path_steps)
        {
            if (!path_is_clear)
            {
                // cout << "not path_is_clear " << endl;
                break;
            }

            try
            {
                nb_real.x = step[0];
                nb_real.y = step[1];
                nb_real.yaw = mod_2pi(step[2]);

                m_gridmap_handler_.convert_fine_pose_to_grid(nb_real, nb_grid);

                if (m_gridmap_handler_.check_grid_within_map(nb_grid.x, nb_grid.y))
                {
                    
                    bool step_is_clear = m_gridmap_handler_.check_grid_clear(nb_grid.x, nb_grid.y, ClassGridMapHandler::EnumMode::plan);
                    if (!step_is_clear)
                    {
                        // cout << "collide " << endl;
                        path_is_clear = false;
                    }
                    StructPoseReal a_point(step[0], step[1], step[2]);
                    m_path_RS_section_.push_back(a_point);
                }
                else
                {
                    m_path_RS_section_.push_back(nb_real);
                }
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
            }
        }
        if( path_is_clear )
        {
            found_clear_RS_path = true;
        }
        else
        {
            found_clear_RS_path = false;
        }
        // cout << "path_is_clear " << path_is_clear << endl;
    }

    if (found_clear_RS_path)
    {
        m_grid_last_incremental_step_ = curr_grid;
        // cout << "this grid is m_grid_last_incremental_step_  " << m_grid_last_incremental_step_.x << "  " << m_grid_last_incremental_step_.y << endl;
    }

    t2 = helper_get_time();
    m_timer_and_counter_.sec__rs_collision_check += (t2 - t1);

    // cout << "ClassHybridAStar::try_RS_curve() DONE " << endl;
    // cout << "found_clear_RS_path " << found_clear_RS_path << endl;
    
    return found_clear_RS_path;
}


void ClassHybridAStar::explore_one_node()
{
    cout << "explore_one_node()" << endl;

    double _t1_preparation = helper_get_time();
    double _t2_preparation;

    NodeinfoForPQ _curr_node = m_opennodes_pq_.top();

    int _curr_node_grid_1d = m_gridmap_handler_.convert_3D_to_oneD(_curr_node.self_grid.x, _curr_node.self_grid.y, _curr_node.self_grid.yaw);
    if (m_gridinfo_vector_[_curr_node_grid_1d].grid_type != AStarGridType::Open)
    {
        if (m_opennodes_pq_.size())
        {
            m_opennodes_pq_.pop();
        }
        // cout << "Top node in pq has unsafe type:" << m_gridinfo_vector_[_curr_node_grid_1d].grid_type << endl;
        _t2_preparation = helper_get_time();
        m_timer_and_counter_.sec__grid_search += (_t2_preparation - _t1_preparation);
        return;
    }

    // passed initial checking, so now change the type to Closed.
    m_gridinfo_vector_[_curr_node_grid_1d].grid_type = AStarGridType::Closed;
    m_opennodes_pq_.pop();

    GridInfo _current_grid = m_gridinfo_vector_[_curr_node_grid_1d];

    _t2_preparation = helper_get_time();
    m_timer_and_counter_.sec__grid_search += (_t2_preparation - _t1_preparation);
    
    if (m_counter_for_trigger_rs_search_ % m_interval_for_rs_search_ == 0)
    {
        if (try_RS_curve(_current_grid.self_grid))
        {
            FLAG_found_rs_solution_ = true;
            return;
        }
        else
        {
            FLAG_found_rs_solution_ = false;
        }
    }
    m_counter_for_trigger_rs_search_ ++;
    

    double _t1 = helper_get_time();

    double curr_theta = _current_grid.real_pose.yaw;
    double cos_theta = cos(curr_theta);
    double sin_theta = sin(curr_theta);

    // cout << "explore_one_node:: " << all_grids_[curr_grid].fine_pose[0] << " " << all_grids_[curr_grid].fine_pose[1] << " "
    //  << all_grids_[curr_grid].fine_pose[2] << endl;


    double step_dx, step_dy;
    int count = 0;
    GridInfo neigbr_grid;

    GridInfo _local_target_for_astar;
    bool _local_target_is_set = false;

    for (auto mm : m_motion_model_.motions)
    {
        step_dx = cos_theta * mm.dx + sin_theta * mm.dy;
        step_dy = sin_theta * mm.dx + cos_theta * mm.dy;
        neigbr_grid.real_pose.x = _current_grid.real_pose.x + step_dx;
        neigbr_grid.real_pose.y = _current_grid.real_pose.y + step_dy;
        neigbr_grid.real_pose.yaw = mod_2pi(curr_theta + mm.dyaw);
        neigbr_grid.steer_type = count;

        m_gridmap_handler_.convert_fine_pose_to_grid(neigbr_grid.real_pose, neigbr_grid.self_grid);

        // cout << "nb " << neigbr_grid.self_grid.x << " " << neigbr_grid.self_grid.y << " " << neigbr_grid.self_grid.yaw << " " << endl;

        // ignore this neighbour node if it is out of map or inside obstacle.
        if ( ! m_gridmap_handler_.check_grid_within_map(neigbr_grid.self_grid.x, neigbr_grid.self_grid.y))
        {
            count ++;
            continue;
        }
        
        if ( ! m_gridmap_handler_.check_grid_clear(neigbr_grid.self_grid.x, neigbr_grid.self_grid.y, ClassGridMapHandler::EnumMode::plan))
        {
            count ++;
            continue;
        }

        // cout << "valid grid." << endl;

        // cout << "m_open_grids_.begin() " << m_open_grids_.begin()->self_grid.x << " " << m_open_grids_.begin()->self_grid.y << " " << m_open_grids_.begin()->self_grid.yaw << " " << endl;
        
        int _neighbor_grid_index_1d = m_gridmap_handler_.convert_3D_to_oneD(neigbr_grid.self_grid.x, neigbr_grid.self_grid.y, neigbr_grid.self_grid.yaw);
        
        if (m_gridinfo_vector_[_neighbor_grid_index_1d].grid_type == AStarGridType::NewGrid)
        {
            // simply add this node into Open type.
            m_gridinfo_vector_[_neighbor_grid_index_1d].grid_type = AStarGridType::Open;
            m_gridinfo_vector_[_neighbor_grid_index_1d].parent_grid = _current_grid.self_grid;
            m_gridinfo_vector_[_neighbor_grid_index_1d].self_grid = neigbr_grid.self_grid;
            m_gridinfo_vector_[_neighbor_grid_index_1d].real_pose = neigbr_grid.real_pose;
            m_gridinfo_vector_[_neighbor_grid_index_1d].steer_type = count;
            double _new_g_cost = _current_grid.gcost + mm.edge_cost * estimate_steering_cost(_current_grid.steer_type, count);
            m_gridinfo_vector_[_neighbor_grid_index_1d].gcost = _new_g_cost;
            
            m_gridinfo_vector_[_neighbor_grid_index_1d].fcost = _new_g_cost + computeHCostEuclidean(m_gridinfo_vector_[_neighbor_grid_index_1d].real_pose, m_goal_pose_);

            m_opennodes_pq_.push(NodeinfoForPQ(
                m_gridinfo_vector_[_neighbor_grid_index_1d].fcost,
                m_gridinfo_vector_[_neighbor_grid_index_1d].self_grid
            ));
        }
        else if (m_gridinfo_vector_[_neighbor_grid_index_1d].grid_type == AStarGridType::Open)
        {
            double _new_g_cost = _current_grid.gcost + mm.edge_cost * estimate_steering_cost(_current_grid.steer_type, count);
            double _new_f_cost;
            
            _new_f_cost = _new_g_cost + computeHCostEuclidean(neigbr_grid.real_pose, m_goal_pose_);
            
            if (_new_f_cost < m_gridinfo_vector_[_neighbor_grid_index_1d].fcost)
            {
                m_gridinfo_vector_[_neighbor_grid_index_1d].grid_type = AStarGridType::Open;
                m_gridinfo_vector_[_neighbor_grid_index_1d].parent_grid = _current_grid.self_grid;
                m_gridinfo_vector_[_neighbor_grid_index_1d].self_grid = neigbr_grid.self_grid;
                m_gridinfo_vector_[_neighbor_grid_index_1d].real_pose = neigbr_grid.real_pose;
                m_gridinfo_vector_[_neighbor_grid_index_1d].steer_type = count;
                m_gridinfo_vector_[_neighbor_grid_index_1d].gcost = _new_g_cost;
                m_gridinfo_vector_[_neighbor_grid_index_1d].fcost = _new_f_cost;

                m_opennodes_pq_.push(NodeinfoForPQ(
                    m_gridinfo_vector_[_neighbor_grid_index_1d].fcost,
                    m_gridinfo_vector_[_neighbor_grid_index_1d].self_grid
                ));
            }
        }

        
        count++;
        // cout << "count  " << count << "   m_open_grids_.size() " << m_open_grids_.size() << endl;
        // auto open_iterator = m_open_grids_.begin();
        // cout << "grid " << (*open_iterator).self_grid.x << " " << (*open_iterator).self_grid.y << " " << (*open_iterator).self_grid.yaw << " " << endl;
        // cout << "" << endl;
    }
    double _t2 = helper_get_time();
    m_timer_and_counter_.sec__grid_search += (_t2 - _t1);
}

/// @brief call this from external to obtain the data of the result path.
/// @param r_path the reference variable to store the data. 
void ClassHybridAStar::get_path(ClassCustomPathContainer &r_path)
{
    ROS_DEBUG_STREAM("ClassHybridAStar::get_path() start.");
    r_path.clear_points();

    StructPoseGrid temp_step = m_grid_last_incremental_step_;
    if(FLAG_trapped_ || FLAG_timeout_)
    {
        temp_step = m_min_cost_node_;
    }

    while (temp_step != m_start_grid_)
    {
        // cout << m_clos_grids_[temp_step].real_pose.x << " ";
        // cout << m_clos_grids_[temp_step].real_pose.y << endl;
        // cout << "get_path(): temp: "  << temp_step.x << " ";
        // cout << temp_step.y << endl;

        int _temp_step_index_1d = m_gridmap_handler_.convert_3D_to_oneD(temp_step.x, temp_step.y, temp_step.yaw);
        array<double, 3> _temp_step_pose = m_gridinfo_vector_[_temp_step_index_1d].real_pose.to_array3();
        
        r_path.pushfront( _temp_step_pose );
        temp_step = m_gridinfo_vector_[_temp_step_index_1d].parent_grid;

        if(temp_step.x == 0 && temp_step.y == 0)
        {
            break;
        }
    }
    int _temp_step_index_1d = m_gridmap_handler_.convert_3D_to_oneD(temp_step.x, temp_step.y, temp_step.yaw);
    r_path.pushfront( m_gridinfo_vector_[_temp_step_index_1d].real_pose.to_array3() );  // this one should be the starting point.


    if( !(FLAG_trapped_ || FLAG_timeout_))
    {
        for( auto point : m_path_RS_section_ )
        {
            r_path.pushback(point.to_array3());
        }
    }

    ROS_DEBUG_STREAM("ClassHybridAStar::get_path() Done.");
}


bool ClassHybridAStar::checkStartAndGoalAccessible()
{   
    bool _temp = true;

    if( ! m_gridmap_handler_.check_grid_clear(m_start_grid_.x, m_start_grid_.y, ClassGridMapHandler::EnumMode::plan))
    {
        ROS_WARN_STREAM("!! Start grid is in obstacle.");
        _temp = false;
    }

    if( ! m_gridmap_handler_.check_grid_clear(m_goal_grid_.x, m_goal_grid_.y, ClassGridMapHandler::EnumMode::plan))
    {
        ROS_WARN_STREAM("!! Goal grid is in obstacle.");
        _temp = false;
    }

    return _temp;
}


void ClassHybridAStar::setupTheFirstGrid()
{
    int _start_grid_index_1d = m_gridmap_handler_.convert_3D_to_oneD(m_start_grid_.x, 
                                                                     m_start_grid_.y, 
                                                                     m_start_grid_.yaw);
    m_gridinfo_vector_.at(_start_grid_index_1d).fcost = 0;
    m_gridinfo_vector_.at(_start_grid_index_1d).gcost = 0;
    m_gridinfo_vector_.at(_start_grid_index_1d).grid_type = AStarGridType::Open;
    m_gridinfo_vector_.at(_start_grid_index_1d).parent_grid = m_start_grid_;
    m_gridinfo_vector_.at(_start_grid_index_1d).real_pose = m_start_pose_;
    m_gridinfo_vector_.at(_start_grid_index_1d).self_grid = m_start_grid_;

    NodeinfoForPQ _start_grid_nodeinfo(
        m_gridinfo_vector_.at(_start_grid_index_1d).fcost,
        m_gridinfo_vector_.at(_start_grid_index_1d).self_grid
    );

    m_opennodes_pq_.push(_start_grid_nodeinfo);
}

#endif
