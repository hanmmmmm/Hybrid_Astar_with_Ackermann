
#ifndef HYBRID_ASTAR_H
#define HYBRID_ASTAR_H

#include <iostream>
#include <queue>
#include <unordered_map>
#include <map>
#include <math.h>

#include <algorithm>
#include <set>
#include <chrono>

#include <ros/package.h>

#include "../car_pathplan/struct_simple_pose.h"

#include "array_hasher.h"

#include "enum_fail_reasons.h"

#include "Reeds_Shepp_curve/reedsshepp.h"

#include "../car_pathplan/class_gridmap_handler.h"
#include "../car_pathplan/class_custom_path.h"

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

using std::chrono::high_resolution_clock;

namespace pt = boost::property_tree;

class ClassHybridAStar
{
public:

    enum AStarGridType
    {
        NewGrid,
        Open,
        Closed
    };

    struct GridInfo
    {
        double gcost = 0;
        double fcost = 0;
        int steer_type = 0; // 0-5, total 6 types
        StructPoseReal real_pose;
        StructPoseGrid parent_grid;
        StructPoseGrid self_grid;
        AStarGridType grid_type = AStarGridType::NewGrid;

        GridInfo()
        {
        };
    };

    struct NodeinfoForPQ
    {
        double cost;
        StructPoseGrid self_grid;
        NodeinfoForPQ(double cost, StructPoseGrid self_grid)
        :cost(cost), self_grid(self_grid)
        {
        }
    };

    struct CostCompareMethod
    {
        bool operator()(NodeinfoForPQ const& p1, NodeinfoForPQ const& p2)
        {
            return p1.cost > p2.cost;  // smallest will be at the top
        }
    };
    
    enum EnumForwardBackward
    {
        forward,
        backward
    };

    struct MotionPrimitive
    {
        double dx;
        double dy;
        double dyaw;
        double edge_cost;
        EnumForwardBackward direction;
    };

    enum EnumResultState
    {
        searching,
        success__find_path_by_increment,
        success__find_path_by_ReedsShepp,
        failed__trapped__use_closest,
        failed__timeout__use_closest
    };

    struct CostMultipler
    {
        double forward = 1.0;
        double backward = 1.0;
        double mild_steer = 1.0;
        double fast_steer = 1.0;
    };
    

private:
    ClassGridMapHandler m_gridmap_handler_;

    ReedsSheppClass m_RS_curve_finder_;

    ClassAStar m_astar_finder_;

    vector<MotionPrimitive> m_motion_model_;

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

    CostMultipler m_costmultipler_;

private:
    int m_counter_for_trigger_rs_search_;
    int m_interval_for_rs_search_;

    int m_time_out_ms_;

    double m_dummy_min_cost_value_;

    double m_step_length_forward_;
    double m_step_length_backward_;
    double m_turning_raius_;
    double m_turning_angle_;

    bool m_use_astar_as_hcost_;

    bool FLAG_reach_goal_;
    bool FLAG_trapped_;
    bool FLAG_inside_obstacle_;
    bool FLAG_found_rs_solution_;
    bool FLAG_timeout_;

    double m_debug_rs_search_time_;
    double m_debug_grid_search_time_;
    double m_debug_rs_collision_check_time_;
    double m_debug_astar_search_time_;

    int m_debug_times_find_min_cost_;
    int m_debug_times_rs_search_;
    

    int8_t m_plan_threshold_;
    int8_t m_vali_threshold_;

    double m_yaw_angle_bin_size_;

private:

    void build_motion_model();

    double mod_2pi(double angle);

    void explore_one_node(); 

    double estimate_steering_cost(const int last_steer, const int new_steer);

    bool try_RS_curve(const StructPoseGrid &r_curr_grid); // try find some ReedsShepp curves from the given pose to goal.

    // void get_closest_path_to_goal(vector<StructPoseReal> &r_path); // when path-finding fails, then find an alternate path that is close to goal.

    double compute_h_cost_Euclidean(const StructPoseReal n, const StructPoseReal g);
    double compute_h_cost_Manhattan(const StructPoseReal n, const StructPoseReal g);
    double compute_h_cost_Chebyshev(const StructPoseReal n, const StructPoseReal g);
    double compute_h_cost_AStar(const StructPoseReal &n, const StructPoseReal &g);

    double helper_get_time();

public:
    ClassHybridAStar();
    ~ClassHybridAStar();

    bool load_parameters();

    bool setup(const int timeout_ms,
               const array<double, 3> startpose,
               const array<double, 3> goalpose,
               const int grid_map_width,
               const int grid_map_height,
               vector<int8_t> &r_gridmap,
               const double grid_resolution);

    bool search();

    // void get_path(std::deque<array<double, 3>> &r_path);
    void get_path(ClassCustomPathContainer &r_path);
};

ClassHybridAStar::ClassHybridAStar()
{
    m_dummy_min_cost_value_ = std::numeric_limits<double>::max();

    m_use_astar_as_hcost_ = false;

    load_parameters();

    build_motion_model();

    m_debug_rs_search_time_ = 0.0;
    m_debug_grid_search_time_ = 0.0;
    m_debug_rs_collision_check_time_ = 0.0;
    m_debug_times_find_min_cost_ = 0;
    m_debug_times_rs_search_ = 0;
    m_debug_astar_search_time_ = 0;
}

ClassHybridAStar::~ClassHybridAStar()
{
}

bool ClassHybridAStar::load_parameters()
{

    string _path = ros::package::getPath("car_pathplan");

    cout << "pkg path: >" << _path << "< " << endl;

    pt::ptree root;
    pt::read_json(_path + "/cfg/hybrid_astar_cfg.json", root);

    m_time_out_ms_ = root.get<int>("searching_time_limt_ms");

    m_turning_raius_ = root.get<double>("searching_turning_radius_meter");

    pt::ptree step_length = root.get_child("searching_step_length_meter");
    m_step_length_forward_ = step_length.get<double>("forward");
    m_step_length_backward_ = step_length.get<double>("backward");

    pt::ptree obstacle_threshold = root.get_child("gridmap_obstacle_threshold");
    m_plan_threshold_ = obstacle_threshold.get<int8_t>("planning");
    m_vali_threshold_ = obstacle_threshold.get<int8_t>("validate");

    m_interval_for_rs_search_ = root.get<int>("reedshepp_every_n_nodes");

    m_yaw_angle_bin_size_ = root.get<double>("yaw_angle_grid_bin_size_rad");

    pt::ptree action_cost_multipler = root.get_child("action_cost_multipler");
    m_costmultipler_.forward = action_cost_multipler.get<double>("forward_action");
    m_costmultipler_.backward = action_cost_multipler.get<double>("backward_action");
    m_costmultipler_.mild_steer = action_cost_multipler.get<double>("steer_action");
    m_costmultipler_.fast_steer = action_cost_multipler.get<double>("aggresive_steer");

    pt::ptree h_cost_options = root.get_child("h_cost_options");

    m_use_astar_as_hcost_ = h_cost_options.get<bool>("use_astar_as_hcost");

    int _astar_step_size = h_cost_options.get<int>("astar_step_size_int");

    m_astar_finder_.set_step_size(_astar_step_size);

    return true;
}


bool ClassHybridAStar::setup(
    const int timeout_ms,
    const array<double, 3> startpose,
    const array<double, 3> goalpose,
    const int map_width_grid,
    const int map_height_grid,
    vector<int8_t> &map,
    const double grid_resolution)
{

    // cout << "ClassHybridAStar::setup() START" << endl;

    // m_time_out_ms_ = timeout_ms;

    // update the gridmap using the latest message.

    if (!m_gridmap_handler_.set_grid_map_ptr(&map))
        return false;
    if (!m_gridmap_handler_.set_grid_width_height(map_width_grid, map_height_grid))
        return false;
    if (!m_gridmap_handler_.set_planning_obstacle_threshold(m_plan_threshold_))
        return false;
    if (!m_gridmap_handler_.set_validate_obstacle_threshold(m_vali_threshold_))
        return false;
    if (!m_gridmap_handler_.set_grid_meter_ratio(grid_resolution, m_yaw_angle_bin_size_))
        return false;

    m_astar_finder_.setup_map(map_width_grid, map_height_grid, map, grid_resolution, m_plan_threshold_);


    // Clean the containers

    m_gridinfo_vector_.clear();
    m_gridinfo_vector_.resize(map.size()*m_gridmap_handler_.m_number_of_angle_layers_);

    while (m_opennodes_pq_.size()>0)
    {
        m_opennodes_pq_.pop();
    }

    m_result_path_.clear();
    m_path_RS_section_.clear();
    

    // update values of the start node and goal node

    m_start_pose_.x = startpose[0];
    m_start_pose_.y = startpose[1];
    m_start_pose_.yaw = startpose[2];

    m_goal_pose_.x = goalpose[0];
    m_goal_pose_.y = goalpose[1];
    m_goal_pose_.yaw = goalpose[2];

    m_gridmap_handler_.convert_fine_pose_to_grid(m_start_pose_, m_start_grid_);
    m_gridmap_handler_.convert_fine_pose_to_grid(m_goal_pose_, m_goal_grid_);

    // cout << "start_grid_ " << endl;
    // cout << start_grid_[0] << " " << start_grid_[1] << " " << start_grid_[2] << endl;
    // cout << start_pose_[0] << " " << start_pose_[1] << " " << start_pose_[2] << endl;

    // cout << "goal_grid_ " << endl;
    // cout << goal_grid_[0] << " " << goal_grid_[1] << " " << goal_grid_[2] << endl;
    // cout << goal_pose_[0] << " " << goal_pose_[1] << " " << goal_pose_[2] << endl;


    // check if the start node or goal node are reachable. If not, then do not proceed.

    if( ! m_gridmap_handler_.check_grid_clear(m_start_grid_.x, m_start_grid_.y, ClassGridMapHandler::EnumMode::plan))
    {
        cout << "!!! Start grid is on obstacle." << endl;
        return false;
    }

    if( ! m_gridmap_handler_.check_grid_clear(m_goal_grid_.x, m_goal_grid_.y, ClassGridMapHandler::EnumMode::plan))
    {
        cout << "!!! Goal grid is on obstacle." << endl;
        return false;
    }

    // if the checking passed, then update the start node data, and insert it into open_node_priority_queue.

    int _start_grid_index_1d = m_gridmap_handler_.convert_3D_to_oneD(m_start_grid_.x, m_start_grid_.y, m_start_grid_.yaw);
    m_gridinfo_vector_[_start_grid_index_1d].fcost = 0;
    m_gridinfo_vector_[_start_grid_index_1d].gcost = 0;
    m_gridinfo_vector_[_start_grid_index_1d].grid_type = AStarGridType::Open;
    m_gridinfo_vector_[_start_grid_index_1d].parent_grid = m_start_grid_;
    m_gridinfo_vector_[_start_grid_index_1d].real_pose = m_start_pose_;
    m_gridinfo_vector_[_start_grid_index_1d].self_grid = m_start_grid_;

    NodeinfoForPQ _start_grid_nodeinfo(
        m_gridinfo_vector_[_start_grid_index_1d].fcost,
        m_gridinfo_vector_[_start_grid_index_1d].self_grid
    );

    m_opennodes_pq_.push(_start_grid_nodeinfo);

    // cout << "PQ size: " << m_opennodes_pq_.size() << endl;

    // reset the flags, counters, timers.

    FLAG_reach_goal_ = false;
    FLAG_trapped_ = false;
    FLAG_found_rs_solution_ = false;
    FLAG_timeout_ = false;

    m_counter_for_trigger_rs_search_ = 0;
    

    m_debug_rs_search_time_ = 0.0;
    m_debug_grid_search_time_ = 0.0;
    m_debug_rs_collision_check_time_ = 0.0;
    m_debug_times_find_min_cost_ = 0;
    m_debug_times_rs_search_ = 0;
    m_debug_astar_search_time_ = 0;

    // cout << "m_start_grid_ " << m_start_grid_.x << " " << m_start_grid_.y << endl;

    // cout << "ClassHybridAStar::setup() DONE" << endl;

    return true;
}

bool ClassHybridAStar::search()
{
    // cout << "ClassHybridAStar::search()  Looking for path " << endl;

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
            std::cerr << "ClassHybridAStar search trapped, failed, tried " << search_iteration_count << " times" << endl;
            return false; // there is no OPEN node to search
        }

        explore_one_node();

        search_iteration_count++;

        if (FLAG_found_rs_solution_)
        {
            cout << "search success, found_rs_solution, tried " << search_iteration_count << " times" << endl;
            break;
        }

        double time2 = helper_get_time();
        

        if( int((time2-time1)*1000.0) >= m_time_out_ms_ )
        {
            std::cerr << "ClassHybridAStar search timeout, failed, tried " << search_iteration_count << " times" << endl;
            FLAG_timeout_ = true;
            // find a path closest to the goal.
            return true;
        }

    }

    t2 = helper_get_time();

    cout << "                                  " ;
    cout << "Time in ms: " << int(m_debug_grid_search_time_*1000.0) << ", " ;
    cout << int(m_debug_rs_search_time_*1000.0) << ", " << int(m_debug_rs_collision_check_time_*1000.0) << ", " << int(m_debug_astar_search_time_*1000.0) << endl;

    // cout << "                                  " ;
    // cout << "Time in ms: " << int(m_debug_grid_search_time_*1000.0) + 
    //                                 int(m_debug_rs_search_time_*1000.0) + int(m_debug_rs_collision_check_time_*1000.0) << endl;
    
    cout << "                                  " ;
    cout << "Time ClassHybridAStar::search() in ms: " << int((t2 - t1)*1000.0) << endl;


    cout << "rs_search " << m_debug_times_rs_search_ << " times" << endl;

    cout << "Found the goal. Tried " << search_iteration_count << " iterations " << endl;

    return true;
}



bool ClassHybridAStar::try_RS_curve(const StructPoseGrid &curr_grid)
{
    m_debug_times_rs_search_ ++;

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
    m_debug_rs_search_time_ += (t2 - t1);

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
    m_debug_rs_collision_check_time_ += (t2 - t1);

    // cout << "ClassHybridAStar::try_RS_curve() DONE " << endl;
    // cout << "found_clear_RS_path " << found_clear_RS_path << endl;
    
    return found_clear_RS_path;
}


void ClassHybridAStar::explore_one_node()
{
    // cout << "explore_one_node()" << endl;

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
        m_debug_grid_search_time_ += (_t2_preparation - _t1_preparation);
        return;
    }

    // passed initial checking, so now change the type to Closed.
    m_gridinfo_vector_[_curr_node_grid_1d].grid_type = AStarGridType::Closed;
    m_opennodes_pq_.pop();

    GridInfo _current_grid = m_gridinfo_vector_[_curr_node_grid_1d];

    _t2_preparation = helper_get_time();
    m_debug_grid_search_time_ += (_t2_preparation - _t1_preparation);
    
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

    for (auto mm : m_motion_model_)
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
            if (! m_use_astar_as_hcost_)
            {
                m_gridinfo_vector_[_neighbor_grid_index_1d].fcost = _new_g_cost + compute_h_cost_Euclidean(m_gridinfo_vector_[_neighbor_grid_index_1d].real_pose, m_goal_pose_);
            }
            else
            {
                if ( ! _local_target_is_set)
                {
                    m_gridinfo_vector_[_neighbor_grid_index_1d].fcost = _new_g_cost + compute_h_cost_AStar(m_gridinfo_vector_[_neighbor_grid_index_1d].real_pose, m_goal_pose_);
                    _local_target_is_set = true;
                    _local_target_for_astar = m_gridinfo_vector_[_neighbor_grid_index_1d];
                }
                else
                {
                    double _local_target_hcost = _local_target_for_astar.fcost - _local_target_for_astar.gcost;
                    double _hcost_from_beibor_to_local_target = compute_h_cost_AStar(m_gridinfo_vector_[_neighbor_grid_index_1d].real_pose, _local_target_for_astar.real_pose);
                    double _h_cost = _hcost_from_beibor_to_local_target + _local_target_hcost;
                    m_gridinfo_vector_[_neighbor_grid_index_1d].fcost = _new_g_cost + _h_cost;
                }
            }
            


            m_opennodes_pq_.push(NodeinfoForPQ(
                m_gridinfo_vector_[_neighbor_grid_index_1d].fcost,
                m_gridinfo_vector_[_neighbor_grid_index_1d].self_grid
            ));
        }
        else if (m_gridinfo_vector_[_neighbor_grid_index_1d].grid_type == AStarGridType::Open)
        {
            double _new_g_cost = _current_grid.gcost + mm.edge_cost * estimate_steering_cost(_current_grid.steer_type, count);
            double _new_f_cost;
            if (! m_use_astar_as_hcost_)
            {
                _new_f_cost = _new_g_cost + compute_h_cost_Euclidean(neigbr_grid.real_pose, m_goal_pose_);
            }
            else
            {
                // _new_f_cost = _new_g_cost + compute_h_cost_AStar(neigbr_grid.real_pose, m_goal_pose_);

                if ( ! _local_target_is_set)
                {
                   _new_f_cost = _new_g_cost + compute_h_cost_AStar(neigbr_grid.real_pose, m_goal_pose_);
                    _local_target_is_set = true;
                    _local_target_for_astar = m_gridinfo_vector_[_neighbor_grid_index_1d];
                }
                else
                {
                    double _local_target_hcost = _local_target_for_astar.fcost - _local_target_for_astar.gcost;
                    double _hcost_from_beibor_to_local_target = compute_h_cost_AStar(m_gridinfo_vector_[_neighbor_grid_index_1d].real_pose, _local_target_for_astar.real_pose);
                    double _h_cost = _hcost_from_beibor_to_local_target + _local_target_hcost;
                    _new_f_cost = _new_g_cost + _h_cost;
                }

            }
            

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
    m_debug_grid_search_time_ += (_t2 - _t1);
}

void ClassHybridAStar::get_path(ClassCustomPathContainer &r_path)
{
    cout << "ClassHybridAStar::get_path()" << endl;
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

    cout << "ClassHybridAStar::get_path() Done." << endl;
}

void ClassHybridAStar::build_motion_model()
{
    MotionPrimitive mp;
    double raius, angle, curve_dx, curve_dy;

    m_turning_angle_ = m_step_length_forward_ / m_turning_raius_;

    raius = m_turning_raius_;
    angle = std::abs(m_turning_angle_);

    curve_dx = raius * sin(angle);
    curve_dy = raius * (1.0 - cos(angle));


    // first 3 options are moving forward
    mp.edge_cost = m_step_length_forward_ * m_costmultipler_.forward;
    mp.direction = EnumForwardBackward::forward;

    // to front left
    mp.dx = curve_dx;
    mp.dy = curve_dy;
    mp.dyaw = angle;
    m_motion_model_.push_back(mp);

    // to front center
    mp.dx = m_step_length_forward_;
    mp.dy = 0;
    mp.dyaw = 0;
    m_motion_model_.push_back(mp);

    // to front right
    mp.dx = curve_dx;
    mp.dy = -curve_dy;
    mp.dyaw = -angle;
    m_motion_model_.push_back(mp);


    m_turning_angle_ = m_step_length_backward_ / m_turning_raius_;

    raius = m_turning_raius_;
    angle = std::abs(m_turning_angle_);

    curve_dx = raius * sin(angle);
    curve_dy = raius * (1.0 - cos(angle));

    // next 3 options are moving backward
    mp.edge_cost = m_step_length_backward_ * m_costmultipler_.backward;
    mp.direction = EnumForwardBackward::backward;

    // to back right
    mp.dx = -curve_dx;
    mp.dy = -curve_dy;
    mp.dyaw = angle;
    m_motion_model_.push_back(mp);

    // to back center
    mp.dx = -m_step_length_backward_;
    mp.dy = 0;
    mp.dyaw = 0;
    m_motion_model_.push_back(mp);

    // to back left
    mp.dx = -curve_dx;
    mp.dy = curve_dy;
    mp.dyaw = -angle;
    m_motion_model_.push_back(mp);
}


double ClassHybridAStar::estimate_steering_cost(const int last_steer, const int new_steer)
{
    int _diff = std::abs(last_steer - new_steer);
    if (_diff == 0)
    {
        return 1.0;
    }
    else if (_diff == 1)
    {
        return 1.3;
    }
    else if (_diff > 0)
    {
        return 2.0;
    }
}


inline double ClassHybridAStar::mod_2pi(double a)
{
    double angle = a;
    while (angle > 2 * M_PI)
    {
        angle -= 2 * M_PI;
    }
    while (angle < 0)
    {
        angle += 2 * M_PI;
    }
    return angle;
}

inline double ClassHybridAStar::compute_h_cost_Euclidean(const StructPoseReal n, const StructPoseReal g)
{
    double dx = n.x - g.x;
    double dy = n.y - g.y;
    return sqrt(dx * dx + dy * dy);
}

inline double ClassHybridAStar::compute_h_cost_Manhattan(const StructPoseReal n, const StructPoseReal g)
{
    double dx = abs(n.x - g.x);
    double dy = abs(n.y - g.y);
    return dx + dy;
}

inline double ClassHybridAStar::compute_h_cost_Chebyshev(const StructPoseReal n, const StructPoseReal g)
{
    double dx = abs(n.x - g.x);
    double dy = abs(n.y - g.y);
    return std::max(dx, dy);
}

inline double ClassHybridAStar::compute_h_cost_AStar(const StructPoseReal &n, const StructPoseReal &g)
{
    double _t1 = helper_get_time();
    m_astar_finder_.setup_points(array<double, 2>{n.x, n.y}, array<double, 2>{g.x, g.y});
    if (m_astar_finder_.search())
    {
        double _t2 = helper_get_time();
        // cout << "- " << (_t2 - _t1) << "    " << m_debug_astar_search_time_ << endl;
        m_debug_astar_search_time_ += (_t2 - _t1);
        return m_astar_finder_.get_path_length_meter();
    }
    else
    {
        double _t2 = helper_get_time();
        // cout << "- " << (_t2 - _t1) << "    " << m_debug_astar_search_time_ << endl;
        m_debug_astar_search_time_ += (_t2 - _t1);
        return 999.0;
    }
}



/**
 * @brief 
 * return time in seconds, epoch time. 
 */
double ClassHybridAStar::helper_get_time()
{
    return high_resolution_clock::now().time_since_epoch().count()/1000000000.0; 
}




#endif
