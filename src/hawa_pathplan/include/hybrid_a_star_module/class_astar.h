
#ifndef ASTAR_H
#define ASTAR_H

#include <iostream>
#include <queue>
#include <unordered_map>
#include <map>
#include <math.h>

#include <algorithm>
#include <set>
#include <chrono>

// #include <ros/package.h>

#include "../car_pathplan/struct_simple_pose.h"

// #include "enum_fail_reasons.h"

#include "../car_pathplan/class_gridmap_handler.h"

using std::cout;
using std::endl;

using std::string;

using std::vector;
using std::array;
using std::deque;
using std::queue;
using std::priority_queue;

using std::chrono::high_resolution_clock;

// namespace pt = boost::property_tree;

class ClassAStar
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
        int gcost = 0;
        int fcost = 0;
        StructPoseGrid parent_grid;
        StructPoseGrid self_grid;
        AStarGridType grid_type = AStarGridType::NewGrid;

        GridInfo()
        {
        };
    };

    struct NodeinfoForPQ
    {
        int cost;
        StructPoseGrid self_grid;
        NodeinfoForPQ(int cost, StructPoseGrid self_grid)
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
    

    struct MotionPrimitive
    {
        int dx;
        int dy;
        int edge_cost;
    };

    // enum EnumResultState
    // {
    //     searching,
    //     success__find_path_by_increment,
    //     success__find_path_by_ReedsShepp,
    //     failed__trapped__use_closest,
    //     failed__timeout__use_closest
    // };

    

private:
    ClassGridMapHandler m_gridmap_handler_;

    vector<MotionPrimitive> m_motion_model_;

    vector<GridInfo> m_gridinfo_vector_;

    priority_queue<NodeinfoForPQ, vector<NodeinfoForPQ>, CostCompareMethod> m_opennodes_pq_;

    StructPoseGrid m_start_grid_;

    StructPoseGrid m_goal_grid_;

    deque<StructPoseReal> m_result_path_;  // container for storing the final complete path.


private:

    int m_time_out_ms_;

    double m_dummy_min_cost_value_;

    bool FLAG_reach_goal_;
    bool FLAG_trapped_;
    bool FLAG_inside_obstacle_;
    bool FLAG_timeout_;

    double m_debug_grid_search_time_;
    int8_t m_plan_threshold_;

    int m_step_length;

private:

    void build_motion_model();

    void explore_one_node(); 

    int compute_h_cost_Euclidean(const StructPoseGrid n, const StructPoseGrid g);
    int compute_h_cost_Manhattan(const StructPoseGrid n, const StructPoseGrid g);
    int compute_h_cost_Chebyshev(const StructPoseGrid n, const StructPoseGrid g);

    double helper_get_time();

public:
    ClassAStar();
    ~ClassAStar();

    bool load_parameters();

    bool setup(const int timeout_ms,
               const array<double, 3> startpose,
               const array<double, 3> goalpose,
               const int grid_map_width,
               const int grid_map_height,
               vector<int8_t> &r_gridmap,
               const double grid_resolution);

    bool search();

    void get_path(std::deque<array<int, 2>> &r_path);
    void get_path_in_meter(std::deque<array<double, 2>> &r_path);
};

ClassAStar::ClassAStar()
{
    m_dummy_min_cost_value_ = std::numeric_limits<double>::max();

    load_parameters();

    build_motion_model();

    m_debug_grid_search_time_ = 0.0;

}

ClassAStar::~ClassAStar()
{
}

bool ClassAStar::load_parameters()
{

    // string _path = ros::package::getPath("car_pathplan");

    // cout << "pkg path: >" << _path << "< " << endl;

    // pt::ptree root;
    // pt::read_json(_path + "/cfg/hybrid_astar_cfg.json", root);

    // m_time_out_ms_ = root.get<int>("searching_time_limt_ms");

    // pt::ptree obstacle_threshold = root.get_child("gridmap_obstacle_threshold");
    // m_plan_threshold_ = obstacle_threshold.get<int8_t>("planning");
    // m_vali_threshold_ = obstacle_threshold.get<int8_t>("validate");

    m_step_length = 3;

    return true;
}


bool ClassAStar::setup(
    const int timeout_ms,
    const array<double, 3> startpose,
    const array<double, 3> goalpose,
    const int map_width_grid,
    const int map_height_grid,
    vector<int8_t> &map,
    const double grid_resolution)
{

    // cout << "ClassAStar::setup() START" << endl;

    m_time_out_ms_ = timeout_ms;

    // update the gridmap using the latest message.

    if (!m_gridmap_handler_.set_grid_map_ptr(&map))
        return false;
    if (!m_gridmap_handler_.set_grid_width_height(map_width_grid, map_height_grid))
        return false;
    if (!m_gridmap_handler_.set_planning_obstacle_threshold(m_plan_threshold_))
        return false;
    if (!m_gridmap_handler_.set_grid_meter_ratio(grid_resolution, 0.1))
        return false;


    // Clean the containers

    m_gridinfo_vector_.clear();
    m_gridinfo_vector_.resize(map.size());

    while (m_opennodes_pq_.size()>0)
    {
        m_opennodes_pq_.pop();
    }

    m_result_path_.clear();
    

    // update values of the start node and goal node

    StructPoseReal m_start_pose_, m_goal_pose_;

    m_start_pose_.x = startpose[0];
    m_start_pose_.y = startpose[1];

    m_goal_pose_.x = goalpose[0];
    m_goal_pose_.y = goalpose[1];

    m_gridmap_handler_.convert_fine_pose_to_grid(m_start_pose_, m_start_grid_);
    m_gridmap_handler_.convert_fine_pose_to_grid(m_goal_pose_, m_goal_grid_);

    // cout << "start_grid_ " << endl;
    // cout << m_start_grid_.x << " " << m_start_grid_.y  << endl;

    // cout << "goal_grid_ " << endl;
    // cout << m_goal_grid_.x << " " << m_goal_grid_.y << endl;


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

    int _start_grid_index_1d = m_gridmap_handler_.convert_twoD_to_oneD(m_start_grid_.x, m_start_grid_.y);
    m_gridinfo_vector_[_start_grid_index_1d].fcost = 0;
    m_gridinfo_vector_[_start_grid_index_1d].gcost = 0;
    m_gridinfo_vector_[_start_grid_index_1d].grid_type = AStarGridType::Open;
    m_gridinfo_vector_[_start_grid_index_1d].parent_grid = m_start_grid_;
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
    FLAG_timeout_ = false;

    m_debug_grid_search_time_ = 0.0;

    // cout << "m_start_grid_ " << m_start_grid_.x << " " << m_start_grid_.y << endl;

    // cout << "ClassAStar::setup() DONE" << endl;

    return true;
}

bool ClassAStar::search()
{
    // cout << "ClassAStar::search()  Looking for path " << endl;

    double time1 = helper_get_time();

    StructPoseGrid grid_pose;

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
            std::cerr << "search trapped, failed, tried " << search_iteration_count << " times" << endl;
            return false; // there is no OPEN node to search
        }

        explore_one_node();

        search_iteration_count++;

        double time2 = helper_get_time();
        
        if( int((time2-time1)*1000.0) >= m_time_out_ms_ )
        {
            std::cerr << "search timeout, failed, tried " << search_iteration_count << " times" << endl;
            FLAG_timeout_ = true;
            // find a path closest to the goal.
            return true;
        }

    }

    t2 = helper_get_time();
    
    cout << "                                  " ;
    cout << "Time in us: " << int((t2 - t1)*1000000.0) << endl;

    cout << "Found the goal. Tried " << search_iteration_count << " iterations " << endl;

    return true;
}


void ClassAStar::explore_one_node()
{
    // cout << "explore_one_node()" << endl;

    int _curr_node_grid_1d = m_gridmap_handler_.convert_twoD_to_oneD(m_opennodes_pq_.top().self_grid.x, m_opennodes_pq_.top().self_grid.y);

    GridInfo *_current_grid = &(m_gridinfo_vector_[_curr_node_grid_1d]);

    if (_current_grid->grid_type != AStarGridType::Open)
    {
        if (m_opennodes_pq_.size())
        {
            m_opennodes_pq_.pop();
        }
        // cout << "Top node in pq has unsafe type:" << m_gridinfo_vector_[_curr_node_grid_1d].grid_type << endl;
        return;
    }

    // passed initial checking, so now change the type to Closed.

    _current_grid->grid_type = AStarGridType::Closed;
    m_opennodes_pq_.pop();

    // check if reach the goal.

    bool x_match_goal = std::abs(_current_grid->self_grid.x - m_goal_grid_.x) <= m_step_length;
    bool y_match_goal = std::abs(_current_grid->self_grid.y - m_goal_grid_.y) <= m_step_length;

    if (x_match_goal && y_match_goal)
    {
        int _goal_node_grid_1d = m_gridmap_handler_.convert_twoD_to_oneD(m_goal_grid_.x, m_goal_grid_.y);
        m_gridinfo_vector_[_goal_node_grid_1d].grid_type = AStarGridType::Open;
        m_gridinfo_vector_[_goal_node_grid_1d].parent_grid = _current_grid->self_grid;
        m_gridinfo_vector_[_goal_node_grid_1d].self_grid = m_goal_grid_;
        FLAG_reach_goal_ = true;
        return;
    }


    // Didnot reach goal, so continue searching.

    GridInfo neigbr_grid;
    int _neighbor_grid_index_1d;
    int _new_g_cost, _new_f_cost;

    for (auto mm : m_motion_model_)
    {
        neigbr_grid.self_grid.x = _current_grid->self_grid.x + mm.dx;
        neigbr_grid.self_grid.y = _current_grid->self_grid.y + mm.dy;

        // cout << "nb " << neigbr_grid.self_grid.x << " " << neigbr_grid.self_grid.y << " " << neigbr_grid.self_grid.yaw << " " << endl;

        // ignore this neighbour node if it is out of map or inside obstacle.
        if ( ! m_gridmap_handler_.check_grid_within_map(neigbr_grid.self_grid.x, neigbr_grid.self_grid.y))
        {
            continue;
        }
        
        if ( ! m_gridmap_handler_.check_grid_clear(neigbr_grid.self_grid.x, neigbr_grid.self_grid.y, ClassGridMapHandler::EnumMode::plan))
        {
            continue;
        }

        // cout << "valid grid." << endl;

        // cout << "m_open_grids_.begin() " << m_open_grids_.begin()->self_grid.x << " " << m_open_grids_.begin()->self_grid.y << " " << m_open_grids_.begin()->self_grid.yaw << " " << endl;
        
        _neighbor_grid_index_1d = m_gridmap_handler_.convert_twoD_to_oneD(neigbr_grid.self_grid.x, neigbr_grid.self_grid.y);

        GridInfo *_neibor_grid = &(m_gridinfo_vector_[_neighbor_grid_index_1d]);

        if (_neibor_grid->grid_type == AStarGridType::NewGrid)
        {
            // simply add this node into Open type.
            _neibor_grid->grid_type = AStarGridType::Open;
            _neibor_grid->parent_grid = _current_grid->self_grid;
            _neibor_grid->self_grid = neigbr_grid.self_grid;

            _new_g_cost = _current_grid->gcost + mm.edge_cost;
            _neibor_grid->gcost = _new_g_cost;
            _neibor_grid->fcost = _new_g_cost + compute_h_cost_Manhattan(neigbr_grid.self_grid, m_goal_grid_);

            m_opennodes_pq_.push(NodeinfoForPQ(
                _neibor_grid->fcost,
                _neibor_grid->self_grid
            ));
        }
        else if (_neibor_grid->grid_type == AStarGridType::Open)
        {
            _new_g_cost = _current_grid->gcost + mm.edge_cost;
            _new_f_cost = _new_g_cost + compute_h_cost_Manhattan(neigbr_grid.self_grid, m_goal_grid_);

            if (_new_f_cost < _neibor_grid->fcost)
            {
                _neibor_grid->grid_type = AStarGridType::Open;
                _neibor_grid->parent_grid = _current_grid->self_grid;
                _neibor_grid->self_grid = neigbr_grid.self_grid;

                _neibor_grid->gcost = _new_g_cost;
                _neibor_grid->fcost = _new_f_cost;

                m_opennodes_pq_.push(NodeinfoForPQ(
                    _neibor_grid->fcost,
                    _neibor_grid->self_grid
                ));
            }
        }
        
        // if (m_gridinfo_vector_[_neighbor_grid_index_1d].grid_type == AStarGridType::NewGrid)
        // {
        //     // simply add this node into Open type.
        //     m_gridinfo_vector_[_neighbor_grid_index_1d].grid_type = AStarGridType::Open;
        //     m_gridinfo_vector_[_neighbor_grid_index_1d].parent_grid = _current_grid->self_grid;
        //     m_gridinfo_vector_[_neighbor_grid_index_1d].self_grid = neigbr_grid.self_grid;

        //     _new_g_cost = _current_grid->gcost + mm.edge_cost;
        //     m_gridinfo_vector_[_neighbor_grid_index_1d].gcost = _new_g_cost;
        //     m_gridinfo_vector_[_neighbor_grid_index_1d].fcost = _new_g_cost + compute_h_cost_Manhattan(neigbr_grid.self_grid, m_goal_grid_);

        //     m_opennodes_pq_.push(NodeinfoForPQ(
        //         m_gridinfo_vector_[_neighbor_grid_index_1d].fcost,
        //         m_gridinfo_vector_[_neighbor_grid_index_1d].self_grid
        //     ));
        // }
        // else if (m_gridinfo_vector_[_neighbor_grid_index_1d].grid_type == AStarGridType::Open)
        // {
        //     _new_g_cost = _current_grid->gcost + mm.edge_cost;
        //     _new_f_cost = _new_g_cost + compute_h_cost_Manhattan(neigbr_grid.self_grid, m_goal_grid_);

        //     if (_new_f_cost < m_gridinfo_vector_[_neighbor_grid_index_1d].fcost)
        //     {
        //         m_gridinfo_vector_[_neighbor_grid_index_1d].grid_type = AStarGridType::Open;
        //         m_gridinfo_vector_[_neighbor_grid_index_1d].parent_grid = _current_grid->self_grid;
        //         m_gridinfo_vector_[_neighbor_grid_index_1d].self_grid = neigbr_grid.self_grid;

        //         m_gridinfo_vector_[_neighbor_grid_index_1d].gcost = _new_g_cost;
        //         m_gridinfo_vector_[_neighbor_grid_index_1d].fcost = _new_f_cost;

        //         m_opennodes_pq_.push(NodeinfoForPQ(
        //             m_gridinfo_vector_[_neighbor_grid_index_1d].fcost,
        //             m_gridinfo_vector_[_neighbor_grid_index_1d].self_grid
        //         ));
        //     }
        // }

        // cout << "count  " << count << "   m_open_grids_.size() " << m_open_grids_.size() << endl;
        // auto open_iterator = m_open_grids_.begin();
        // cout << "grid " << (*open_iterator).self_grid.x << " " << (*open_iterator).self_grid.y << " " << (*open_iterator).self_grid.yaw << " " << endl;
        // cout << "" << endl;
    }

}

void ClassAStar::get_path(std::deque<array<int, 2>> &r_path)
{
    cout << "ClassAStar::get_path()" << endl;
    r_path.clear();

    StructPoseGrid temp_step = m_goal_grid_;

    while (temp_step != m_start_grid_)
    {
        // cout << m_clos_grids_[temp_step].real_pose.x << " ";
        // cout << m_clos_grids_[temp_step].real_pose.y << endl;
        // cout << "get_path(): temp: "  << temp_step.x << " ";
        // cout << temp_step.y << endl;

        int _temp_step_index_1d = m_gridmap_handler_.convert_twoD_to_oneD(temp_step.x, temp_step.y);
        array<int, 2> _temp_step_grid = {m_gridinfo_vector_[_temp_step_index_1d].self_grid.x,
                                         m_gridinfo_vector_[_temp_step_index_1d].self_grid.y} ;
        
        r_path.push_front( _temp_step_grid );
        temp_step = m_gridinfo_vector_[_temp_step_index_1d].parent_grid;

        if(temp_step.x == 0 && temp_step.y == 0)
        {
            break;
        }
    }
    // int _temp_step_index_1d = m_gridmap_handler_.convert_3D_to_oneD(m_start_grid_.x, m_start_grid_.y, m_start_grid_.yaw);
    // r_path.push_front( m_gridinfo_vector_[_temp_step_index_1d].real_pose.to_array3() );  // this one should be the starting point.
    r_path.push_front( array<int, 2>{m_start_grid_.x, m_start_grid_.y} ); 


    cout << "ClassAStar::get_path() Done." << endl;
}

void ClassAStar::get_path_in_meter(std::deque<array<double, 2>> &r_path)
{
    cout << "ClassAStar::get_path()" << endl;
    r_path.clear();

    StructPoseGrid temp_step = m_goal_grid_;

    while (temp_step != m_start_grid_)
    {
        // cout << m_clos_grids_[temp_step].real_pose.x << " ";
        // cout << m_clos_grids_[temp_step].real_pose.y << endl;
        // cout << "get_path(): temp: "  << temp_step.x << " ";
        // cout << temp_step.y << endl;

        int _temp_step_index_1d = m_gridmap_handler_.convert_twoD_to_oneD(temp_step.x, temp_step.y);
        array<int, 2> _temp_step_grid = {m_gridinfo_vector_[_temp_step_index_1d].self_grid.x,
                                         m_gridinfo_vector_[_temp_step_index_1d].self_grid.y} ;
        
        array<double, 2> _temp_step_meter;

        m_gridmap_handler_.convert_grid_pose_to_fine(_temp_step_grid[0], _temp_step_grid[1], 
                                                     _temp_step_meter[0], _temp_step_meter[1]);
        
        r_path.push_front( _temp_step_meter );
        temp_step = m_gridinfo_vector_[_temp_step_index_1d].parent_grid;

        if(temp_step.x == 0 && temp_step.y == 0)
        {
            break;
        }
    }
    // int _temp_step_index_1d = m_gridmap_handler_.convert_3D_to_oneD(m_start_grid_.x, m_start_grid_.y, m_start_grid_.yaw);
    // r_path.push_front( m_gridinfo_vector_[_temp_step_index_1d].real_pose.to_array3() );  // this one should be the starting point.

    array<double, 2> _temp_step_meter;

    m_gridmap_handler_.convert_grid_pose_to_fine(m_start_grid_.x, m_start_grid_.y, 
                                                _temp_step_meter[0], _temp_step_meter[1]);

    r_path.push_front( _temp_step_meter ); 


    cout << "ClassAStar::get_path() Done." << endl;
}


void ClassAStar::build_motion_model()
{
    MotionPrimitive mp;

    

    mp.dx = m_step_length;
    mp.dy = 0;
    mp.edge_cost = 10 * m_step_length;
    m_motion_model_.push_back(mp);

    mp.dx = -m_step_length;
    mp.dy = 0;
    mp.edge_cost = 10 * m_step_length;
    m_motion_model_.push_back(mp);

    mp.dx = 0;
    mp.dy = m_step_length;
    mp.edge_cost = 10 * m_step_length;
    m_motion_model_.push_back(mp);

    mp.dx = 0;
    mp.dy = -m_step_length;
    mp.edge_cost = 10 * m_step_length;
    m_motion_model_.push_back(mp);

    mp.dx = m_step_length;
    mp.dy = m_step_length;
    mp.edge_cost = 14 * m_step_length;
    m_motion_model_.push_back(mp);

    mp.dx = -m_step_length;
    mp.dy = m_step_length;
    mp.edge_cost = 14 * m_step_length;
    m_motion_model_.push_back(mp);

    mp.dx = -m_step_length;
    mp.dy = -m_step_length;
    mp.edge_cost = 14 * m_step_length;
    m_motion_model_.push_back(mp);

    mp.dx = m_step_length;
    mp.dy = -m_step_length;
    mp.edge_cost = 14 * m_step_length;
    m_motion_model_.push_back(mp);
}


inline int ClassAStar::compute_h_cost_Euclidean(const StructPoseGrid n, const StructPoseGrid g)
{
    int dx = n.x - g.x;
    int dy = n.y - g.y;
    return int(sqrt(dx * dx + dy * dy)*10);
}

inline int ClassAStar::compute_h_cost_Manhattan(const StructPoseGrid n, const StructPoseGrid g)
{
    // int dx = abs(n.x - g.x);
    // int dy = abs(n.y - g.y);
    // return (dx + dy)*10;
    return (abs(n.x - g.x) + abs(n.y - g.y))*10;
}

inline int ClassAStar::compute_h_cost_Chebyshev(const StructPoseGrid n, const StructPoseGrid g)
{
    int dx = abs(n.x - g.x);
    int dy = abs(n.y - g.y);
    return std::max(dx, dy)*10;
}



/**
 * @brief 
 * return time in seconds, epoch time. 
 */
double ClassAStar::helper_get_time()
{
    return high_resolution_clock::now().time_since_epoch().count()/1000000000.0; 
}




#endif
