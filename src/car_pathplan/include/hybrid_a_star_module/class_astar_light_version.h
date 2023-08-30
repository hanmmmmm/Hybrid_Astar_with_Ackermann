
#ifndef ASTAR_H
#define ASTAR_H

#include <iostream>
#include <queue>
#include <unordered_map>
#include <map>
#include <math.h>

#include <algorithm>
// #include <set>
#include <chrono>

#include "../car_pathplan/struct_simple_pose.h"

// #include "../car_pathplan/class_gridmap_handler.h"
#include "../car_pathplan/class_gridmap_2d_handler.h"

using std::cout;
using std::endl;

using std::string;

using std::vector;
using std::array;
using std::deque;
using std::queue;
using std::priority_queue;
using std::unordered_map;

using std::chrono::high_resolution_clock;

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


private:
    ClassGridMap2DHandler m_gridmap_handler_;

    vector<MotionPrimitive> m_motion_model_;

    // vector<GridInfo> m_gridinfo_vector_;

    // priority_queue<NodeinfoForPQ, vector<NodeinfoForPQ>, CostCompareMethod> m_opennodes_pq_;

    StructPoseGrid m_start_grid_;

    StructPoseGrid m_goal_grid_;


private:

    double m_time_out_ms_;

    double m_dummy_min_cost_value_;

    bool FLAG_reach_goal_;
    bool FLAG_trapped_;
    bool FLAG_inside_obstacle_;
    bool FLAG_timeout_;

    double m_debug_grid_search_time_;
    int8_t m_plan_threshold_;

    int m_step_length;

    double m_result_path_length_meter_;

    double m_grid_resolution_;

private:

    void build_motion_model();

    // void explore_one_node(priority_queue<NodeinfoForPQ, vector<NodeinfoForPQ>, CostCompareMethod> &pq_ptr, vector<GridInfo> &vector_ptr); 
    void explore_one_node(priority_queue<NodeinfoForPQ, vector<NodeinfoForPQ>, CostCompareMethod> &pq_ptr, unordered_map<int, GridInfo> &vector_ptr); 

    double helper_get_time();

public:
    ClassAStar();
    ~ClassAStar();

    bool load_parameters();

    bool setup_map(const int grid_map_width,
               const int grid_map_height,
               vector<int8_t> &r_gridmap,
               const double grid_resolution,
               const int8_t plan_threshold);
    
    bool setup_points(const array<double, 2> startpose, 
                      const array<double, 2> goalpose);
    
    void set_step_size(int val);

    bool search();

    void get_path(std::deque<array<int, 2>> &r_path);

    double get_path_length_meter();
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

    m_step_length = 4;

    return true;
}

void ClassAStar::set_step_size(int val)
{
    m_step_length = val;
}

bool ClassAStar::setup_map(
    const int map_width_grid,
    const int map_height_grid,
    vector<int8_t> &map,
    const double grid_resolution,
    const int8_t plan_threshold)
{

    // cout << "ClassAStar::setup() START" << endl;

    m_time_out_ms_ = 2;

    // update the gridmap using the latest message.

    if (!m_gridmap_handler_.set_grid_map_ptr(&map))
        return false;
    if (!m_gridmap_handler_.set_grid_width_height(map_width_grid, map_height_grid))
        return false;
    if (!m_gridmap_handler_.set_planning_obstacle_threshold(plan_threshold))
        return false;
    if (!m_gridmap_handler_.set_grid_meter_ratio(grid_resolution))
        return false;

    m_grid_resolution_ = grid_resolution;


    

    // cout << "ClassAStar::setup() DONE" << endl;

    return true;
}

bool ClassAStar::setup_points(const array<double, 2> startpose, const array<double, 2> goalpose)
{
    // update values of the start node and goal node

    // StructPoseReal m_start_pose_, m_goal_pose_;

    // m_start_pose_.x = startpose[0];
    // m_start_pose_.y = startpose[1];

    // m_goal_pose_.x = goalpose[0];
    // m_goal_pose_.y = goalpose[1];

    // m_gridmap_handler_.convert_fine_pose_to_grid(m_start_pose_, m_start_grid_);
    m_gridmap_handler_.convert_fine_pose_to_grid(startpose[0], startpose[1], m_start_grid_.x, m_start_grid_.y);
    // m_gridmap_handler_.convert_fine_pose_to_grid(m_goal_pose_, m_goal_grid_);
    m_gridmap_handler_.convert_fine_pose_to_grid(goalpose[0], goalpose[1], m_goal_grid_.x, m_goal_grid_.y);

    // cout << "ClassAStar  start_grid_ " << endl;
    // cout << m_start_grid_.x << " " << m_start_grid_.y  << endl;

    // cout << "ClassAStar  goal_grid_ " << endl;
    // cout << m_goal_grid_.x << " " << m_goal_grid_.y << endl;


    // check if the start node or goal node are reachable. If not, then do not proceed.

    if( ! m_gridmap_handler_.check_grid_clear(m_start_grid_.x, m_start_grid_.y))
    {
        cout << "!!! Start grid is on obstacle." << endl;
        return false;
    }

    if( ! m_gridmap_handler_.check_grid_clear(m_goal_grid_.x, m_goal_grid_.y))
    {
        cout << "!!! Goal grid is on obstacle." << endl;
        return false;
    }


    // Clean the containers

    // m_gridinfo_vector_.clear();
    // m_gridinfo_vector_.resize(m_gridmap_handler_.get_map_size());

    // while (m_opennodes_pq_.size()>0)
    // {
    //     m_opennodes_pq_.pop();
    // }
    

    // // if the checking passed, then update the start node data, and insert it into open_node_priority_queue.

    // int _start_grid_index_1d = m_gridmap_handler_.convert_twoD_to_oneD(m_start_grid_.x, m_start_grid_.y);
    // m_gridinfo_vector_[_start_grid_index_1d].fcost = 0;
    // m_gridinfo_vector_[_start_grid_index_1d].gcost = 0;
    // m_gridinfo_vector_[_start_grid_index_1d].grid_type = AStarGridType::Open;
    // m_gridinfo_vector_[_start_grid_index_1d].parent_grid = m_start_grid_;
    // m_gridinfo_vector_[_start_grid_index_1d].self_grid = m_start_grid_;

    // NodeinfoForPQ _start_grid_nodeinfo(
    //     m_gridinfo_vector_[_start_grid_index_1d].fcost,
    //     m_gridinfo_vector_[_start_grid_index_1d].self_grid
    // );

    // m_opennodes_pq_.push(_start_grid_nodeinfo);

    // cout << "PQ size: " << m_opennodes_pq_.size() << endl;

    // reset the flags, counters, timers.

    FLAG_reach_goal_ = false;
    FLAG_trapped_ = false;
    FLAG_timeout_ = false;

    m_debug_grid_search_time_ = 0.0;

    // cout << "m_start_grid_ " << m_start_grid_.x << " " << m_start_grid_.y << endl;

}

bool ClassAStar::search()
{
    // cout << "ClassAStar::search()  Looking for path " << endl;

    double time1 = helper_get_time();

    double t1, t2, t3, t4, t5;

    t1 = helper_get_time();

    // vector<GridInfo> _gridinfo_vector_;
    // _gridinfo_vector_.resize(m_gridmap_handler_.get_map_size());

    // const size_t _mapsize = m_gridmap_handler_.get_map_size();

    // array<GridInfo, 40000> _gridinfo_vector_;

    unordered_map<int, GridInfo> _gridinfo_vector_;

    t2 = helper_get_time();

    priority_queue<NodeinfoForPQ, vector<NodeinfoForPQ>, CostCompareMethod> _opennodes_pq_;

    t3 = helper_get_time();


    int _start_grid_index_1d = m_gridmap_handler_.convert_twoD_to_oneD(m_start_grid_.x, m_start_grid_.y);
    _gridinfo_vector_[_start_grid_index_1d].fcost = 0;
    _gridinfo_vector_[_start_grid_index_1d].gcost = 0;
    _gridinfo_vector_[_start_grid_index_1d].grid_type = AStarGridType::Open;
    _gridinfo_vector_[_start_grid_index_1d].parent_grid = m_start_grid_;
    _gridinfo_vector_[_start_grid_index_1d].self_grid = m_start_grid_;

    _opennodes_pq_.push(
        NodeinfoForPQ(
            _gridinfo_vector_[_start_grid_index_1d].fcost, 
            _gridinfo_vector_[_start_grid_index_1d].self_grid));


    t4 = helper_get_time();


    int search_iteration_count = 0;
    while (!FLAG_reach_goal_)
    {
        // cout << "search_iteration_count " << search_iteration_count << endl;

        // check if there's any valid nodes to search.
        if (_opennodes_pq_.size() == 0)
        {
            FLAG_trapped_ = true;
            std::cerr << "ClassAStar  search trapped, failed, tried " << search_iteration_count << " times" << endl;
            return false; // there is no OPEN node to search
        }

        if( (helper_get_time()-time1)*1000.0 >= m_time_out_ms_ )
        {
            std::cerr << "ClassAStar  search timeout, failed, tried " << search_iteration_count << " times" << endl;
            FLAG_timeout_ = true;
            // find a path closest to the goal.
            return false;
        }

        explore_one_node(_opennodes_pq_, _gridinfo_vector_);

        search_iteration_count++;
    }

    t5 = helper_get_time();
    
    // cout << "                                  " ;
    // cout << "ClassAStar  Time in us: " << int((t2 - t1)*1000000.0) << "  " << int((t3 - t2)*1000000.0)  << "  "  << int((t4 - t3)*1000000.0) << "  " << int((t5 - t4)*1000000.0) << "  "<< endl;

    // cout << "Found the goal. Tried " << search_iteration_count << " iterations " << endl;

    return true;
}


void ClassAStar::explore_one_node(priority_queue<NodeinfoForPQ, vector<NodeinfoForPQ>, CostCompareMethod> &pq_ptr, unordered_map<int, GridInfo> &vector_ptr)
{
    // cout << "explore_one_node()" << endl;

    int _curr_node_grid_1d = m_gridmap_handler_.convert_twoD_to_oneD(pq_ptr.top().self_grid.x, pq_ptr.top().self_grid.y);

    GridInfo *_current_grid = &(vector_ptr[_curr_node_grid_1d]);

    if (_current_grid->grid_type != AStarGridType::Open)
    {
        // cout << "case 1 " << endl;
        if (pq_ptr.size())
        {
            pq_ptr.pop();
        }
        // cout << "Top node in pq has unsafe type:" << vector_ptr[_curr_node_grid_1d].grid_type << endl;
        return;
    }

    // passed initial checking, so now change the type to Closed.

    _current_grid->grid_type = AStarGridType::Closed;
    pq_ptr.pop();

    // cout << "case 2 " << endl;

    // check if reach the goal.

    if ( (std::abs(_current_grid->self_grid.x - m_goal_grid_.x) <= m_step_length) && 
         (std::abs(_current_grid->self_grid.y - m_goal_grid_.y) <= m_step_length) )
    {
        int _goal_node_grid_1d = m_gridmap_handler_.convert_twoD_to_oneD(m_goal_grid_.x, m_goal_grid_.y);
        // vector_ptr[_goal_node_grid_1d].grid_type = AStarGridType::Open;
        vector_ptr[_goal_node_grid_1d].parent_grid = _current_grid->self_grid;
        vector_ptr[_goal_node_grid_1d].self_grid = m_goal_grid_;
        FLAG_reach_goal_ = true;
        m_result_path_length_meter_ = _current_grid->fcost * m_grid_resolution_ / 10.0;
        return;
    }

    // cout << "case 3 " << endl;


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
            // cout << "case 4 " << endl;
            continue;
        }
        
        if ( ! m_gridmap_handler_.check_grid_clear(neigbr_grid.self_grid.x, neigbr_grid.self_grid.y))
        {
            // cout << "case 5 " << endl;
            continue;
        }

        // cout << "case 6 " << endl;

        // cout << "valid grid." << endl;

        // cout << "m_open_grids_.begin() " << m_open_grids_.begin()->self_grid.x << " " << m_open_grids_.begin()->self_grid.y << " " << m_open_grids_.begin()->self_grid.yaw << " " << endl;
        
        _neighbor_grid_index_1d = m_gridmap_handler_.convert_twoD_to_oneD(neigbr_grid.self_grid.x, neigbr_grid.self_grid.y);

        GridInfo *_neibor_grid = &(vector_ptr[_neighbor_grid_index_1d]);

        if (_neibor_grid->grid_type == AStarGridType::NewGrid)
        {
            // cout << "case 7 " << endl;
            // simply add this node into Open type.
            _neibor_grid->grid_type = AStarGridType::Open;
            _neibor_grid->parent_grid = _current_grid->self_grid;
            _neibor_grid->self_grid = neigbr_grid.self_grid;

            _new_g_cost = _current_grid->gcost + mm.edge_cost;
            _neibor_grid->gcost = _new_g_cost;
            
            _neibor_grid->fcost = _new_g_cost + (abs(neigbr_grid.self_grid.x - m_goal_grid_.x) + abs(neigbr_grid.self_grid.y - m_goal_grid_.y))*10;

            pq_ptr.push(NodeinfoForPQ(
                _neibor_grid->fcost,
                _neibor_grid->self_grid
            ));
        }
        else if (_neibor_grid->grid_type == AStarGridType::Open)
        {
            // cout << "case 8 " << endl;
            _new_g_cost = _current_grid->gcost + mm.edge_cost;
            _new_f_cost = _new_g_cost + (abs(neigbr_grid.self_grid.x - m_goal_grid_.x) + abs(neigbr_grid.self_grid.y - m_goal_grid_.y))*10;

            if (_new_f_cost < _neibor_grid->fcost)
            {
                _neibor_grid->grid_type = AStarGridType::Open;
                _neibor_grid->parent_grid = _current_grid->self_grid;
                _neibor_grid->self_grid = neigbr_grid.self_grid;

                _neibor_grid->gcost = _new_g_cost;
                _neibor_grid->fcost = _new_f_cost;

                pq_ptr.push(NodeinfoForPQ(
                    _neibor_grid->fcost,
                    _neibor_grid->self_grid
                ));
            }
        }
        
    }

}


// void ClassAStar::get_path(std::deque<array<int, 2>> &r_path)
// {
//     cout << "ClassAStar::get_path()" << endl;
//     r_path.clear();

//     StructPoseGrid temp_step = m_goal_grid_;

//     while (temp_step != m_start_grid_)
//     {
//         // cout << m_clos_grids_[temp_step].real_pose.x << " ";
//         // cout << m_clos_grids_[temp_step].real_pose.y << endl;
//         // cout << "get_path(): temp: "  << temp_step.x << " ";
//         // cout << temp_step.y << endl;

//         int _temp_step_index_1d = m_gridmap_handler_.convert_twoD_to_oneD(temp_step.x, temp_step.y);
//         array<int, 2> _temp_step_grid = {m_gridinfo_vector_[_temp_step_index_1d].self_grid.x,
//                                          m_gridinfo_vector_[_temp_step_index_1d].self_grid.y} ;
        
//         r_path.push_front( _temp_step_grid );
//         temp_step = m_gridinfo_vector_[_temp_step_index_1d].parent_grid;

//         if(temp_step.x == 0 && temp_step.y == 0)
//         {
//             break;
//         }
//     }
//     // int _temp_step_index_1d = m_gridmap_handler_.convert_3D_to_oneD(m_start_grid_.x, m_start_grid_.y, m_start_grid_.yaw);
//     // r_path.push_front( m_gridinfo_vector_[_temp_step_index_1d].real_pose.to_array3() );  // this one should be the starting point.
//     r_path.push_front( array<int, 2>{m_start_grid_.x, m_start_grid_.y} ); 


//     cout << "ClassAStar::get_path() Done." << endl;
// }


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

double ClassAStar::get_path_length_meter()
{
    return m_result_path_length_meter_;
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
