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
 * @file hybrid_astar_tools.h
 * @author Mingjie
 * @brief This file contains several functions and structs that are frequently used.  
 * @version 0.2
 * @date 2023-11-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef HYBRID_ASTAR_TOOLS_H
#define HYBRID_ASTAR_TOOLS_H

#include <vector>

#include "../utils/hawa_data_containers.h"


/**
 * @brief Wrapped conter.
*/
class ClassHawaCounter
{
private:
    int m_count_;
public:
    ClassHawaCounter(/* args */);
    ~ClassHawaCounter();
    void increment();
    void resetCount();
    int getVal();
};

ClassHawaCounter::ClassHawaCounter(/* args */)
{
    m_count_ = 0;
}

ClassHawaCounter::~ClassHawaCounter()
{
}

void ClassHawaCounter::increment()
{
    m_count_ ++;
}

void ClassHawaCounter::resetCount()
{
    m_count_ = 0;
}

int ClassHawaCounter::getVal()
{
    return m_count_;
}


struct TimingAndCounter
{
    double sec__rs_search;
    double sec__grid_search;
    double sec__rs_collision_check;
    double sec__astar_search;
    ClassHawaCounter count__find_min_cost_;
    ClassHawaCounter count__rs_search_;

    inline void resetVals()
    {
        sec__astar_search = 0;
        sec__grid_search = 0;
        sec__rs_search = 0;
        sec__rs_collision_check = 0;
        count__find_min_cost_.resetCount();
        count__rs_search_.resetCount();
    }
};

/**
 * @brief The threshold values that will be used for classifiing the grid types. 
*/
struct MapOccThreshold
{
    int8_t plan;
    int8_t vali;        
};

/**
 * @brief This struct handles the logic for triggering the RS curve searching, during the 
 * the hybrid astar exploration.  The value of interval would be modified to the 
 * actual desired value. Briefly, everytime when counter is an interger multiplication of 
 * the interval, then the hybrid astar should try to see if any RS curve connecting to the 
 * goal pose. 
*/
struct StructCounterForRSSearch
{
    int counter = 0;
    int interval = 1;

    inline void resetVals()
    {
        counter = 0;
    }

    inline void setInterval(const int expected_interval)
    {
        if (expected_interval <= 0)
        {
            std::cerr << "StructCounterForRSSearch::setInterval  input value too small:" << expected_interval << std::endl;
        }
        interval = expected_interval;
    }

    inline bool check_match()
    {
        return (counter % interval == 0);
    }

    inline void increment_counter()
    {
        counter ++;
    }
};

struct StructFlags
{
    bool reach_goal;
    bool trapped;
    bool inside_obstacle;
    bool found_rs_solution;
    bool timeout;

    inline void resetVals()
    {
        reach_goal = false;
        trapped = false;
        inside_obstacle = false;
        found_rs_solution = false;
        timeout = false;
    }
};


struct StructImportantPoses
{
    StructPoseGrid start_grid;
    StructPoseReal start_pose;

    StructPoseGrid goal_grid;
    StructPoseReal goal_pose;

    StructPoseGrid grid_last_incremental_step;

    StructPoseGrid min_cost_node;
};


struct StructParametersForSearching
{
    int time_out_ms = 200;

    const double dummy_min_cost_value = std::numeric_limits<double>::max();

    double yaw_angle_bin_size;

    inline void setTimeOutMs(const int val)
    {
        time_out_ms = val;
    }

    inline void setYawBinSize(const double val)
    {
        yaw_angle_bin_size = val;
    }
};


enum AStarGridType
{
    NewGrid,
    Open,
    Closed
};

/**
 * @brief The 6 types of motions used in HybridAstar searching.  F is forward, R is reverse. 
 * S is straight motion, L is steer to left side. R is steer to right side.
*/
enum EnumHAMotionType
{
    F_S, 
    F_L,
    F_R,
    R_S,
    R_L,
    R_R
};

struct GridInfo
{
    double gcost = 0;
    double fcost = 0;
    int steer_type = 0; // 0-5, total 6 types
    EnumHAMotionType motion_type;
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


enum EnumResultState
{
    searching,
    success__find_path_by_increment,
    success__find_path_by_ReedsShepp,
    failed__trapped__use_closest,
    failed__timeout__use_closest
};




double estimate_steering_cost(const int last_steer, const int new_steer)
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
    else
    {
        return 1.0;
    }
}

double estimateChangingMotionTypeCost(const EnumHAMotionType last_motion, const EnumHAMotionType this_motion)
{
    const double _no_change = 1.0;
    const double _small_steer = 1.1;
    const double _large_steer = 1.25;
    const double _change_direction = 1.5;

    if (last_motion == this_motion)
        return _no_change;

    else if ((last_motion == EnumHAMotionType::F_L) && (this_motion == EnumHAMotionType::F_S))
        return _small_steer;
    else if ((last_motion == EnumHAMotionType::F_R) && (this_motion == EnumHAMotionType::F_S))
        return _small_steer;
    else if ((last_motion == EnumHAMotionType::F_S) && (this_motion == EnumHAMotionType::F_L))
        return _small_steer;
    else if ((last_motion == EnumHAMotionType::F_S) && (this_motion == EnumHAMotionType::F_R))
        return _small_steer;
    
    else if ((last_motion == EnumHAMotionType::R_L) && (this_motion == EnumHAMotionType::R_S))
        return _small_steer;
    else if ((last_motion == EnumHAMotionType::R_R) && (this_motion == EnumHAMotionType::R_S))
        return _small_steer;
    else if ((last_motion == EnumHAMotionType::R_S) && (this_motion == EnumHAMotionType::R_L))
        return _small_steer;
    else if ((last_motion == EnumHAMotionType::R_S) && (this_motion == EnumHAMotionType::R_R))
        return _small_steer;

    else if ((last_motion == EnumHAMotionType::F_L) && (this_motion == EnumHAMotionType::F_R))
        return _large_steer;
    else if ((last_motion == EnumHAMotionType::F_R) && (this_motion == EnumHAMotionType::F_L))
        return _large_steer;
    
    else if ((last_motion == EnumHAMotionType::R_L) && (this_motion == EnumHAMotionType::R_R))
        return _large_steer;
    else if ((last_motion == EnumHAMotionType::R_R) && (this_motion == EnumHAMotionType::R_L))
        return _large_steer;
        

    else if ((last_motion == EnumHAMotionType::F_L) && (this_motion == EnumHAMotionType::R_S))
        return _change_direction;
    else if ((last_motion == EnumHAMotionType::F_L) && (this_motion == EnumHAMotionType::R_L))
        return _change_direction;
    else if ((last_motion == EnumHAMotionType::F_L) && (this_motion == EnumHAMotionType::R_R))
        return _change_direction;
    
    else if ((last_motion == EnumHAMotionType::F_S) && (this_motion == EnumHAMotionType::R_S))
        return _change_direction;
    else if ((last_motion == EnumHAMotionType::F_S) && (this_motion == EnumHAMotionType::R_L))
        return _change_direction;
    else if ((last_motion == EnumHAMotionType::F_S) && (this_motion == EnumHAMotionType::R_R))
        return _change_direction;

    else if ((last_motion == EnumHAMotionType::F_R) && (this_motion == EnumHAMotionType::R_S))
        return _change_direction;
    else if ((last_motion == EnumHAMotionType::F_R) && (this_motion == EnumHAMotionType::R_L))
        return _change_direction;
    else if ((last_motion == EnumHAMotionType::F_R) && (this_motion == EnumHAMotionType::R_R))
        return _change_direction;

    else if ((last_motion == EnumHAMotionType::R_L) && (this_motion == EnumHAMotionType::F_S))
        return _change_direction;
    else if ((last_motion == EnumHAMotionType::R_L) && (this_motion == EnumHAMotionType::F_L))
        return _change_direction;
    else if ((last_motion == EnumHAMotionType::R_L) && (this_motion == EnumHAMotionType::F_R))
        return _change_direction;
    
    else if ((last_motion == EnumHAMotionType::R_S) && (this_motion == EnumHAMotionType::F_S))
        return _change_direction;
    else if ((last_motion == EnumHAMotionType::R_S) && (this_motion == EnumHAMotionType::F_L))
        return _change_direction;
    else if ((last_motion == EnumHAMotionType::R_S) && (this_motion == EnumHAMotionType::F_R))
        return _change_direction;

    else if ((last_motion == EnumHAMotionType::R_R) && (this_motion == EnumHAMotionType::F_S))
        return _change_direction;
    else if ((last_motion == EnumHAMotionType::R_R) && (this_motion == EnumHAMotionType::F_L))
        return _change_direction;
    else if ((last_motion == EnumHAMotionType::R_R) && (this_motion == EnumHAMotionType::F_R))
        return _change_direction;

    std::cerr << "estimate_changing_motion_type_cost(): Unknown condition:"<< last_motion << " " << this_motion;

    return 1.0;
}

inline double computeHCostEuclidean(const StructPoseReal n, const StructPoseReal g)
{
    double dx = n.x - g.x;
    double dy = n.y - g.y;
    return std::sqrt(dx * dx + dy * dy);
}

inline double computeHCostManhattan(const StructPoseReal n, const StructPoseReal g)
{
    double dx = std::abs(n.x - g.x);
    double dy = std::abs(n.y - g.y);
    return dx + dy;
}

inline double computeHCostChebyshev(const StructPoseReal n, const StructPoseReal g)
{
    double dx = std::abs(n.x - g.x);
    double dy = std::abs(n.y - g.y);
    return std::max(dx, dy);
}





#endif