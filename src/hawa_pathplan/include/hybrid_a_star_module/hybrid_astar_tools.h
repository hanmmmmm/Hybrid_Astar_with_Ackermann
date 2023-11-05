#ifndef HYBRID_ASTAR_TOOLS_H
#define HYBRID_ASTAR_TOOLS_H

#include <vector>

#include "../utils/hawa_data_containers.h"

struct TimingAndCounter
{
    double sec__rs_search;
    double sec__grid_search;
    double sec__rs_collision_check;
    double sec__astar_search;
    int times__find_min_cost_;
    int times__rs_search_;

    inline void resetVals()
    {
        sec__astar_search = 0;
        sec__grid_search = 0;
        sec__rs_search = 0;
        sec__rs_collision_check = 0;
        times__find_min_cost_ = 0;
        times__rs_search_ = 0;        
    }
};

struct MapOccThreshold
{
    int8_t plan;
    int8_t vali;        
};


struct StructCounterForRSSearch
{
    int counter = 0;
    int interval = 1;

    inline void resetVals()
    {
        counter = 0;
    }

    inline bool check_match()
    {
        return (counter % interval == 0);
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



struct ParametersForSearching
{
    
};


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