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
 * @file class_hybrid_astar.h
 * @author Mingjie
 * @brief This file is the implementation of the Hybrid Astar searching algorithm on
 * an occupancy grid map. This implementation contains only the incremental
 * searching (the part that's like original a-star) and the ReedsShepp curve
 * searching.  This does not have the parts like Voronoi field and numerical
 * optimization.
 * @version 0.2
 * @date 2023-11-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef HAWA_HYBRID_ASTAR_H
#define HAWA_HYBRID_ASTAR_H

#include "common_includes.h"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "std_msgs/msg/bool.hpp"

#include "class_reedsshepp_solver.h"

#include "class_gridmap_handler.h"
#include "class_custom_path_container.h"

#include "custom_data_types.h"
#include "utils/class_utils__converters.h"
#include "utils/class_utils__timer.h"

#include "hybrid_astar_tools.h"
#include "struct_motion_primitives.h"


namespace hawa
{

/**
 * @brief This class is the implementation of the Hybrid Astar searching algorithm on
 * an occupancy grid map. This implementation contains the incremental
 * searching (the part that's like original a-star) and the ReedsShepp curve
 * searching.  This does not have the parts like Voronoi field and numerical
 * optimization.
 */
class ClassHybridAStar
{
private:
    
    // rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_publisher_searching_ ;
    // std::string m_topic_name_searching_published_;

    // ClassGridMapHandler m_gridmap_handler_;

    ClassReedsSheppSolver m_RS_curve_finder_;

    StructMotionModel m_motion_model_;

    std::vector<GridInfo> m_gridinfo_vector_;

    std::priority_queue<NodeinfoForPQ, std::vector<NodeinfoForPQ>, CostCompareMethod> m_opennodes_pq_;

    StructImportantPoses m_important_poses_;

    std::deque<StructPoseReal> m_result_path_; // container for storing the final complete path.
    std::vector<StructPoseReal> m_path_RS_section_; // container for storing the path segments found by ReedsShepp.

    StructCounterForRSSearch m_rs_search_trigger_;

    StructFlags m_flags_;

    TimingAndCounter m_timer_and_counter_;

    MapOccThreshold m_map_occ_threshold_;

    ParametersForSearching m_parameters_;

private:
    bool checkStartAndGoalAccessible();

    void setupTheFirstGrid();

    void prepareForExplore(bool *doable, GridInfo *grid_to_explore);

    void exploreOneNode();

    bool tryFindReedsSheppCurve(const StructPoseGrid &r_curr_grid);

    bool checkRSSearchResult();

public:
    ClassHybridAStar();
    ~ClassHybridAStar();
    
    std::shared_ptr<ClassGridMapHandler> m_gridmap_handler_ptr_;

    bool loadParameters();

    bool setMap(nav_msgs::msg::OccupancyGrid *ptr_map_data);

    bool setStartGoalPoses(StructPoseReal startpose, StructPoseReal goalpose);

    bool setup();

    bool search();

    void getFinalHybridAstarPath(ClassCustomPathContainer &r_path);
};

} // namespace hawa




#endif
