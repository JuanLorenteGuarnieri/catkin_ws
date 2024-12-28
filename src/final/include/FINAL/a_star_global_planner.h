/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
// #include <angles/angles.h>
// #include <base_local_planner/world_model.h>
// #include <base_local_planner/costmap_model.h>

#include <time.h> 
#include <unordered_map>
#include <unordered_set>
#include "FINAL/TreeNode.h"

#ifndef A_STAR_PLANNER_CPP
#define A_STAR_PLANNER_CPP

#include <functional> // Para std::hash

struct hash_vector {
    std::size_t operator()(const std::vector<int>& vec) const {
        std::size_t seed = 0;
        for (int v : vec) {
            seed ^= std::hash<int>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};


namespace a_star_planner {

class AStarPlanner : public nav_core::BaseGlobalPlanner {
    
public:

    AStarPlanner();
    AStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    // overridden classes from interface nav_core::BaseGlobalPlanner
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan);
    bool computeAStar(const std::vector<int>& start, const std::vector<int>& goal, std::vector<std::vector<int>>& sol);
    void getPlan(const std::vector<std::vector<int>> sol, std::vector<geometry_msgs::PoseStamped>& plan);
    

private:
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    visualization_msgs::Marker line_strip_marker_;

    costmap_2d::Costmap2DROS* costmap_ros_;
	costmap_2d::Costmap2D* costmap_;
    std::string global_frame_id_;
	bool initialized_;

    double max_samples_;
    double max_dist_;
    double resolution_;


    // functions to compute the plan
    bool obstacleFree(const unsigned int x0, const unsigned int y0, const unsigned int x1, const unsigned int y1);
    std::vector<std::vector<int>> reconstructPath( std::unordered_map<std::vector<int>, std::vector<int>, hash_vector>& cameFrom, std::vector<int>& current,
    std::vector<std::vector<int>>& sol);
    double heuristic(const std::vector<int>& a, const std::vector<int>& b);
    std::vector<std::vector<int>> getNeighbors(const std::vector<int>& node);
};

};
 #endif
