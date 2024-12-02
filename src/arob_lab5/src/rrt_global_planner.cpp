#include <pluginlib/class_list_macros.h>
#include "AROB_lab5/rrt_global_planner.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlanner, nav_core::BaseGlobalPlanner)

//Default Constructor
namespace rrt_planner {

double distance(const unsigned int x0, const unsigned int y0, const unsigned int x1, const unsigned int y1){
    return std::sqrt((int)(x1-x0)*(int)(x1-x0) + (int)(y1-y0)*(int)(y1-y0));
}

RRTPlanner::RRTPlanner() : costmap_ros_(NULL), initialized_(false),
                            max_samples_(0.0){}

RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, costmap_ros);
}

void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

    if (!initialized_){
        // Initialize the publisher for the markers
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/rrt_marker", 1);

        ros::NodeHandle nh("~/" + name);
        ros::NodeHandle nh_local("~/local_costmap/");
        ros::NodeHandle nh_global("~/global_costmap/");

        nh.param("maxsamples", max_samples_, 0.0);

        //to make sure one of the nodes in the plan lies in the local costmap
        double width, height;
        nh_local.param("width", width, 3.0);
        nh_local.param("height", height, 3.0);
        max_dist_ = (std::min(width, height)/3.0);  //or any other distance within local costmap

        nh_global.param("resolution", resolution_, 0.05);

        // std::cout << "Parameters: " << max_samples_ << ", " << dist_th_ << ", " << visualize_markers_ << ", " << max_dist_ << std::endl;
        // std::cout << "Local costmap size: " << width << ", " << height << std::endl;
        // std::cout << "Global costmap resolution: " << resolution_ << std::endl;

        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros->getCostmap();
        global_frame_id_ = costmap_ros_->getGlobalFrameID();

        initialized_ = true;
    }
	else{
	    ROS_WARN("This planner has already been initialized... doing nothing.");
    }
}

bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, 
                            std::vector<geometry_msgs::PoseStamped>& plan ){

    // std::cout << "RRTPlanner::makePlan" << std::endl;
    
    if (!initialized_){
        ROS_ERROR("The planner has not been initialized.");
        return false;
    }

	if (start.header.frame_id != costmap_ros_->getGlobalFrameID()){
		ROS_ERROR("The start pose must be in the %s frame, but it is in the %s frame.",
				  global_frame_id_.c_str(), start.header.frame_id.c_str());
		return false;
	}

	if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
		ROS_ERROR("The goal pose must be in the %s frame, but it is in the %s frame.",
				  global_frame_id_.c_str(), goal.header.frame_id.c_str());
		return false;
	}
    
    plan.clear();
    costmap_ = costmap_ros_->getCostmap();  // Update information from costmap
    
    // Get start and goal poses in map coordinates
    unsigned int goal_mx, goal_my, start_mx, start_my;
    if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my)){
        ROS_WARN("Goal position is out of map bounds.");
        return false;
    }    
    costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my);

    std::vector<int> point_start{(int)start_mx,(int)start_my};
    std::vector<int> point_goal{(int)goal_mx,(int)goal_my};    
  	std::vector<std::vector<int>> solRRT;
    bool computed = computeRRT(point_start, point_goal, solRRT);
    if (computed){        
        getPlan(solRRT, plan);
        // add goal
        plan.push_back(goal);
    }else{
        ROS_WARN("No plan computed");
    }

    return computed;
}

bool RRTPlanner::computeRRT(const std::vector<int> start, const std::vector<int> goal, 
                            std::vector<std::vector<int>>& sol) {
    bool finished = false;

    // Clear previous markers in RViz
    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_pub_.publish(delete_marker);
    ros::Duration(0.1).sleep();

    visualization_msgs::Marker edge_marker;
    edge_marker.header.frame_id = "map";
    edge_marker.header.stamp = ros::Time::now();
    edge_marker.ns = "rrt_tree";
    edge_marker.type = visualization_msgs::Marker::LINE_LIST;
    edge_marker.scale.x = 0.05; // line size
    edge_marker.color.r = 0.0;
    edge_marker.color.g = 0.0;
    edge_marker.color.b = 1.0;
    edge_marker.color.a = 1.0;
    edge_marker.points.clear();

    // Initialize random number generator
    srand(time(NULL));

    // Initialize the tree with the starting point in map coordinates
    TreeNode* root = new TreeNode(start);
    std::vector<TreeNode*> tree{root};

    unsigned int goal_mx = goal[0], goal_my = goal[1];

    // Main RRT loop
    int marker_id = 0; // Unique ID for markers
    for (int i = 0; i < max_samples_; i++) {
        
        // Sample a random point in free space
        std::vector<int> rand_point = {
            static_cast<int>(rand() % costmap_->getSizeInCellsX()),
            static_cast<int>(rand() % costmap_->getSizeInCellsY())
        };

        // Skip if the sampled point is an obstacle
        if (costmap_->getCost(rand_point[0], rand_point[1]) == costmap_2d::LETHAL_OBSTACLE) {
            continue;
        }

        // Find the nearest node in the tree
        TreeNode* nearest_node = nullptr;
        double min_dist = std::numeric_limits<double>::max();

        for (auto node : tree) {
            double dist = distance(node->getNode()[0], node->getNode()[1], rand_point[0], rand_point[1]);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_node = node;
            }
        }

        // Steer: Generate a new node in the direction of the random point
        double theta = atan2(rand_point[1] - nearest_node->getNode()[1],
                             rand_point[0] - nearest_node->getNode()[0]);
        std::vector<int> new_point = {
            static_cast<int>(nearest_node->getNode()[0] + max_dist_ * cos(theta)),
            static_cast<int>(nearest_node->getNode()[1] + max_dist_ * sin(theta))
        };

        // Ensure the new point is within bounds
        if (new_point[0] < 0 || new_point[0] >= costmap_->getSizeInCellsX() ||
            new_point[1] < 0 || new_point[1] >= costmap_->getSizeInCellsY()) {
            continue;
        }

        // Check if the edge is obstacle-free
        if (obstacleFree(nearest_node->getNode()[0], nearest_node->getNode()[1], new_point[0], new_point[1])) {
            // Add the new point to the tree
            TreeNode* new_node = new TreeNode(new_point);
            new_node->setParent(nearest_node);
            tree.push_back(new_node);

            edge_marker.id = marker_id++;

            // Add the edge to the marker for visualization
            geometry_msgs::Point nearest_world_point, new_world_point;
            costmap_->mapToWorld(nearest_node->getNode()[0], nearest_node->getNode()[1], nearest_world_point.x, nearest_world_point.y);
            nearest_world_point.z = 0.0;

            costmap_->mapToWorld(new_point[0], new_point[1], new_world_point.x, new_world_point.y);
            new_world_point.z = 0.0;

            edge_marker.points.push_back(nearest_world_point);
            edge_marker.points.push_back(new_world_point);

            // Publish marker every N iterations
            if (i % 20 == 0) {
                edge_marker.header.stamp = ros::Time::now();
                marker_pub_.publish(edge_marker);
            }

            // Check if the goal is reachable from the new point
            if (distance(new_point[0], new_point[1], goal_mx, goal_my) <= max_dist_ / resolution_) {
                // Goal is reached, construct the solution path
                TreeNode* current = new_node;
                while (current != nullptr) {
                    sol.push_back(current->getNode());
                    current = current->getParent();
                }
                sol.push_back(goal); // Add goal as the last point
                finished = true;
                break;
            }
        }
    }


    // Clean up tree nodes
    for (auto node : tree) {
        delete node;
    }

    return finished;
}

bool RRTPlanner::obstacleFree(const unsigned int x0, const unsigned int y0, 
                            const unsigned int x1, const unsigned int y1){
    //Bresenham algorithm to check if the line between points (x0,y0) - (x1,y1) is free of collision

    int dx = x1 - x0;
    int dy = y1 - y0;

    int incr_x = (dx > 0) ? 1.0 : -1.0;
    int incr_y = (dy > 0) ? 1.0 : -1.0;

    unsigned int da, db, incr_x_2, incr_y_2;
    if (abs(dx) >= abs(dy)){
        da = abs(dx); db = abs(dy);
        incr_x_2 = incr_x; incr_y_2 = 0;
    }else{
        da = abs(dy); db = abs(dx);
        incr_x_2 = 0; incr_y_2 = incr_y;
    }

    int p = 2*db - da;
    unsigned int a = x0; 
    unsigned int b = y0;
    unsigned int end = da;
    for (unsigned int i=0; i<end; i++){
        if (costmap_->getCost(a, b) != costmap_2d::FREE_SPACE){  // to include cells with inflated cost
            return false;
        }else{
            if (p >= 0){
                a += incr_x;
                b += incr_y;
                p -= 2*da;
            }else{
                a += incr_x_2;
                b += incr_y_2;
            }
            p += 2*db;
        }
    }

    return true;
}

void RRTPlanner::getPlan(const std::vector<std::vector<int>> sol, std::vector<geometry_msgs::PoseStamped>& plan){

    for (auto it = sol.rbegin(); it != sol.rend(); it++){
        std::vector<int> point = (*it);
        geometry_msgs::PoseStamped pose;

        costmap_->mapToWorld((unsigned int)point[0], (unsigned int)point[1], 
                            pose.pose.position.x, pose.pose.position.y);
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = global_frame_id_;
        pose.pose.orientation.w = 1;
        plan.push_back(pose);

    }
}

};
