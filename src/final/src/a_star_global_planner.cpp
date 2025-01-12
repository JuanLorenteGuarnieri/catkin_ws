#include <pluginlib/class_list_macros.h>
#include "FINAL/a_star_global_planner.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(a_star_planner::AStarPlanner, nav_core::BaseGlobalPlanner)

//Default Constructor
namespace a_star_planner {

double distance(const unsigned int x0, const unsigned int y0, const unsigned int x1, const unsigned int y1){
    return std::sqrt((int)(x1-x0)*(int)(x1-x0) + (int)(y1-y0)*(int)(y1-y0));
}

AStarPlanner::AStarPlanner() : costmap_ros_(NULL), initialized_(false),
                            max_samples_(0.0){}

AStarPlanner::AStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, costmap_ros);
}

void AStarPlanner::publishExploredNode(const std::vector<int>& node) {
    static visualization_msgs::Marker explored_nodes_marker;
    static int counter = 0;
    
    if (counter == 10000) {
        counter = 0;
        explored_nodes_marker.points.clear();
    }
    else{
        counter++;
    }

    if (explored_nodes_marker.points.empty()) {
        explored_nodes_marker.header.frame_id = global_frame_id_;
        explored_nodes_marker.header.stamp = ros::Time::now();
        explored_nodes_marker.ns = "explored_nodes";
        explored_nodes_marker.action = visualization_msgs::Marker::ADD;
        explored_nodes_marker.pose.orientation.w = 1.0;
        explored_nodes_marker.type = visualization_msgs::Marker::POINTS;
        explored_nodes_marker.scale.x = resolution_;
        explored_nodes_marker.scale.y = resolution_;
        explored_nodes_marker.color.r = 0.0f;
        explored_nodes_marker.color.g = 0.0f;
        explored_nodes_marker.color.b = 1.0f;  // Blue
        explored_nodes_marker.color.a = 0.5f;  // Transparent
    }

    geometry_msgs::Point p;
    double wx, wy;
    costmap_->mapToWorld(node[0], node[1], wx, wy);

    p.x = wx;
    p.y = wy;
    p.z = 0.0; 

    explored_nodes_marker.points.push_back(p);

    marker_pub_.publish(explored_nodes_marker);
}


void AStarPlanner::publishOptimalPath(const std::vector<std::vector<int>>& path) {
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = global_frame_id_;
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = "optimal_path";
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.pose.orientation.w = 1.0;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.scale.x =  0.1;  // Line thickness
    path_marker.color.r = 0.1f;  // Red
    path_marker.color.g = 0.0f;
    path_marker.color.b = 0.0f;
    path_marker.color.a = 1.0f;

    for (const auto& point : path) {
        geometry_msgs::Point p;
        double wx, wy;
        costmap_->mapToWorld(point[0], point[1], wx, wy);

        p.x = wx;
        p.y = wy;
        p.z = 0.0;  // Mantener Z en 0 para visualización 2D
        path_marker.points.push_back(p);
    }

    marker_pub_.publish(path_marker);
}


void AStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

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

bool AStarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, 
                            std::vector<geometry_msgs::PoseStamped>& plan ){

    // std::cout << "AStarPlanner::makePlan" << std::endl;
    
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

    std::vector<std::vector<int>> solAStar;
    bool computed = computeAStar(point_start, point_goal, solAStar);
    
    if (computed) {
        getPlan(solAStar, plan);
        plan.push_back(goal);  // Agregar el objetivo al final
    } else {
        ROS_WARN("No plan computed.");
    }

    return computed;
}

double AStarPlanner::heuristic(const std::vector<int>& a, const std::vector<int>& b) {
    if (heuristic_type_ == HeuristicType::EUCLIDEAN) {
        return distance(a[0], a[1], b[0], b[1]);
    } else if (heuristic_type_ == HeuristicType::MANHATTAN) {
        return std::abs(a[0] - b[0]) + std::abs(a[1] - b[1]);
    } else {
        ROS_WARN("Invalid heuristic type. Using Manhattan distance.");
        return std::abs(a[0] - b[0]) + std::abs(a[1] - b[1]);
    }
}

std::vector<std::vector<int>> AStarPlanner::getNeighbors(const std::vector<int>& node) {
    std::vector<std::vector<int>> neighbors;
    bool hayDiagonales = false;

    std::vector<std::pair<int, int>> directions = {
        {0, 1},  // arriba
        {0, -1}, // abajo
        {1, 0},  // derecha
        {-1, 0}  // izquierda
    };

    if (hayDiagonales) {
        directions.insert(directions.end(), {
            {1, 1},   // diagonal superior derecha
            {1, -1},  // diagonal inferior derecha
            {-1, 1},  // diagonal superior izquierda
            {-1, -1}  // diagonal inferior izquierda
        });
    }

    for (const auto& dir : directions) {
        int nx = node[0] + dir.first;  // Nueva coordenada x
        int ny = node[1] + dir.second; // Nueva coordenada y

        // Verificar que el vecino esté dentro de los límites del mapa
        if (nx >= 0 && nx < costmap_->getSizeInCellsX() &&
            ny >= 0 && ny < costmap_->getSizeInCellsY()) {
            
            // Verificar que el vecino no sea un obstáculo
            if (costmap_->getCost(nx, ny) != costmap_2d::LETHAL_OBSTACLE &&
                obstacleFree(node[0], node[1], nx, ny)) {
                neighbors.push_back({nx, ny});
            }
        }
    }

    return neighbors;
}


bool AStarPlanner::computeAStar(const std::vector<int>& start, const std::vector<int>& goal, 
                               std::vector<std::vector<int>>& sol) {
    // Initialize cost maps
    using Node = std::vector<int>;
    bool verbose = false;

    auto compare = [](const std::pair<Node, double>& a, const std::pair<Node, double>& b) {
        return a.second > b.second;
    };

    std::priority_queue<std::pair<Node, double>, std::vector<std::pair<Node, double>>, decltype(compare)> open_set(compare);
    open_set.emplace(start, heuristic(start, goal));

    std::unordered_map<Node, Node, hash_vector> came_from;
    std::unordered_map<Node, double, hash_vector> g_score;
    std::unordered_map<Node, double, hash_vector> f_score;
    g_score[start] = 0;
    f_score[start] = heuristic(start, goal);

    if (verbose){
        ROS_INFO("A* Algorithm Starting...");
        ROS_INFO("Start Node: [%d, %d]", start[0], start[1]);
        ROS_INFO("Goal Node: [%d, %d]", goal[0], goal[1]);
    }


    while (!open_set.empty()) {
        Node current = open_set.top().first;
        open_set.pop();
        if (verbose){
            ROS_INFO("Current Node: [%d, %d]", current[0], current[1]);
        }
        publishExploredNode(current); //ESTO SEÑALA LO QUE HACE EL ALGORITMO, OJO QUE SI LO PONES ES MUCHÍSIMO MÁS LENTO

        if (current == goal) {
            if (verbose){
                ROS_INFO("Goal Reached!: [%d, %d] - [%d, %d]", current[0], current[1], goal[0], goal[1]);
            }
            reconstructPath(came_from, current, sol);
            publishOptimalPath(sol);
            return true;
        }

        for (const auto& neighbor : getNeighbors(current)) {
            if (g_score.find(neighbor) == g_score.end()) {
                g_score[neighbor] = std::numeric_limits<double>::infinity();
            }
            if (f_score.find(neighbor) == f_score.end()) {
                f_score[neighbor] = std::numeric_limits<double>::infinity();
            }

            double tentative_g_score = g_score[current] + heuristic(current, neighbor);
            if (tentative_g_score < g_score[neighbor]) {
                if (verbose){
                    ROS_INFO("Updating Neighbor: [%d, %d]: %f", neighbor[0], neighbor[1], tentative_g_score);
                }
                

                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g_score;
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal);

                open_set.emplace(neighbor, f_score[neighbor]);

                if (verbose){
                    ROS_INFO("Neighbor Updated: [%d, %d], g_score: %f, f_score: %f", neighbor[0], neighbor[1], g_score[neighbor], f_score[neighbor]);
                }
            }
        }
    }

    return false;
}

std::vector<std::vector<int>> AStarPlanner::reconstructPath( std::unordered_map<std::vector<int>, std::vector<int>, hash_vector>& cameFrom, std::vector<int>& current,
    std::vector<std::vector<int>>& sol) {
    sol.clear();
    sol.push_back(current);
    while (cameFrom.find(current) != cameFrom.end()) {
        current = cameFrom[current];
        sol.push_back(current);
    }
    return sol;
}

bool AStarPlanner::obstacleFree(const unsigned int x0, const unsigned int y0, 
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

void AStarPlanner::getPlan(const std::vector<std::vector<int>> sol, std::vector<geometry_msgs::PoseStamped>& plan){

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
