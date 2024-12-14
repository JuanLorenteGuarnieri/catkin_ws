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
  	std::vector<std::vector<int>> solA;
    bool computed = computeAStar(point_start, point_goal, solA
    );
    if (computed){        
        getPlan(solA, plan);
        // add goal
        plan.push_back(goal);
    }else{
        ROS_WARN("No plan computed");
    }

    return computed;
}

double AStarPlanner::heuristic(const std::vector<int>& a, const std::vector<int>& b) {
    return std::hypot(b[0] - a[0], b[1] - a[1]);
}

std::vector<std::vector<int>> AStarPlanner::getNeighbors(const std::vector<int>& node) {
    std::vector<std::vector<int>> neighbors;
    std::vector<std::vector<int>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    for (const auto& dir : directions) {
        std::vector<int> neighbor = {node[0] + dir[0], node[1] + dir[1]};
        if (neighbor[0] >= 0 && neighbor[0] < costmap_->getSizeInCellsX() &&
            neighbor[1] >= 0 && neighbor[1] < costmap_->getSizeInCellsY() &&
            costmap_->getCost(neighbor[0], neighbor[1]) < costmap_2d::LETHAL_OBSTACLE) {
            neighbors.push_back(neighbor);
        }
    }
    return neighbors;
}


bool AStarPlanner::computeAStar(const std::vector<int>& start, const std::vector<int>& goal, 
                               std::vector<std::vector<int>>& sol) {
    // Initialize cost maps
    std::unordered_map<std::vector<int>, double, hash_vector> g, f;
    std::unordered_map<std::vector<int>, std::vector<int>, hash_vector> parent;

    // Set initial costs
    g[start] = 0;
    f[start] = heuristic(start, goal);

    // Priority queue for nodes to explore
    std::priority_queue<std::pair<double, std::vector<int>>, 
                        std::vector<std::pair<double, std::vector<int>>>, 
                        std::greater<>> open_set;

    open_set.emplace(f[start], start);
    std::unordered_set<std::vector<int>, hash_vector> visited;

    while (!open_set.empty()) {
        // Get the node with the smallest f(x) value
        auto [current_cost, current] = open_set.top();
        open_set.pop();

        // Goal reached
        if (current == goal) {
            sol.clear();
            for (auto node = goal; node != start; node = parent[node]) {
                sol.push_back(node);
            }
            sol.push_back(start);
            std::reverse(sol.begin(), sol.end());
            return true;
        }

        // Mark current node as visited
        visited.insert(current);

        // Explore neighbors
        for (const auto& neighbor : getNeighbors(current)) {
            if (visited.count(neighbor)) continue; // Skip visited nodes

            double tentative_g = g[current] + distance(current[0],current[1], neighbor[0], neighbor[1]);
            if (g.find(neighbor) == g.end() || tentative_g < g[neighbor]) {
                // Update costs
                g[neighbor] = tentative_g;
                f[neighbor] = g[neighbor] + heuristic(neighbor, goal);
                parent[neighbor] = current;

                // Add to open set
                open_set.emplace(f[neighbor], neighbor);
            }
        }
    }

    return false; // No path found
}

/*
bool AStarPlanner::computeAStar2(const std::vector<int> start, const std::vector<int> goal, 
                        std::vector<std::vector<int>>& sol) {
    // Inicializar estructuras de datos
    std::priority_queue<std::pair<double, std::vector<int>>, 
                        std::vector<std::pair<double, std::vector<int>>>, 
                        std::greater<std::pair<double, std::vector<int>>>> open_set;
    std::set<std::vector<int>> closed_set;
    std::map<std::vector<int>, double> g_cost; // Costo acumulado (g(x))
    std::map<std::vector<int>, double> f_cost; // Costo total (f(x))
    std::map<std::vector<int>, std::vector<int>> parents; // Para reconstruir el camino

    // Inicializar nodo inicial
    g_cost[start] = 0.0;
    f_cost[start] = heuristic(start, goal); // \( f(x) = h(x) \)
    open_set.push({f_cost[start], start});

    // Marker para visualización
    visualization_msgs::Marker edge_marker;
    edge_marker.header.frame_id = "map";
    edge_marker.header.stamp = ros::Time::now();
    edge_marker.ns = "a_path";
    edge_marker.type = visualization_msgs::Marker::LINE_LIST;
    edge_marker.scale.x = 0.05;
    edge_marker.color.r = 0.0;
    edge_marker.color.g = 0.0;
    edge_marker.color.b = 1.0;
    edge_marker.color.a = 1.0;
    edge_marker.points.clear();

    while (!open_set.empty()) {
        // Seleccionar el nodo con menor \( f(x) \) en el open_set
        auto current = open_set.top().second;
        open_set.pop();

        // Si alcanzamos el objetivo, reconstruir la solución
        if (current == goal) {
            std::vector<int> node = goal;
            while (node != start) {
                sol.push_back(node);
                node = parents[node];
            }
            sol.push_back(start);
            std::reverse(sol.begin(), sol.end());
            return true;
        }

        // Añadir el nodo actual al conjunto cerrado
        closed_set.insert(current);

        // Expandir vecinos
        for (auto& neighbor : getNeighbors(current)) {
            if (closed_set.count(neighbor) > 0 || 
                costmap_->getCost(neighbor[0], neighbor[1]) == costmap_2d::LETHAL_OBSTACLE) {
                continue; // Ignorar nodos ya visitados o en obstáculos
            }

            // Calcular costos
            double tentative_g = g_cost[current] + distance(current[0], current[1], neighbor[0], neighbor[1]);
            if (g_cost.find(neighbor) == g_cost.end() || tentative_g < g_cost[neighbor]) {
                g_cost[neighbor] = tentative_g;
                f_cost[neighbor] = g_cost[neighbor] + heuristic(neighbor, goal);
                parents[neighbor] = current;

                // Añadir a open_set
                open_set.push({f_cost[neighbor], neighbor});

                // Visualización
                geometry_msgs::Point p1, p2;
                costmap_->mapToWorld(current[0], current[1], p1.x, p1.y);
                costmap_->mapToWorld(neighbor[0], neighbor[1], p2.x, p2.y);
                p1.z = p2.z = 0.0;
                edge_marker.points.push_back(p1);
                edge_marker.points.push_back(p2);
                marker_pub_.publish(edge_marker);
            }
        }
    }

    return false; 
}

*/
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
