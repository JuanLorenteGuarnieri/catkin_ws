#include <pluginlib/class_list_macros.h>

#include "AROB_lab4/llc_local_planner.h"



//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(p14_llc_local_planner::LLCLocalPlanner,nav_core::BaseLocalPlanner)

using namespace std;

namespace p14_llc_local_planner{

	double euclideanDistance(const geometry_msgs::Pose pose1, const geometry_msgs::Pose pose2){
		double ex = pose2.position.x - pose1.position.x;
		double ey = pose2.position.y - pose1.position.y;

		return std::sqrt(ex*ex+ey*ey);
	}

	LLCLocalPlanner::LLCLocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
		initialize(name, tf, costmap_ros);
	}

	void LLCLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){

		if(!initialized_){

			// std::cout << "Initialize LLCLocalPlanner..." << std::endl;

			costmap_ros_ = costmap_ros;
			costmap_ = costmap_ros_->getCostmap();
			tf_ = tf;

			ros::NodeHandle nh("~/" + name);
			ros::NodeHandle nh_local("~/local_costmap/");

			nh.param("kalpha", kalpha_, 0.0);
			nh.param("krho", krho_, 0.0);
			nh.param("kbeta", kbeta_, 0.0);
			nh.param("rho_th", rho_th_, 0.0);

			nh_local.getParam("robot_radius", robot_radius_);

			std::cout << "Parameters: kalpha: " << kalpha_ << ", krho: " << krho_ << ", kbeta: " << kbeta_;
			std::cout << ", rho_th: " << rho_th_ << std::endl;
			std::cout << "Robot radius: " << robot_radius_ << std::endl;

			initialized_ = true;

		}else{
			ROS_WARN("This planner has already been initialized, doing nothing.");
		}

	}

	bool LLCLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){

		std:cout << "setPlan  LLCLocalPlanner..." << std::endl;
		
		if(!initialized_){
			ROS_ERROR("THe planner has not been initialized.");
			return false;
		}

		//reset the global plan
		global_plan_.clear();

		// update obstacle info from costmap
		costmap_ = costmap_ros_->getCostmap();

		//Prune the plan to store the set of points within the local costmap
		for (auto it = plan.begin(); it != plan.end(); ++it){

			unsigned mx, my;
			if (costmap_->worldToMap((*it).pose.position.x, (*it).pose.position.y, mx, my))
				global_plan_.push_back((*it));
		}

		if (global_plan_.empty()){
			ROS_WARN("Global plan empty");
			return false;
		}

	   	return true;
	}

	bool LLCLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
	    //Compute the velocity command (v,w) for a differential-drive robot

		std::cout << "ComputeVelocityCommands ..." << std::endl;

		if(!initialized_){
			ROS_ERROR("THe planner has not been initialized.");
			return false;
		}

		//Get robot and goal pose in the global frame of the local costmap
		geometry_msgs::PoseStamped robot_pose;
		costmap_ros_->getRobotPose(robot_pose);
		geometry_msgs::PoseStamped goal = global_plan_.back();

		for (size_t i = 0; i < global_plan_.size(); ++i) {
			ROS_INFO("Global plan waypoint %lu: x = %f, y = %f", i, global_plan_[i].pose.position.x, global_plan_[i].pose.position.y);
		}

		//Read obstacle information from the costmap
		costmap_ = costmap_ros_->getCostmap();
		for (unsigned int i=0; i<costmap_->getSizeInCellsX()-1; i++){
			for (unsigned int j=0; j<costmap_->getSizeInCellsY()-1; j++){
				if (costmap_->getCost(i,j) == costmap_2d::LETHAL_OBSTACLE){
					double obs_wx, obs_wy;
					geometry_msgs::Pose obs_pose;					
					costmap_->mapToWorld(i, j, obs_pose.position.x, obs_pose.position.y); //mapToWorld returns coordinates in the global frame of the local costmap
					// std::cout << "Obs coordinates: " << obs_pose.position.x << ", " << obs_pose.position.y << std::endl;
					
					if (euclideanDistance(robot_pose.pose, obs_pose) < 1.5*robot_radius_){
						ROS_ERROR("Imminent collision");
						return false;
					}

				}
			}
		}
		//include here your code for the low level controller
		// Calculate polar coordinates (p, alpha, B)
		double dx = goal.pose.position.x - robot_pose.pose.position.x;
		double dy = goal.pose.position.y - robot_pose.pose.position.y;
		double rho = euclideanDistance(goal.pose, robot_pose.pose);
		double theta = tf::getYaw(robot_pose.pose.orientation);
		double alpha = -theta + atan2(dy, dx);
		double beta = -theta - alpha;



		// Control gains (assumed values)
		// Set linear and angular velocities based on the control law
		cmd_vel.linear.x = krho_ * rho;
		cmd_vel.angular.z = kalpha_ * alpha + kbeta_ * beta;
		
		std::cout << "robot_pose position x: " << robot_pose.pose.position.x  << "  robot_pose position y: " << robot_pose.pose.position.y  << endl;
		std::cout << "goal position x: " << goal.pose.position.x  << "  goal position y: " << goal.pose.position.y  << endl;
		std::cout << "dx: " << dx << "   dy: " << dy << endl;
		std::cout << "Rho: " << rho << endl;
		std::cout <<  " Vel linear: " << cmd_vel.linear.x << endl;
		std::cout <<  " Vel angular: " << cmd_vel.angular.z << endl;

		return true;
	}

	bool LLCLocalPlanner::isGoalReached(){
		//Check if the robot has reached the position and orientation of the goal

		std::cout << "isGoalReached ..." << std::endl;

		if (! initialized_) {
			ROS_ERROR("This planner has not been initialized.");
			return false;
		}

		//Get robot and goal pose in the global frame of the local costmap
		geometry_msgs::PoseStamped robot_pose;
		costmap_ros_->getRobotPose(robot_pose);
		const geometry_msgs::PoseStamped goal = global_plan_.back();

		bool goalReached = false;

		//implement here the condition(s) to have reached the goal

		// Thresholds for distance and orientation
		double distance_tolerance = 0.01;  // Adjust as necessary
		double angle_tolerance = 0.01;  // Adjust as necessary (in radians)

		// Calculate the distance to the goal
		double dx = goal.pose.position.x - robot_pose.pose.position.x;
		double dy = goal.pose.position.y - robot_pose.pose.position.y;
		double rho = sqrt(dx * dx + dy * dy);
		if (rho_th_ > abs(dx)) {
			return false;
		}

		// Calculate orientation difference
		double theta_robot = tf::getYaw(robot_pose.pose.orientation);
		double theta_goal = tf::getYaw(goal.pose.orientation);
		double orientation_diff = fabs(theta_goal - theta_robot);

		if (orientation_diff > angle_tolerance) {
			return false;
		}
		// Check if both position and orientation are within thresholds
		// goalReached = (rho < distance_tolerance) && (orientation_diff < angle_tolerance);


		return goalReached;		
	}

};
