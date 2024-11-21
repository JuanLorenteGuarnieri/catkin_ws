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

	bool LLCLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
		std::cout << "ComputeVelocityCommands ..." << std::endl;

		if (!initialized_) {
			ROS_ERROR("The planner has not been initialized.");
			return false;
		}

		// Get robot and goal pose in the global frame of the local costmap
		geometry_msgs::PoseStamped robot_pose;
		costmap_ros_->getRobotPose(robot_pose);
		geometry_msgs::PoseStamped goal = global_plan_.back();

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

		// Calculate polar coordinates (rho, alpha, beta)
		double dx = goal.pose.position.x - robot_pose.pose.position.x;
		double dy = goal.pose.position.y - robot_pose.pose.position.y;
		double rho = euclideanDistance(goal.pose, robot_pose.pose);
		double theta = tf::getYaw(robot_pose.pose.orientation);
		double alpha = angles::normalize_angle(atan2(dy, dx) - theta);
		double beta = angles::normalize_angle(-theta - alpha);

		// Compute control law for linear and angular velocities
		cmd_vel.linear.x = krho_ * rho; // Linear velocity
		cmd_vel.angular.z = kalpha_ * alpha + kbeta_ * beta; // Angular velocity

		if (krho_ <= 0 || kbeta_ >= 0 || (kalpha_ - krho_ <= 0))
		{
			cmd_vel.linear.x = 0.0;
			cmd_vel.angular.z = 0.0;
		}

		// Debug information
		ROS_INFO("Robot position: (%f, %f), Goal position: (%f, %f)", 
				robot_pose.pose.position.x, robot_pose.pose.position.y, 
				goal.pose.position.x, goal.pose.position.y);
		ROS_INFO("Rho: %f, Alpha: %f, Beta: %f", rho, alpha, beta);
		ROS_INFO("Linear Velocity: %f, Angular Velocity: %f", cmd_vel.linear.x, cmd_vel.angular.z);

		return true;
	}

	bool LLCLocalPlanner::isGoalReached() {
		std::cout << "isGoalReached ..." << std::endl;

		if (!initialized_) {
			ROS_ERROR("This planner has not been initialized.");
			return false;
		}

		// Get robot and goal pose in the global frame of the local costmap
		geometry_msgs::PoseStamped robot_pose;
		costmap_ros_->getRobotPose(robot_pose);
		const geometry_msgs::PoseStamped& goal = global_plan_.back();

		// Calculate distance and orientation difference
		double dx = goal.pose.position.x - robot_pose.pose.position.x;
		double dy = goal.pose.position.y - robot_pose.pose.position.y;
		double rho = sqrt(dx * dx + dy * dy);

		double theta_robot = tf::getYaw(robot_pose.pose.orientation);
		double theta_goal = tf::getYaw(goal.pose.orientation);
		double orientation_diff = fabs(angles::normalize_angle(theta_goal - theta_robot));

		// Check if both position and orientation are within thresholds
		if (rho < rho_th_ && orientation_diff < 0.1) {
			ROS_INFO("Goal reached successfully.");
			return true;
		}

		return false;
	}

};
