#include "lab2_drones/follow_targets_3d.hpp"

FollowTargets3D::FollowTargets3D(ros::NodeHandle nh) : nh_(nh)
{
    // Read from parameters the path for the targets,
    // otherwise use a default value.
    if (!nh_.getParam("targets_file_path", targets_file_path_))
    {
        ROS_WARN("There is no 'targets_file_path' parameter. Using default value.");
        targets_file_path_ = "/home/qassiel/catkin_ws/src/p14_arob_lab2_drones/data/targets.txt";
    }
    // Try to open the targets file.
    if (!readTargets_(targets_file_path_))
    {
        ROS_ERROR("Could not read targets from file: %s", targets_file_path_.c_str());
        ros::shutdown();
    }

    // Assign a first goal and save the current index. In this case, the goal is the first target.
    current_goal_index_ = 0;
    current_goal_.pose.position = targets_[current_goal_index_].position;
    // Even if we only want the position, it is a good idea to initialize rotation to zero.
    current_goal_.pose.orientation = geometry_msgs::Quaternion();
    // ADDITIONAL EXERCISE: : Usually, it is a good practice to wait until odometry is received before sending
    // any goal. Implement the logic to achieve this.
    received_odom_ = false;

    // EXERCISE: Define your publishers and subscribers
    // Find the topic to send pose commands and the type of message it receives
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/command/pose", 1000);

    // EXERCISE: Find a topic to listen to the position of the quadrotor in order to send a new goal
    // once the previous is reached. 
    // INFO: In order to pass as a callback a member of a function, we have to pass a reference 
    // to the callback method and a reference to the object of the class:
    // https://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks
    // (THE LAST SENTENCE IN SUBSCRIPTIONS IS REALLY IMPORTANT)
    odom_sub_ = nh_.subscribe("/ground_truth/state", 1000, &FollowTargets3D::odometryCallback_, this);

    // ADDITIONAL EXERCISE: In some cases, robot controllers might need to receive the goal continuously
    // (e.g. at a frequency of 100Hz).
    // You will have to save the next_goal_id and send the goal in an interval.
    // Find how this is commonly done in ROS.
    // goal_timer_ = nh_.createTimer(ros::Duration(0.01), &FollowTargets3D::goalTimerCallback_, this);

    ROS_INFO("FollowTargets3D initialized");
}

bool FollowTargets3D::readTargets_(std::string file_path)
{
    //Open the file
    std::ifstream input_file;
    input_file.open(file_path, std::ifstream::in);
    if (!input_file) {
        return false;
    }
    targets_.clear();
    geometry_msgs::Pose tempPose;
    std::string line;

    while (std::getline(input_file, line)) {
        std::istringstream iss(line);
        std::string value;

        // Read and parse x, y, z values separated by semicolons
        std::getline(iss, value, ';');
        tempPose.position.x = std::stod(value);
        
        std::getline(iss, value, ';');
        tempPose.position.y = std::stod(value);
        
        std::getline(iss, value, ';');
        tempPose.position.z = std::stod(value);

        targets_.push_back(tempPose);
    }

    // Close the file
    input_file.close();

    // Show the targets for debugging
    for (const auto& target : targets_) {
        std::cout << "Goal " << target.position.x << " "
                  << target.position.y << " "
                  << target.position.z << std::endl;
    }

    return true;
}

void FollowTargets3D::odometryCallback_(const nav_msgs::Odometry& msg)
{
    if (!received_odom_) {
        ROS_INFO("Odometry received, sending first goal");
        received_odom_ = true;
        pose_pub_.publish(current_goal_);
        return;
    }

    // Check if the current position is sufficiently close to the current goal
    double distance = sqrt(
        pow(current_goal_.pose.position.x - msg.pose.pose.position.x, 2) +
        pow(current_goal_.pose.position.y - msg.pose.pose.position.y, 2) +
        pow(current_goal_.pose.position.z - msg.pose.pose.position.z, 2)
    );

    // Define a threshold for being "sufficiently close"
    double threshold = 0.5; // meters

    if (distance < threshold)
    {
        // Move to the next target
        current_goal_index_++;
        
        if (current_goal_index_ < targets_.size())
        {
            current_goal_.pose.position = targets_[current_goal_index_].position;
            ROS_INFO("New goal published: [%.2f, %.2f, %.2f]",
                     current_goal_.pose.position.x,
                     current_goal_.pose.position.y,
                     current_goal_.pose.position.z);
            pose_pub_.publish(current_goal_);
        }
        else
        {
            ROS_INFO("All targets reached.");
            ros::shutdown();
        }
    }
}

// Timer callback to publish the current goal at regular intervals
void FollowTargets3D::goalTimerCallback_(const ros::TimerEvent&)
{
    // If odometry has been received, continuously publish the current goal
    if (received_odom_) {
        pose_pub_.publish(current_goal_);
    }
}