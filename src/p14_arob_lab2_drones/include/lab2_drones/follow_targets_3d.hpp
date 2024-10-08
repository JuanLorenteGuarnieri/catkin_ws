#ifndef FOLLOW_TARGETS_3D_HPP
#define FOLLOW_TARGETS_3D_HPP

#include <vector>
#include <math.h>
#include <fstream>

#include <iostream>
#include <sstream>
#include <stdio.h> 

// EXERCISE: Add the dependencies to the CMakeLists.txt and package.xml files
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h> // The added dependency should be tf2

class FollowTargets3D
{
    
public:
	FollowTargets3D(ros::NodeHandle nh );

    ~FollowTargets3D() {}
    
private:
    ros::NodeHandle nh_;

    // Variables for the publisher, subscriber, and timer
    ros::Publisher pose_pub_;              // Publisher for sending pose commands
    ros::Subscriber odom_sub_;             // Subscriber for receiving odometry
    ros::Timer goal_timer_;                // Timer to send the goal at regular intervals

    // Variable to manage whether odometry has been received
    bool received_odom_;

    // Vector of goals and the index of the current goal
    std::vector<geometry_msgs::Pose> targets_;
    geometry_msgs::PoseStamped current_goal_;
    int current_goal_index_;

    // Path to the target file
    std::string targets_file_path_;

    // Private methods for callbacks and reading target files
    
    bool readTargets_(std::string file);
    
    //  EXERCISE: Implement the logic to publish new goals when the previous one is reached
    //  (more info in the constructor of this class).
    void odometryCallback_(const nav_msgs::Odometry& msg);
    
    void goalTimerCallback_(const ros::TimerEvent& event);            // Timer callback
};


#endif