# Exercise 1

To execute the first package use:

    roslaunch first_package start_stage.launch

# Exercise 2

To execute the second package helloWorld use:

    roscore
    rosrun second_package helloWorld

and to execute the listener and talker node use:

    roslaunch second_package start_communication.launch


# Exercise 3

first execute the first package (Exercise 1) and next 
execute thenext command to show the current position of Audrie:

    rosrun teleop_twist_keyboard_cpp robot_location
