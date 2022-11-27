// This script read the current robot state into a config file as the home position for homing process
/* Author: Yifan Yin */
#include <iostream>
#include <fstream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/JointState.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "set_home_position");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Rate rate(100);

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
     const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    moveit::core::RobotStatePtr current_state =  move_group.getCurrentState();

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // write to a configuration file
    std::ofstream config("/home/yyin34/catkin_cb2/src/motion_planning/config/home_pos.txt");
    if (config.is_open())
    {   
        for (int i=0; i<joint_group_positions.size(); ++i){
            config << joint_group_positions[i] << '\n';
        }
        config.close();
    }
    else std::cout << "Unable to open file";

    return 0;
}