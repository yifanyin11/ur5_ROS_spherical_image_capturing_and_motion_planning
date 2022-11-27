// This script read the home position from a config file and home the robot to that position
/* Author: Yifan Yin */
#include <iostream>
#include <fstream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
    // ros setups
    ros::init(argc, argv, "robot_homing");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate rate(1000);

    // moveit setups for planning
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    move_group.setMaxVelocityScalingFactor(0.03);
    move_group.setMaxAccelerationScalingFactor(0.1);

    // moveit setups for visualization
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");

    // vector to store home position
    std::vector<double> joint_group_positions;
    
    // read home position from the configuration file
    std::ifstream config("/home/yyin34/catkin_cb2/src/motion_planning/config/home_pos.txt");
    std::string value_str;

    if (config.is_open())
    {   
        while(std::getline(config, value_str)){
            joint_group_positions.push_back(std::stod(value_str));
            // std::cout << std::stod(value_str) << std::endl;
        }
        config.close();
    }
    else std::cout << "Unable to open file";

    // set joint space goal
    move_group.setJointValueTarget(joint_group_positions);

    // plan 
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("robot_homing", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

    // visualization
    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    bool confirm = false;
    std::string input;

    //move the robot
    move_group.move();


    ros::shutdown();
    return 0;
}