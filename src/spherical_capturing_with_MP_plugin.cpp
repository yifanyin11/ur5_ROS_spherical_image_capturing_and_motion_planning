#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <ros/ros.h>
#include "std_msgs/Char.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "actionlib_msgs/GoalStatusArray.h"
#include "actionlib_msgs/GoalStatus.h"

int MOVE_STATUS=0;

void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& status)
{
 if (!status->status_list.empty())
   {
//     actionlib_msgs::GoalStatus goalStatus = status->status_list[0];
     actionlib_msgs::GoalStatus goalStatus = status->status_list.back();
     MOVE_STATUS = (int) goalStatus.status;
   }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "spherical_capturing_with_MP_plugin");
    ros::NodeHandle node_handle;
    ros::Publisher command_pub = node_handle.advertise<std_msgs::Char>("demo3/gripper_request", 2000);
    ros::Publisher goalState_pub = node_handle.advertise<moveit_msgs::RobotState>("/rviz/moveit/update_custom_goal_state", 2000);

//    ros::Subscriber move_status_sub = node_handle.subscribe("/move_group/status/", 1000, statusCallback);

    ros::Subscriber move_status_sub = node_handle.subscribe("/follow_joint_trajectory/status", 1000, statusCallback);

//    ros::Subscriber move_status_sub = node_handle.subscribe("/execute_trajectory/status", 1000, statusCallback);
    static tf::TransformBroadcaster br;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate rate(2000.0);

    // Register block and box poses
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    tf::TransformListener listener_cube;
    tf::TransformListener listener_box;
    tf::TransformListener listener_gripper_ee;
    tf::StampedTransform transform_wc;
    tf::StampedTransform transform_wb;
    tf::StampedTransform transform_ge;
    tf::Transform transform_target1;
    tf::Transform transform_target2;
    tf::Transform transform_wc_final;
    tf::Transform transform_wb_final;

    std_msgs::Char msg;

    std::clock_t c_start;
    std::clock_t c_end;

    // lookup gripper_pick to ee_link
    ROS_INFO("Lookup gripper_pick to ee_link ---");
    while (node_handle.ok()){
        try{
            listener_gripper_ee.lookupTransform("gripper_pick", "ee_link", ros::Time(0), transform_ge);
            break;
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        rate.sleep();
    }

    // register cube to world transform
    ROS_INFO("Waiting cube tranform to be registered ---");
    while (node_handle.ok()){
        try{
            listener_cube.lookupTransform("world", "cube", ros::Time(0), transform_wc);
            break;
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        rate.sleep();
    }
    ROS_INFO("Cube tranform found by main!");

    // register box to world transform
    ROS_INFO("Waiting box tranform to be registered ---");
    while (node_handle.ok()){
        try{
            listener_box.lookupTransform("world", "box", ros::Time(0), transform_wb);
            break;
        }
        catch (tf::TransformException ex){
            // ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        rate.sleep();
    }
    ROS_INFO("Box tranform found by main!");

    // Calculate two target poses of ee_link
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    geometry_msgs::Pose target_pose1, target_pose2;
    tf::Quaternion q_cube_gripper, q_box_gripper;
    double offset_cube=0.0, offset_box=0.0;

    // gripper orientation wrt marker frames
    q_cube_gripper.setEuler(M_PI, 0, 0);
    q_cube_gripper.normalize();
    q_box_gripper.setRPY(M_PI, 0, 0);
    q_box_gripper.normalize();
    // gripper translation wrt marker frames (assume only offset along z)
    tf::Vector3 t_cube_gripper(0, 0, offset_cube);
    tf::Vector3 t_box_gripper(0, 0, offset_box);

    tf::Transform transform_cg(q_cube_gripper, t_cube_gripper);
    tf::Transform transform_bg(q_box_gripper, t_box_gripper);

//    target_pose1.orientation.x = transform_target1.getRotation().x();
//    target_pose1.orientation.y = transform_target1.getRotation().y();
//    target_pose1.orientation.z = transform_target1.getRotation().z();
//    target_pose1.orientation.w = transform_target1.getRotation().w();

//    target_pose1.position.x = transform_target1.getOrigin().x();
//    target_pose1.position.y = transform_target1.getOrigin().y();
//    target_pose1.position.z = transform_target1.getOrigin().z();

//    target_pose2.orientation.x = transform_target2.getRotation().x();
//    target_pose2.orientation.y = transform_target2.getRotation().y();
//    target_pose2.orientation.z = transform_target2.getRotation().z();
//    target_pose2.orientation.w = transform_target2.getRotation().w();

//    target_pose2.position.x = transform_target2.getOrigin().x();
//    target_pose2.position.y = transform_target2.getOrigin().y();
//    target_pose2.position.z = transform_target2.getOrigin().z()+0.2;

    // Visualization
    // ^^^^^^^^^^^^^^^^^
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();
    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "MoveGroupInterface", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();

    // MoveIt Setups
    // ^^^^^^^^^^^^^^^^^
    // Use planning scene monitor
    // **************************
    
    // setups
    planning_scene_monitor::LockedPlanningSceneRW scene(visual_tools.getPlanningSceneMonitor());
    scene->getCurrentStateNonConst().update();
    robot_model::RobotModelConstPtr robot_model = scene->getRobotModel();
    const robot_state::JointModelGroup* joint_model_group = robot_model->getJointModelGroup("manipulator");
    robot_state::RobotState goal_state = scene->getCurrentStateNonConst();   
    moveit_msgs::RobotState goal_msg;
//    std::string input;


    tf::Transform transform_rotation;
    tf::Quaternion q;
    q.setEuler(0, 0, M_PI);
    transform_rotation.setRotation(q);

    robot_state::RobotState best_goal_state = scene->getCurrentStateNonConst();
    bool flag = false;
    std::vector<double> state_values(6);

    transform_wc_final = transform_wc;

    for (int i=0; i<4; ++i){
        // rotate world_cube
        transform_wc_final = transform_wc_final*transform_rotation;

        transform_target1 = transform_wc_final*transform_cg*transform_ge;
        // set translations
        target_pose1.position.x = transform_target1.getOrigin().x();
        target_pose1.position.y = transform_target1.getOrigin().y();
        target_pose1.position.z = transform_target1.getOrigin().z();

        target_pose1.orientation.x = transform_target1.getRotation().x();
        target_pose1.orientation.y = transform_target1.getRotation().y();
        target_pose1.orientation.z = transform_target1.getRotation().z();
        target_pose1.orientation.w = transform_target1.getRotation().w();

        goal_state.setFromIK(joint_model_group, target_pose1);
        if (!flag){
            best_goal_state = goal_state;
            flag=true;
        }
        else{
            if (goal_state.distance(scene->getCurrentStateNonConst())<best_goal_state.distance(scene->getCurrentStateNonConst())){
                //Good ik
                best_goal_state = goal_state;
            }
        }
    }

    best_goal_state.printStateInfo(std::cout);
    moveit::core::robotStateToRobotStateMsg(best_goal_state, goal_msg);
    // publish goal state msg
    goalState_pub.publish(goal_msg);


//    bool confirm = false;
    // plan for the first target
//    while (!confirm){
//        goal_state.setToDefaultValues();
//        goal_state.setFromIK(joint_model_group, target_pose1);
//        goal_state.printStateInfo(std::cout);
//        moveit::core::robotStateToRobotStateMsg(goal_state, goal_msg);
//        // publish goal state msg
//        goalState_pub.publish(goal_msg);

//        std::cout << "Enter 'y' to confirm the IK solution pose; 'n' to try a new solve; 'q' to quit the program: " << std::endl;
//        std::cin >> input;
//        if (input=="y" || input=="Y") confirm = true;
//        else if (input=="n" || input=="N") confirm = false;
//        else return 0;
//    }

//    while(node_handle.ok()){
//        goalState_pub.publish(goal_msg);
//        if (MOVE_STATUS!=0){
//            break;
//        }
//        rate.sleep();
//    }
    ros::Duration(1.0).sleep();

    while(node_handle.ok()){
        if (!(MOVE_STATUS==3 || MOVE_STATUS==4)){
            break;
        }
        rate.sleep();
    }

    // waiting execution to be finished
    while(node_handle.ok()){
        if (MOVE_STATUS==3 || MOVE_STATUS==4){
            break;
        }
        rate.sleep();
    }

    ROS_INFO("MOVEIT execution completed!");

//     // Use move_group
//     //***************

//     static const std::string PLANNING_GROUP = "manipulator";
//     // the name of the planning group you would like to control and plan for.
//     moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
//     moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//     // Raw pointers used to refer to the planning group
//     const robot_state::JointModelGroup* joint_model_group =
//         move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

//     // Scale movement speed
//     move_group.setMaxVelocityScalingFactor(0.05);

//     // Getting Basic Information
//     // std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
//     //             std::ostream_iterator<std::string>(std::cout, ", "));

//     // Start Planning
//     // ^^^^^^^^^^^^^^^^^
//     // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
//     // plan a motion for this group to a desired pose for the end-effector

//     // plan for the first target
//     // call the custom planner to compute the plan and visualize it
//     bool success, confirm;

//     do{
//         move_group.setPoseTarget(target_pose1);
//         // use students' planner
//         // TODO *******************************
//         moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//         // ************************************

//         success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//         ROS_INFO_NAMED("pick_and_place", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

//         // Visualizing plans
//         // We can also visualize the plan as a line with markers in RViz.
//         ROS_INFO_NAMED("pick_and_place", "Visualizing plan 1 as trajectory line");
//         visual_tools.publishAxisLabeled(target_pose1, "pose1");
//         visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
//         visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//         visual_tools.trigger();
// //        visual_tools.publishTrajectoryPath(my_plan.trajectory_, my_plan.start_state_);
// //        visual_tools.trigger();
//         confirm = true;
//         std::string input;
//         if (success){
//             //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue");
//             std::cout << "Enter 'y' to confirm the plan; 'n' to redo the planning: " << std::endl;
//             std::cin >> input;
//             if (input=="y" || input=="Y") confirm = true;
//             else confirm = false;
//         }
//         else{
//             ROS_INFO("Planning failed; Redoing the planning ...");
//         }
//     }
//     while(!(confirm && success));
//     // move the robot
//     move_group.move();

    ros::Duration(1.0).sleep();

//    while(node_handle.ok()){
//            br.sendTransform(tf::StampedTransform(transform_target1, ros::Time::now(), "world", "idle"));
//            rate.sleep();
//    }

    // // plan for the second target
    scene->getCurrentStateNonConst().update();
    robot_model = scene->getRobotModel();
    joint_model_group = robot_model->getJointModelGroup("manipulator");
    goal_state = scene->getCurrentStateNonConst();

    transform_wb_final = transform_wb;

    for (int i=0; i<4; ++i){
        transform_wb_final = transform_wb_final*transform_rotation;

        transform_target2 = transform_wb_final*transform_bg*transform_ge;

        target_pose2.position.x = transform_target2.getOrigin().x();
        target_pose2.position.y = transform_target2.getOrigin().y();
        target_pose2.position.z = transform_target2.getOrigin().z()+0.2;

        target_pose2.orientation.x = transform_target2.getRotation().x();
        target_pose2.orientation.y = transform_target2.getRotation().y();
        target_pose2.orientation.z = transform_target2.getRotation().z();
        target_pose2.orientation.w = transform_target2.getRotation().w();

        goal_state.setFromIK(joint_model_group, target_pose2);
        if (!flag){
            best_goal_state = goal_state;
            flag=true;
        }
        else{
            if (goal_state.distance(scene->getCurrentStateNonConst())<best_goal_state.distance(scene->getCurrentStateNonConst())){
                //Good ik
                best_goal_state = goal_state;
            }
        }
    }

    goal_state.printStateInfo(std::cout);
    moveit::core::robotStateToRobotStateMsg(goal_state, goal_msg);

    goalState_pub.publish(goal_msg);

    ros::Duration(1.0).sleep();

    // waiting execution to be finished
    while(node_handle.ok()){
        if (!(MOVE_STATUS==3 || MOVE_STATUS==4)){
            break;
        }
        rate.sleep();
    }

    while(node_handle.ok()){
        if (MOVE_STATUS==3 || MOVE_STATUS==4){
            break;
        }
        rate.sleep();
    }


    ROS_INFO("MOVEIT execution completed!");

    ros::Duration(1.0).sleep();

    // do{
    //     move_group.setPoseTarget(target_pose2);
    //     // use students' planner
    //     // TODO *******************************
    //     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //     // ************************************

    //     success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //     ROS_INFO_NAMED("pick_and_place", "Visualizing plan 2 (pose goal) %s", success ? "" : "FAILED");

    //     // Visualizing plans
    //     // We can also visualize the plan as a line with markers in RViz.
    //     ROS_INFO_NAMED("pick_and_place", "Visualizing plan 2 as trajectory line");
    //     visual_tools.publishAxisLabeled(target_pose2, "pose2");
    //     visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    //     visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    //     visual_tools.trigger();
    //     visual_tools.publishTrajectoryPath(my_plan.trajectory_, my_plan.start_state_);
    //     visual_tools.trigger();
    //     confirm = true;
    //     // ask for students' confirmation
    //     std::string input;
    //     if (success){
    //         //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue");
    //         std::cout << "Enter 'y' to confirm the plan; 'n' to redo the planning: " << std::endl;
    //         std::cin >> input;
    //         if (input=="y" || input=="Y") confirm = true;
    //         else confirm = false;
    //     }
    //     else{
    //         ROS_INFO("Planning failed; Redoing the planning ...");
    //     }
    // } while(!(confirm && success));
    // //move the robot
    // move_group.move();

    // // waiting execution to be finished
    // while(ros::ok()){
    //     if (MOVE_STATUS==3){
    //         break;
    //     }
    // }


}
