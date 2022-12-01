#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <random>

// Utils
tf::Vector3 randomDirection(double radius){
    std::random_device rd; // obtain a random number from hardware
    std::mt19937 gen(rd()); // seed the generator
    std::default_random_engine generator;

    double MIN_THETA, MAX_THETA, MIN_PHI, MAX_PHI;

    MIN_PHI=0.0, MAX_PHI=360.0;

    std::uniform_real_distribution<double> distr_real1(MIN_PHI,MAX_PHI);
    
    // ******** TODO *************
    // include angle limits to avoid collision
    double phi = distr_real1(gen);
    // std::cout << phi << std::endl;

    MIN_THETA=0.0, MAX_THETA=60.0;

    std::uniform_real_distribution<double> distr_real2(MIN_THETA,MAX_THETA);
    double theta = distr_real2(gen);
    std::cout << theta << std::endl;
    std::cout << phi << std::endl;
    phi = phi*M_PI/180.0;
    theta = theta*M_PI/180.0;

    tf::Vector3 dir(radius*cos(phi)*sin(theta), radius*sin(phi)*sin(theta), radius*cos(theta));
    std::cout << "Length of dir " << dir.length() << std::endl;
    return dir;
}

Eigen::Matrix3d skew3(tf::Vector3 x){
    Eigen::Matrix3d x_hat;
    x_hat << 0.0, -1.0*x.getZ(), x.getY(), x.getZ(), 0.0, -1.0*x.getX(), -1.0*x.getY(), x.getX(), 0.0;
    return x_hat;
}

tf::Transform getTransformObjToCamera(tf::Vector3 t_oc, tf::Vector3 t_co_0, double radius){
    // tf::Vector3 omega = t_oc.cross(t_pt_0);
    tf::Vector3 omega = t_co_0.cross(-1.0*t_oc);
    omega = omega.normalize();
    // double theta = acos((double)t_oc.dot(-1.0*t_pt_0)/(radius*radius));
    double theta = acos((double)t_co_0.dot(-1.0*t_oc)/(radius*radius));
    Eigen::Matrix3d omega_hat = skew3(omega);
    Eigen::Matrix3d R_oc_eigen = Eigen::Matrix3d::Identity() + sin(theta)*omega_hat+(1.0-cos(theta))*(omega_hat*omega_hat);
    std::cout << "Determinant of R " << R_oc_eigen.determinant() << std::endl;
    tf::Matrix3x3 R_oc(R_oc_eigen.coeff(0,0), R_oc_eigen.coeff(0,1), R_oc_eigen.coeff(0,2), 
                    R_oc_eigen.coeff(1,0), R_oc_eigen.coeff(1,1), R_oc_eigen.coeff(1,2),
                    R_oc_eigen.coeff(2,0), R_oc_eigen.coeff(2,1), R_oc_eigen.coeff(2,2));
    tf::Transform transform_tp(R_oc, t_oc);
    return transform_tp;
}

void transform2PoseMsg(tf::Transform& transform, geometry_msgs::Pose& pose){
    pose.position.x = transform.getOrigin().x();
    pose.position.y = transform.getOrigin().y();
    pose.position.z = transform.getOrigin().z();

    pose.orientation.x = transform.getRotation().x();
    pose.orientation.y = transform.getRotation().y();
    pose.orientation.z = transform.getRotation().z();
    pose.orientation.w = transform.getRotation().w();
}

// Main function
int main(int argc, char** argv)
{
    ros::init(argc, argv, "spherical_capturing");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate rate(2000.0);

    double min_radius = 0.05, max_radius = 0.2;
    int num_layers = 3;
    int max_scan_per_layer = 3;

    // Register object poses
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    tf::TransformListener listener;

    tf::StampedTransform transform_wo1;
    tf::StampedTransform transform_wo2;
    tf::StampedTransform transform_ce;
    tf::Transform transform_target1;
    tf::Transform transform_target2;
    tf::Transform transform_wo1_final;
    tf::Transform transform_wo2_final;

    // lookup camera_link to ee_link
    ROS_INFO("Lookup camera_link to ee_link ---");
    while (node_handle.ok()){
        try{
            listener.lookupTransform("camera_link", "ee_link", ros::Time(0), transform_ce);
            break;
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        rate.sleep();
    }

    // register object transforms
    ROS_INFO("Waiting object1 tranform to be registered ---");
    while (node_handle.ok()){
        try{
            listener.lookupTransform("world", "object1", ros::Time(0), transform_wo1);
            break;
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        rate.sleep();
    }
    ROS_INFO("Object1 tranform found by main!");

    // register box to world transform
    ROS_INFO("Waiting object2 tranform to be registered ---");
    while (node_handle.ok()){
        try{
            listener.lookupTransform("world", "object2", ros::Time(0), transform_wo2);
            break;
        }
        catch (tf::TransformException ex){
            // ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        rate.sleep();
    }
    ROS_INFO("Object2 tranform found by main!");

    // MoveIt Setups
    // ^^^^^^^^^^^^^^^^^
    static const std::string PLANNING_GROUP = "manipulator";
    // the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // Raw pointers used to refer to the planning group 
    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = false;

    // Scale movement speed
    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.1);  

    // Visualization
    // ^^^^^^^^^^^^^^^^^
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "MoveGroupInterface", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // Loop for scanning the first object
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    int count = 0, layer = 0;
    double radius = min_radius;
    geometry_msgs::Pose target_pose1, target_pose2;

    while (node_handle.ok() && layer<num_layers){
        // keep track on loop parameters
        if (count++<max_scan_per_layer){
            count = 0;
            layer++;
            radius = min_radius+layer*(max_radius-min_radius)/(num_layers-1);
        }
        // Evaluate unknown transforms
        // set t0, which is the fixed translation of camera-obj, it's also the initial translation of obj-camera
        tf::Vector3 t_co1_0(0.0, 0.0, -1.0*radius);
        // call function to evaluate translation of tip-probe in general by spherical coordinates
        tf::Vector3 t_o1c = randomDirection(radius);
        tf::Transform transform_o1c = getTransformObjToCamera(t_o1c, t_co1_0, radius);
        transform_target1 = transform_wo1*transform_o1c*transform_ce;
        transform2PoseMsg(transform_target1, target_pose1);

        // set target
        move_group.setPoseTarget(target_pose1);
        // plan 
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("full_ultrasound_scan", "Visualizing plan1 (pose goal) %s", success ? "" : "FAILED");
        // visualization
        visual_tools.deleteAllMarkers();
        visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
        visual_tools.trigger();
        // move the robot
        move_group.move();
        ros::Duration(1.0).sleep();

        // ******** TODO *************
        // capture images from ros topics
    }

    count = 0;

    while (node_handle.ok() && count<max_scan_per_layer){
        // keep track on loop parameters
        if (count++<max_scan_per_layer){
            count = 0;
            layer++;
            radius = min_radius+layer*(max_radius-min_radius)/(num_layers-1);
        }
        // Evaluate unknown transforms
        // set t0, which is the fixed translation of camera-obj, it's also the initial translation of obj-camera
        tf::Vector3 t_co2_0(0.0, 0.0, -1.0*radius);
        // call function to evaluate translation of tip-probe in general by spherical coordinates
        tf::Vector3 t_o2c = randomDirection(radius);
        tf::Transform transform_o2c = getTransformObjToCamera(t_o2c, t_co2_0, radius);
        transform_target2 = transform_wo2*transform_o2c*transform_ce;
        transform2PoseMsg(transform_target2, target_pose2);

        // set target
        move_group.setPoseTarget(target_pose2);
        // plan 
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("full_ultrasound_scan", "Visualizing plan1 (pose goal) %s", success ? "" : "FAILED");
        // visualization
        visual_tools.deleteAllMarkers();
        visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
        visual_tools.trigger();
        // move the robot
        move_group.move();
        ros::Duration(1.0).sleep();

        // ******** TODO *************
        // capture images from ros topics
    }

    return 0;
}
