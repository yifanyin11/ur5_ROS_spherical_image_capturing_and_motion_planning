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

#include <vision_guided_planning/image_capture.h>
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

// Callbacks
int MOVE_STATUS=0;

void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& status)
{
 if (!status->status_list.empty())
   {
    actionlib_msgs::GoalStatus goalStatus = status->status_list.back();
    MOVE_STATUS = (int) goalStatus.status;
   }
}

bool captureImage(ros::NodeHandle& nh, std::string& img_path, std::string& img_name){
    ros::ServiceClient client = nh.serviceClient<vision_guided_planning::image_capture>("image_capture");
    vision_guided_planning::image_capture srv;
    srv.request.path = img_path;
    srv.request.image_name = img_name;

    if(client.call(srv)){
        if (srv.response.status==1){
            ROS_INFO("Image saved.");
        }
        else{
            ROS_INFO("Save failed!");
        }
    }
    else{
        ROS_ERROR("Fail to call service image_capture!");
        return false;
    }
    return true;
}

// Main function
int main(int argc, char** argv)
{
    ros::init(argc, argv, "spherical_capturing_with_MP_plugin");
    ros::NodeHandle node_handle;
    ros::Publisher goalState_pub = node_handle.advertise<moveit_msgs::RobotState>("/rviz/moveit/update_custom_goal_state", 2000);

    ros::Subscriber move_status_sub = node_handle.subscribe("/follow_joint_trajectory/status", 1000, statusCallback);

    static tf::TransformBroadcaster br;
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

    std_msgs::Char msg;

    std::clock_t c_start;
    std::clock_t c_end;

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

    // Visualization
    // ^^^^^^^^^^^^^
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

    // Loop for scanning the first object
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    int count = 0, layer = 0, img_count=0;
    double radius = min_radius;
    geometry_msgs::Pose target_pose1, target_pose2;
    std::string img_path = "./";
    std::string img_name;

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

        // find the best solution
        tf::Transform transform_rotation;
        tf::Quaternion q;
        q.setEuler(0, 0, M_PI);
        transform_rotation.setRotation(q);

        robot_state::RobotState best_goal_state = scene->getCurrentStateNonConst();
        bool flag = false;
        std::vector<double> state_values(6);

        transform_wo1_final = transform_wo1;

        for (int i=0; i<4; ++i){
            // rotate world_obj
            transform_wo1_final = transform_wo1_final*transform_rotation;

            transform_target1 = transform_wo1_final*transform_o1c*transform_ce;
            // set translations
            transform2PoseMsg(transform_target1, target_pose1);
        
            goal_state.setFromIK(joint_model_group, target_pose1);
            if (!flag){
                best_goal_state = goal_state;
                flag=true;
            }
            else{
                if (goal_state.distance(scene->getCurrentStateNonConst())<best_goal_state.distance(scene->getCurrentStateNonConst())){
                    // Good ik
                    best_goal_state = goal_state;
                }
            }
        }

        best_goal_state.printStateInfo(std::cout);
        moveit::core::robotStateToRobotStateMsg(best_goal_state, goal_msg);
        // publish goal state msg
        goalState_pub.publish(goal_msg);

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

        ros::Duration(1.0).sleep();

//    while(node_handle.ok()){
//            br.sendTransform(tf::StampedTransform(transform_target1, ros::Time::now(), "world", "idle"));
//            rate.sleep();
//    }

        img_name = "obj1_"+std::to_string(img_count)+".png";
        if (!captureImage(node_handle, img_path, img_name)) continue;
        img_count++;

        scene->getCurrentStateNonConst().update();
        robot_model = scene->getRobotModel();
        joint_model_group = robot_model->getJointModelGroup("manipulator");
        goal_state = scene->getCurrentStateNonConst();

    }

    // plan for the second target

    count = 0;
    img_count = 0;

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

        // find the best solution
        tf::Transform transform_rotation;
        tf::Quaternion q;
        q.setEuler(0, 0, M_PI);
        transform_rotation.setRotation(q);

        robot_state::RobotState best_goal_state = scene->getCurrentStateNonConst();
        bool flag = false;
        std::vector<double> state_values(6);
        transform_wo2_final = transform_wo2;

        for (int i=0; i<4; ++i){
            transform_wo2_final = transform_wo2_final*transform_rotation;

            transform_target2 = transform_wo2_final*transform_o2c*transform_ce;

            transform2PoseMsg(transform_target2, target_pose2);

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

        img_name = "obj2_"+std::to_string(img_count)+".png";
        if (!captureImage(node_handle, img_path, img_name)) continue;
        img_count++;

        scene->getCurrentStateNonConst().update();
        robot_model = scene->getRobotModel();
        joint_model_group = robot_model->getJointModelGroup("manipulator");
        goal_state = scene->getCurrentStateNonConst();
    }
    
}
