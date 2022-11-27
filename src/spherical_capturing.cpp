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

tf::Vector3 randomDirection(double offset){
    std::random_device rd; // obtain a random number from hardware
    std::mt19937 gen(rd()); // seed the generator
    std::default_random_engine generator;

    // std::uniform_int_distribution<> distr(0, 3);
    double MIN_THETA, MAX_THETA, MIN_PHI, MAX_PHI;

    MIN_PHI=0.0, MAX_PHI=360.0;

    std::uniform_real_distribution<double> distr_real1(MIN_PHI,MAX_PHI);
    // if (distr(gen)){
    //     MIN_THETA=0.0, MAX_THETA=15.0, MIN_PHI=0.0, MAX_PHI=360.0;
    // }
    // else if{
    //     MIN_THETA=0.0, MAX_THETA=45.0, MIN_PHI=.0, MAX_PHI=135.0;
    // }
    // ******** TODO *************
    // include angle limits to avoid collision
    double phi = distr_real1(gen);
    // std::cout << phi << std::endl;

    if (phi>225.0 && phi<315.0){
        MIN_THETA=0.0, MAX_THETA=15.0;
    }
    else{
        MIN_THETA=0.0, MAX_THETA=45.0;
    }

    std::uniform_real_distribution<double> distr_real2(MIN_THETA,MAX_THETA);
    double theta = distr_real2(gen);
    std::cout << theta << std::endl;
    std::cout << phi << std::endl;
    phi = phi*M_PI/180.0;
    theta = theta*M_PI/180.0;

    tf::Vector3 dir(offset*cos(phi)*sin(theta), offset*sin(phi)*sin(theta), offset*cos(theta));
    std::cout << "Length of dir " << dir.length() << std::endl;
    return dir;
}

Eigen::Matrix3d skew3(tf::Vector3 x){
    Eigen::Matrix3d x_hat;
    x_hat << 0.0, -1.0*x.getZ(), x.getY(), x.getZ(), 0.0, -1.0*x.getX(), -1.0*x.getY(), x.getX(), 0.0;
    return x_hat;
}

tf::Transform getTransformTipToProbe(tf::Vector3 t_tp, tf::Vector3 t_pt_0, double offset){
    // tf::Vector3 omega = t_tp.cross(t_pt_0);
    tf::Vector3 omega = t_pt_0.cross(-1.0*t_tp);
    omega = omega.normalize();
    // double theta = acos((double)t_tp.dot(-1.0*t_pt_0)/(offset*offset));
    double theta = acos((double)t_pt_0.dot(-1.0*t_tp)/(offset*offset));
    Eigen::Matrix3d omega_hat = skew3(omega);
    Eigen::Matrix3d R_tp_eigen = Eigen::Matrix3d::Identity() + sin(theta)*omega_hat+(1.0-cos(theta))*(omega_hat*omega_hat);
    std::cout << "Determinant of R " << R_tp_eigen.determinant() << std::endl;
    tf::Matrix3x3 R_tp(R_tp_eigen.coeff(0,0), R_tp_eigen.coeff(0,1), R_tp_eigen.coeff(0,2), 
                    R_tp_eigen.coeff(1,0), R_tp_eigen.coeff(1,1), R_tp_eigen.coeff(1,2),
                    R_tp_eigen.coeff(2,0), R_tp_eigen.coeff(2,1), R_tp_eigen.coeff(2,2));
    tf::Transform transform_tp(R_tp, t_tp);
    return transform_tp;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "spherical_capturing");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate rate(2000.0);

    int num_pts = 20;
    double offset = 0.03;

    std::ofstream world_ee;
    std::ofstream world_probe;

    // MoveIt Setups
    // ^^^^^^^^^^^^^^^^^
    static const std::string PLANNING_GROUP = "manipulator";
    // the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // Raw pointers used to refer to the planning group 
    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Scale movement speed
    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.1);  

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

    // Getting Basic Information
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
                std::ostream_iterator<std::string>(std::cout, ", "));
    std::cout << " " << std::endl;

    // Lookup known transforms
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    tf::TransformListener listener_tip;
    tf::TransformListener listener_probe_ee;
    tf::StampedTransform transform_wt;
    tf::StampedTransform transform_pe;
    tf::Transform transform_target;
    tf::Transform transform_wp;
    geometry_msgs::Pose target_pose;

    // lookup tip frame and probe-ee_link
    while (node_handle.ok()){
        try{
            listener_tip.lookupTransform("world", "tip_frame", ros::Time(0), transform_wt);
            listener_probe_ee.lookupTransform("probe", "ee_link", ros::Time(0), transform_pe);

            break;
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        rate.sleep();
    }

    // Start Planning
    // ^^^^^^^^^^^^^^^^^
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
    // plan a motion for this group to a desired pose for the end-effector
    // loop until reach the number of points being set
    world_ee.open("/home/yyin34/world_ee.txt");
    world_probe.open("/home/yyin34/world_probe.txt");

    for (int i=0; i<num_pts; ++i){
        // Evaluate unknown transforms
        // set t0, which is the fixed translation of probe-tip, it's also the initial translation of tip-probe
        tf::Vector3 t_pt_0(0.0, offset, 0.0);
        // call function to evaluate translation of tip-probe in general by spherical coordinates
        tf::Vector3 t_tp = randomDirection(offset);
        tf::Transform transform_tp = getTransformTipToProbe(t_tp, t_pt_0, offset);
        transform_wp = transform_wt*transform_tp;
        transform_target = transform_wp*transform_pe;
        static tf::TransformBroadcaster br;
        // while(1){
        //     br.sendTransform(tf::StampedTransform(transform_wp, ros::Time::now(), "world", "target_probe"));
        // }
        target_pose.orientation.x = transform_target.getRotation().x();
        target_pose.orientation.y = transform_target.getRotation().y();
        target_pose.orientation.z = transform_target.getRotation().z();
        target_pose.orientation.w = transform_target.getRotation().w();
        target_pose.position.x = transform_target.getOrigin().x();
        target_pose.position.y = transform_target.getOrigin().y();
        target_pose.position.z = transform_target.getOrigin().z();
        move_group.setPoseTarget(target_pose);
        // use the planner
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        ROS_INFO_NAMED("val_all_dir", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

        // Visualizing plans
        // We can also visualize the plan as a line with markers in RViz.
        ROS_INFO_NAMED("val_all_dir", "Visualizing plan as trajectory line");
        visual_tools.publishAxisLabeled(target_pose, "pose");
        visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
        visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
        visual_tools.trigger();
        bool confirm = true;
        std::string input;
        if (success){
            //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue");
            std::cout << "Enter 'y' to confirm the plan; 'n' to try next target pose; 'q' to quit the program: " << std::endl;
            std::cin >> input;
            if (input=="y" || input=="Y") confirm = true;
            else if (input=="n" || input=="N") confirm = false;
            else return 0;
        }
        else{
            std::cout << "Planning failed; Trying the next target pose ..." << std::endl;
        }
        if(confirm && success) {
            move_group.move();
            
            world_ee << transform_target.getOrigin().x() << " " << transform_target.getOrigin().y()
            << " " << transform_target.getOrigin().z() << " " << transform_target.getRotation().x()
            << " " << transform_target.getRotation().y() << " " << transform_target.getRotation().z()
            << " " << transform_target.getRotation().w() << "\n";

            world_probe << transform_wp.getOrigin().x() << " " << transform_wp.getOrigin().y()
            << " " << transform_wp.getOrigin().z() << " " << transform_wp.getRotation().x()
            << " " << transform_wp.getRotation().y() << " " << transform_wp.getRotation().z()
            << " " << transform_wp.getRotation().w() << "\n";

            input = "";
            while(input!="c"){
                std::cout << "Press 'c' to continue ... ";
                std::cin >> input;
            }
            std::cout << "# points collected: " << i << std::endl;
            move_group.setJointValueTarget(home_joints);
            move_group.move();
        }
        else i-=1;
    }
    world_ee.close();
    world_probe.close();
    return 0;
}
