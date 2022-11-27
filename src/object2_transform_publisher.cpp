#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "object2_transform_publisher");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate rate(2000.0);
    static tf::TransformBroadcaster br;
    ros::NodeHandle node_handle;
    tf::TransformListener listener_marker2;
    tf::StampedTransform transform_wb;
    while (node_handle.ok()){
        try{
            listener_marker2.lookupTransform("world", "marker2", ros::Time(0), transform_wb);
            break;
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        rate.sleep();
    }
    ROS_INFO("Object2 tranform successfully registered!");
    while(node_handle.ok()){
            br.sendTransform(tf::StampedTransform(transform_wb, ros::Time::now(), "world", "object2"));
            rate.sleep();
    }


}
