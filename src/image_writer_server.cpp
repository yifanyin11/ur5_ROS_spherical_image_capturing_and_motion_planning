#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vision_guided_planning/image_capture.h>
#include <sstream>

class ImageServer{
private:
    cv:Mat image;
    ros::NodeHandle nh;
    ros::Subscriber img_sub;

public:
    ImageServer(ros::NodeHandle& nh, const std::string& topic):
    nh(nh){
    img_sub = nh.subscribe(topic, 10, &ImageServer::imageCallback, this);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg){
        try{
            image = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.\n", msg->encoding.c_str());
        }
    }
    void saveImage(vision_guided_planning::image_capture::Request &req, vision_guided_planning::image_capture::Response &res){
        std::string img_full_path = req.path+req.img_name;
        if(!image.empty()){
            if (!cv::imwrite(img_full_path, image)){
                res.status = 0;
                std::cout << "Image cannot be saved as '" << img_full_path << "'." << std::endl;
            }
            else{
                std::cout << "Image saved in '" << img_full_path << "'." << std::endl;
                res.status = 1;
            }
        }
        else{
            res.status = 0;
            ROS_ERROR("Fail to capture image.\n");
        }
    }
};


int main(int argc, char** argv){
    std::string topic;
    ros::init(argc, argv, "image_writer_server");
    ros::NodeHandle nh;
    ImageServer imgSrvr(nh, topic);
    ros::ServiceServer service = nh.adverseService("image_capture", &ImageServer::saveImage, &imgSrvr);
    ros::spin();    
}

