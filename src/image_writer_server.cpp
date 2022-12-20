#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vision_guided_planning/image_capture.h>
#include <sstream>

class ImageServer{
private:
    cv_bridge::CvImagePtr img_ptr;
    ros::NodeHandle nh;
    ros::Subscriber img_sub;

public:
    ImageServer(ros::NodeHandle& nh, const std::string& topic):
    nh(nh){
    img_sub = nh.subscribe(topic, 10, &ImageServer::imageCallback, this);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg){
        try{
            img_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.\n", msg->encoding.c_str());
        }
    }
    bool saveImage(vision_guided_planning::image_capture::Request &req, vision_guided_planning::image_capture::Response &res){
        std::string img_full_path = req.path+req.image_name;
        if(!(img_ptr->image.empty())){
            if (!cv::imwrite(img_full_path, img_ptr->image)){
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
    ros::ServiceServer service = nh.advertiseService("image_capture", &ImageServer::saveImage, &imgSrvr);
    ros::spin();    
}

