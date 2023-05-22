

#include <ros/ros.h> 
#include "GXDevice.h"
#include "StereoCamera.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <chrono>


int main(int argc, char** argv){
    ros::init(argc, argv,"image_publisher_node"); 
    ros::NodeHandle handle; 
    GXDevice VisionDevice; 
    VisionDevice.Init(); 
    std::shared_ptr<vn::StereoCamera> Camera = std::make_shared<vn::StereoCamera>(&VisionDevice); 

    image_transport::ImageTransport it(handle);
    image_transport::Publisher left_pub = it.advertise("left/image_raw", 30);
    image_transport::Publisher right_pub = it.advertise("right/image_raw", 5);

    sensor_msgs::ImagePtr image_msg; 

    ros::Rate rate(5); // 30fps
    int i = 0;
    while(ros::ok()) {
        Camera->AquireImage(); 
        i++;
        sensor_msgs::Image msg_left, msg_right;
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        cv::Mat frame_left = Camera->GetLeftImage();
        cv::Mat frame_right = Camera->GetRightImage();
        //cv::Mat frame_left_resized;
        //cv::resize(frame_left,frame_left_resized, cv::Size(640,640));

        //cv::Mat frame_right_resized;
        //cv::resize(frame_right,frame_right_resized, cv::Size(640,640));

        image_msg = cv_bridge::CvImage(header, "rgb8", frame_left).toImageMsg();
        left_pub.publish(image_msg);
        image_msg = cv_bridge::CvImage(header, "rgb8", frame_right).toImageMsg();
        right_pub.publish(image_msg);
        rate.sleep();

    } 
}
