#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <torch/csrc/jit/runtime/graph_executor.h>

#include "Yolo.hpp"
#include "RektNet.hpp"
#include "Handler3D.hpp"
#include "utils.hpp"


// testing libraries
#include <chrono>
#include <string>
#include <fstream>

static std::string yolo_path = "/home/guillaume/thesis-rio-kerem/src/inference_package/networks/yolov7_reparamed.torchscript.pt";
static Net_config YOLOV7_nets = { 0.8, 0.6, yolo_path, 640, 640};
static YOLOV7 yolo(YOLOV7_nets);

static std::string rektnet_path = "/home/guillaume/thesis-rio-kerem/src/inference_package/networks/traced_rektnet_cuda.pt";
static RektNet rektnet{rektnet_path};

static std::string calibration_path{"/home/guillaume/thesis-rio-kerem/src/inference_package/camera_parameters/"};
static Handler3D handler3d(calibration_path+"calibration.txt");

static std::vector<std::string> class_names{"blue", "large orange", "small orange", "unkown", "yellow"};

static int counter = 1;

// std::ofstream outfile_x("outputs/output_x.txt");
// std::ofstream outfile_y("outputs/output_y.txt");
// std::ofstream outfile_z("outputs/output_z.txt");

void drawPred(const cone_t& cone, cv::Mat& frame)   
{
    int left = cone.box.x;
    int right = cone.box.width + cone.box.x;
    int top = cone.box.y;
    int bottom = cone.box.height + cone.box.y;

    // color based on classId or not valid
    cv::Scalar color;
    if(!cone.valid == true) {
        color = cv::Scalar(0, 0, 255); // Red is (0, 0, 255) in OpenCV
    } 
    else if (cone.classId == 0) { // blue
        color = cv::Scalar(255, 255, 255);
    } else if (cone.classId == 4) { // yellow
        color = cv::Scalar(0, 255, 255);
    } else if (cone.classId == 1) { // large orange
        color = cv::Scalar(0, 100, 255);
    } else if (cone.classId == 2) { // small orange
        color = cv::Scalar(0, 200, 255);
    } else { // unkown set black, shouldnt happen!
        color = cv::Scalar(0, 0, 0);
    }

	// draw bounding box
	rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), color, 2);

	// display class and confidence on top
    std::string class_confidence_text = class_names[cone.classId] + " c:" + cv::format("%.2f", cone.confidence);
    putText(frame, class_confidence_text, cv::Point(left, top-10), cv::FONT_HERSHEY_SIMPLEX, 0.30, color, 1);

	// display translation vector at bottom
    std::string translation_text = cv::format("x: %d, y: %d, z: %d ", cone.translation[0], cone.translation[1], cone.translation[2]);
    putText(frame, translation_text, cv::Point(left, bottom+10), cv::FONT_HERSHEY_SIMPLEX, 0.30, color, 1);
    
    // draw keypoints
    for(auto p: cone.keypoints){
		cv::circle(frame,cv::Point2d(p.first+left, p.second+top),2, color, -1);
	}
}

void printCone(const cone_t& cone) {
    std::cout << "\n\tCone detected" << std::endl;
    std::cout << "Box: " << cone.box << std::endl;
    std::cout << "Class: " << class_names[cone.classId] << std::endl;
    std::cout << "Confidence: " << cone.confidence << std::endl;
    std::cout << "Translation: (" << cone.translation[0] << ", " << cone.translation[1] << ", " << cone.translation[2] << ")" << std::endl;
    std::cout << "Valid: " << cone.valid << std::endl;

    // output specific coordinate measurments to one file
    // outfile_x << cone.translation[0] << std::endl;
    // outfile_y << cone.translation[1] << std::endl;
    // outfile_z << cone.translation[2] << std::endl;
}
static int one = 0;
void imageCallback(const sensor_msgs::ImageConstPtr & msg){
    try{
        std::cout << "\n\t\tFrame nr: " << counter << std::endl;
        // convert the msg to cv mat object
        cv::Mat frame = cv_bridge::toCvShare(msg,"rgb8")->image;

        auto t1 = std::chrono::high_resolution_clock::now();
        // yolo inference
        auto cones = yolo.detect(frame);

        auto t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> ms_double = t2 - t1;
        std::cout << ms_double.count() << "yolo in total took \n";
        t1 = std::chrono::high_resolution_clock::now();
        for(cone_t & cone : cones){
            cone.keypoints = rektnet.detect(frame(cone.box), cone);
            cone.translation = handler3d.solvePnP(cone);
        }
        t2 = std::chrono::high_resolution_clock::now();
        ms_double = t2 - t1;
        std::cout << ms_double.count() << "rektnet in total took \n";


        for(cone_t & cone : cones){
            // draw bounding boxes and keypoints
            drawPred(cone, frame);
        }

        // loop through the vector and remove invalid cones
        for(int i = 0; i < cones.size(); ) {
            if(!cones[i].valid) {
                cones.erase(cones.begin() + i); // remove the invalid cone
            } else {
                ++i;
            }
        }
        try {
                //cv::imwrite("outputs/detection" + std::to_string(counter++) + ".jpeg", frame);
        } catch (const cv::Exception& e) {
            std::cerr << "Error writing image: " << e.what() << std::endl;
        }
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
    }
}

int main(int argc, char ** argv){

    // // to write cout to file instead of terminal
    // std::ofstream out("outputs/output.txt"); // Open the file for writing
    // std::streambuf* coutbuf = std::cout.rdbuf(); // Save the original cout buffer
    // std::cout.rdbuf(out.rdbuf()); // Redirect cout to the file
    
    torch::jit::setGraphExecutorOptimize(false);
    ros::init(argc, argv, "vision_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("left/image_raw", 20, imageCallback);
    ros::spin();

    // std::cout.rdbuf(coutbuf); // Restore the original cout buffer
    // outfile_x.close();
    // outfile_y.close();
    // outfile_z.close();
    
    return 0;
}