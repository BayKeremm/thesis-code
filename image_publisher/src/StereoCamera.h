#pragma once 
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "GXDevice.h"
namespace vn{
class StereoCamera{
    private:
        GXDevice* m_VisionDevice;  // Reference to vision device    
        cv::Mat m_RawImage; // Left and Right
        cv::Mat m_StitchedImage; 
        cv::Mat m_VideoImage; 
        cv::Mat m_IntrinsicParams[2]; 
        cv::Mat m_DistortionCoeff[2]; // 1st row - x 2nd row - y 
        
    public: 
        StereoCamera(); 
        StereoCamera(GXDevice* m_VisionDevice); 
        StereoCamera(int width, int height); 
        ~StereoCamera();  

        void StartCapture() const; 
        void StopCapture() const;
        void AquireImage() ; 
        void SetImage(const cv::Mat& DualImage) ; 
        void SetImage(const cv::Mat& DualImage, int cv_conversion_code) ;
        cv::Mat GetLeftImage();  // Get Left 
        cv::Mat GetRightImage();  // Get Right 
        cv::Mat& GetRawImage(); 
        cv::Mat& GetVideoImage(); 

}; }; 