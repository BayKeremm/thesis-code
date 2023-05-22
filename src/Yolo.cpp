#include "Yolo.hpp"
#include <omp.h>

YOLOV7::YOLOV7(const Net_config config) {

	this->confThreshold = config.confThreshold;
	this->nmsThreshold = config.nmsThreshold;
    
    try {
        yolo = torch::jit::load(config.modelpath,torch::kCUDA);
        std::cout << "success loading yolo model\n";
    }
    catch (const c10::Error& e) {
        std::cerr << "error loading yolo model\n";
    }
	
	this->inpHeight = config.inpHeight;
	this->inpWidth = config.inpWidth;
}

std::vector<cone_t> YOLOV7::detect(const cv::Mat& frame)
{
	// save size relations of initial frame
    float ratioh = (float) frame.rows / this->inpHeight;
    float ratiow = (float) frame.cols / this->inpWidth;

	// preprocess image
    cv::Mat image;
    cv::resize(frame,image,cv::Size(this->inpHeight,this->inpWidth)); // resize
    cv::cvtColor(image,image, cv::COLOR_RGB2BGR ); //opencv uses bgr
    image.convertTo(image, CV_32FC3, 1.0 / 255.0); // normalize

    // define as libtorch input tensor
    auto tensor = torch::from_blob(image.data, { image.rows, image.cols, 3 }, torch::kFloat);
    tensor = tensor.permute({ (2),(0),(1) });
    tensor.unsqueeze_(0); // add batch dim (an inplace operation just like in pytorch)
    torch::cuda::synchronize;
    tensor = tensor.to(torch::kCUDA);
    std::vector<torch::jit::IValue> input;
    input.emplace_back(tensor);

    auto ps = yolo.forward(input).toTuple()->elements()[0].toTensor().select(0,0).to(torch::kCPU,torch::kFloat,true);

    torch::cuda::synchronize;
    std::vector<float> output(ps.data_ptr<float>(), ps.data_ptr<float>() + ps.numel());

	std::vector<float> confidences;
	std::vector<cv::Rect> boxes;
	std::vector<int> classIds;
    for(int i = 0; i < 25200; i++){
        std::vector<float> pred(output.begin()+10*i, output.begin()+10*(i+1));
        float box_score = pred[4]; 
        if(box_score > this->confThreshold){// larger than confidence threshold
            auto maxIter = std::max_element(pred.begin()+5,pred.end());
            float max = *maxIter;
            int max_index = std::distance(pred.begin()+5,maxIter);
            if(max > this->confThreshold) { // conf threshold
                float w = pred[2]*ratiow; 
                float h = pred[3]*ratioh;
                int left = int(pred[0]* ratiow - 0.5*w);
                int top = int(pred[1]* ratioh - 0.5*h);
                    confidences.push_back(max);
                    boxes.emplace_back(left, top, (int)(w), (int)(h));
                    classIds.push_back(max_index);
            } 
        }
    }

	std::vector<int> indices;	
    std::vector<cone_t> cones;
	cv::dnn::NMSBoxes(boxes, confidences, this->confThreshold, this->nmsThreshold, indices);
    for (size_t i = 0; i < indices.size(); ++i)
	{
        // This is neccesarry because the detection goes out of the frame and you get seg fault
        int idx = indices[i];
        if(boxes[idx].x >= 0 && boxes[idx].y >= 0 &&
            boxes[idx].x + boxes[idx].width <= 1200 &&
            boxes[idx].y + boxes[idx].height <= 1000) {
            if(boxes[idx].width < boxes[idx].height) {
		        cones.push_back({boxes[idx],classIds[idx],confidences[idx]});
            }
        }
		
	}
    // filter 8 largest boxes not fallen over
    std::sort(cones.begin(), cones.end(), [](const cone_t& c1, const cone_t& c2) {
        return c1.box.height > c2.box.height;
    });

    std::vector<cone_t> filtered_cones; 
    int numBoxesAdded = 0;
    for (const auto& cone : cones) {
        if (cone.box.width <= cone.box.height && numBoxesAdded < 8) {
            filtered_cones.push_back(cone);
            numBoxesAdded++;
        }
    }

	return filtered_cones;
}
