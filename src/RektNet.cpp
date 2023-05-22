#include "RektNet.hpp"



RektNet::RektNet(std::string filename) {
    try {
        rektnet = torch::jit::load(filename,torch::kCUDA);
        std::cout << "success loading rektnet model\n";
}
    catch (const c10::Error& e) {
        std::cerr << "error loading rektnet model\n";
    }
}

// input frame is the cropped image
std::vector<std::pair<int,int>> RektNet::detect(const cv::Mat& coneFrame, cone_t& cone){
    cv::Mat coneImage;
    cv::resize(coneFrame,coneImage,cv::Size(80,80));
    coneImage.convertTo(coneImage, CV_32FC3, 1.0 / 255.0);

    auto tensor = torch::from_blob(coneImage.data, {coneImage.rows, coneImage.cols, 3 }, torch::kFloat);
    tensor = tensor.permute({ (2),(0),(1) });
    tensor.unsqueeze_(0); //add batch dim
    torch::cuda::synchronize;
    tensor = tensor.to(torch::kCUDA);
    std::vector<torch::jit::IValue> input;
    input.emplace_back(tensor);

    auto ps = rektnet.forward(input).toTuple()->elements()[1].toTensor().to(torch::kCPU,true);
    
    torch::cuda::synchronize;
    // forward
    auto points = ps.select(0,0);

    std::vector<float> output(points.data_ptr<float>(), points.data_ptr<float>() + points.numel()); // 14 elements
    // extract coordinates
    std::vector<std::pair<int,int>> keypoints;
    for(int i=0; i < 14 ; i+=2){
        double x = output[i];
        double y = output[i+1];
        keypoints.emplace_back(std::pair<int,int>(x * cone.box.width, y * cone.box.height));
    }
    std::sort(keypoints.begin(), keypoints.end(), [](const std::pair<int,int> &a, const std::pair<int,int> &b) {return a.first < b.first;});
    return keypoints;
}
/*
std::vector<std::vector<std::pair<int,int>>> RektNet::detectBatched(const cv::Mat& frame, std::vector<cone_t> & cones){

    std::vector<torch::jit::IValue> input;
    for(cone_t & cone: cones){
        // pre-process the image
        cv::Mat coneImage;
        cv::Mat coneFrame = frame(cone.box);
        cv::resize(coneFrame,coneImage,cv::Size(80,80));
        coneImage.convertTo(coneImage, CV_32FC3, 1.0 / 255.0);

        // convert to libtorch input tensor
        auto tensor = torch::from_blob(coneImage.data, {1, coneImage.rows, coneImage.cols, 3 }, torch::kFloat32);
        tensor = tensor.permute({ (0), (3),(1),(2) });
        //tensor.unsqueeze_(0); //add batch dim
        
        tensor = tensor.to(torch::kCUDA);
        
        input.emplace_back(tensor);
    }

    std::vector<std::vector<std::pair<int,int>>> keypoints;
    // forward
    auto output = rektnet.forward(input).toTuple();
    auto points = output->elements()[1].toTensor();
    points = points.to(torch::kCPU);
    std::cout << points.sizes() << "\n";
    // extract coordinates
    /*
        
    for(int i=0; i < 7 ; i++){
        auto cord = points[0][i];
        double x = cord[0].item<double>();
        double y = cord[1].item<double>();
        keypoints.emplace_back(std::pair<int,int>(x * cone.box.width, y * cone.box.height));
    }

    // sort keypoints
    std::sort(keypoints.begin(), keypoints.end(), [](const std::pair<int,int> &a, const std::pair<int,int> &b) {return a.first < b.first;});



    return keypoints;
}
*/



