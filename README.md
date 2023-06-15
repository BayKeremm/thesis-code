
Summary\
This work presents a pipelined approach for a camera detection system to detect cones in frames
and recover their 3D position. It employs an object detection model, a keypoint regression model,
and the Perspective-n-Point algorithm. 

The system achieves sub-80ms latency and <0.5m errors at 10m.

Requirements:
- Ubuntu 20.04 LTS
- CUDA 12.1
- OpenCV 4.7
- Libtorch CUDA 11.7 cxx11 ABI
- ROS Noetic
- Build tool: Catkin

How to run this code:
1. Initialize ros using:

	roscore

2. Build the packages: in a separate terminal navigate to the root of your catkin workspace (~/my_catkin_ws/) and build the package using:

	catkin build image_acquisition_package image_processing_package

3. Source the workspace: after successfully building your package, source the workspace to make the package visible to ROS

	source ~/my_catkin_ws/devel/setup.bash

4. Run the exectuable/node defined in CMakeLists.txt:

	rosrun image_processing_package vision_node

5. Play the rosbag: in a separate tab, go to the directory containing rosbags to test and run:

	rosbag play your_bag_file.bag

6. See the output in terminal were packages are running. Alternatively, if output is set to write to file, wait for completition of rosbag and check out folder with outputs.


Note: The trained neural networks are not included in this repository.
