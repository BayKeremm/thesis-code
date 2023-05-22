cmake_minimum_required(VERSION 3.0.2)
project(inference_package)

set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -gencode arch=compute_75,code=sm_75")

set(CMAKE_BUILD_TYPE Release)
#set(CUDA_LAUNCH_BLOCKING 1)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED COMPONENTS 
    highgui 
    dnn
    core 
    imgproc 
    video 
    calib3d 
    features2d
)
find_package(Torch REQUIRED COMPONENTS script PATHS "/home/guillaume/libtorch" )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES inference_package
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/src/*.cpp")
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/src/*.hxx")

add_executable(vision_node ${SOURCES}  )

target_link_libraries(vision_node ${OpenCV_LIBS} ${catkin_LIBRARIES} ${TORCH_LIBRARIES} )