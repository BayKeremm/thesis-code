cmake_minimum_required(VERSION 3.0.2)
project(image_publisher)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    sensor_msgs 
    image_transport 
    std_msgs 
    cv_bridge 
)
find_package(OpenCV REQUIRED COMPONENTS 
    highgui 
    core 
    imgproc 
    video 
    calib3d 
    features2d
)
find_library(GX_CAMERA_LIBRARY gxiapi PATHS {PROJECT_SOURCE_DIR}/vendor/lib)

#list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/vendor/opencv")

catkin_package(
 INCLUDE_DIRS  ${PROJECT_SOURCE_DIR}/vendor/include 
 CATKIN_DEPENDS roscpp

#  DEPENDS system_lib
)

###########
## Build ##
###########
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( 
 include ${catkin_INCLUDE_DIRS}
 ${PROJECT_SOURCE_DIR}/vendor/include
 ${OpenCV_INCLUDE_DIRS}
)


file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/src/*.cpp")
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/src/*.hxx")
message(${SOURCES})
add_executable(image_publisher_node ${SOURCES}  )

target_link_libraries(image_publisher_node ${OpenCV_LIBS} ${GX_CAMERA_LIBRARY} ${catkin_LIBRARIES})

