cmake_minimum_required(VERSION 2.8.3)
project(ps_ardrone)

find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  cv_bridge
  image_transport
  OpenCV
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(main src/main.cpp src/ardrone.cpp src/communicationcontroller.cpp src/navdatacontroller.cpp src/cameracontroller.cpp src/objectdetection.cpp src/pugixml.cpp src/imageobject.cpp)
target_link_libraries(main ${catkin_LIBRARIES})

add_executable(hsv src/hsv.cpp src/objectdetection.cpp src/pugixml.cpp src/imageobject.cpp src/cameracontroller.cpp)
target_link_libraries(hsv ${catkin_LIBRARIES})
