cmake_minimum_required(VERSION 3.0.2)
project(rm_omni)

find_package(catkin REQUIRED COMPONENTS
cv_bridge_new
image_transport
roscpp
sensor_msgs
rm_interfaces
std_msgs
rospy
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# add_subdirectory(src)

add_executable(${PROJECT_NAME}_node src/rm_omni_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
)

