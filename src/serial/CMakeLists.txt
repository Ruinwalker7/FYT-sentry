cmake_minimum_required(VERSION 3.5)
project(serial)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  rm_interfaces
)

#catkin_python_setup()

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES backend
 CATKIN_DEPENDS roscpp std_msgs 
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  "/usr/include/eigen3"
)

set(SRC
  "src/transporter_driver/uart_transporter.cpp"
  "src/robot_base_node.cpp"
  "src/robot_base_main.cpp"
)


# Create robot_base_example node
add_executable(simple_robot_base ${SRC})
target_link_libraries(simple_robot_base 
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS scripts/virtual_serial.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)