cmake_minimum_required(VERSION 3.1.3)
project(bliss_driver)

## 需要生成消息，即自定义数据结构
find_package(catkin REQUIRED COMPONENTS
  message_generation            
  actionlib
  control_msgs
  joint_state_publisher
  moveit_msgs
  robot_state_publisher
  roscpp
  sensor_msgs
  std_msgs
)

## 将msg文件夹中的文件进行转换生成ROS C++头文件
add_message_files(FILES
  ExtraFeatures.msg
)


## 生成ROS C++头文件的消息，将其以来的其他消息结构放置在此处
generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)


# 头文件引入
include_directories(
  include include/bliss_driver
  ${catkin_INCLUDE_DIRS}
)


# ROS
add_executable(main_ros
  include/bliss_driver/common.h
  include/bliss_driver/BlissRobot.h
  include/bliss_driver/BlissRobotRos.h
  src/common.cpp
  src/BlissRobot.cpp
  src/BlissRobotRos.cpp
  src/main_ros.cpp
)
add_dependencies(main_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main_ros
  ${catkin_LIBRARIES} -lpthread
)

