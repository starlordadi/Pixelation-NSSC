cmake_minimum_required(VERSION 2.8.3)
project(pixel)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport 
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  rospy
  tf
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(planner src/planner.cpp)
add_executable(planner_husky src/planner_husky.cpp)
add_executable(cam src/cam.cpp)


target_link_libraries(planner
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(planner_husky
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(cam
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


