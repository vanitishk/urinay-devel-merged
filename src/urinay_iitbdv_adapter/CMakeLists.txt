cmake_minimum_required(VERSION 3.8)
project(urinay_iitbdv_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(dv_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(cone_pose_adapter src/cone_pose_adapter.cpp)
ament_target_dependencies(cone_pose_adapter
  dv_msgs
  nav_msgs
  rclcpp
)

install(TARGETS cone_pose_adapter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
