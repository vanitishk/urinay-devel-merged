cmake_minimum_required(VERSION 3.8)
project(urinay)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ofast")
set(CMAKE_BUILD_TYPE "Release")


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(dv_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_exec src/main.cpp)
add_library(${PROJECT_NAME}
  src/structures/Point.cpp
  src/structures/Node.cpp
  src/structures/Edge.cpp
  src/structures/Circle.cpp
  src/structures/Triangle.cpp
  src/structures/Trace.cpp
  src/structures/TraceBuffer.cpp
  src/structures/Vector.cpp
  src/modules/DelaunayTri.cpp
  src/modules/WayComputer.cpp
  src/modules/Visualization.cpp
  src/utils/Logger.cpp
  src/utils/Params.cpp
  src/utils/Failsafe.cpp
  src/utils/KDTree.cpp
)

ament_target_dependencies(${PROJECT_NAME}_exec  rclcpp std_msgs geometry_msgs nav_msgs visualization_msgs dv_msgs tf2 tf2_eigen ament_index_cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs geometry_msgs nav_msgs visualization_msgs dv_msgs tf2 tf2_eigen ament_index_cpp)


target_link_libraries(${PROJECT_NAME}_exec ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(TARGETS ${PROJECT_NAME}_exec DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()