/**
 * @file main.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Main file of Urinay, creates all modules, subcribers and publishers.
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include <dv_msgs/msg/indexed_track.hpp>
#include <std_msgs/msg/header.hpp>
#include <dv_msgs/msg/point_array.hpp>


#include <rclcpp/rclcpp.hpp>
#include <sys/stat.h>

#include <iostream>
#include <functional>

#include "modules/DelaunayTri.hpp"
#include "modules/Visualization.hpp"
#include "modules/WayComputer.hpp"
#include "utils/Logger.hpp"

// Publishers are initialized here


rclcpp::Publisher<dv_msgs::msg::PointArray>::SharedPtr pubPartial;
rclcpp::Publisher<dv_msgs::msg::PointArray>::SharedPtr pubFull;


rclcpp::Node::SharedPtr node;

WayComputer *wayComputer;
Params *params;

// This is the map callback
void mapCallback(const dv_msgs::msg::IndexedTrack::SharedPtr data){
  if (not wayComputer->isLocalTfValid()) {
    Logger::logwarn("CarState not being received.");
    if (params->main.verbose)
      Logger::print_report();
    return;
  }
 if (data->track.empty()) {
    Logger::logwarn("Reading empty set of cones.");
    if (params->main.verbose)
      Logger::print_report();
    return;
  }

  Logger::tick("total");

  // Convert to Node vector
  std::vector<Node> nodes;
  
  
  nodes.reserve(data->track.size());
  for (const dv_msgs::msg::IndexedCone &c : data->track) {
  nodes.emplace_back(c.location.x, c.location.y,
                     c.location.x, c.location.y, c.index);
  }
  
  
  // Update local coordinates of Nodes (makes original local coords unnecessary)
  for (const Node &n : nodes) {
    n.updateLocal(wayComputer->getLocalTf());
  }

  // Delaunay triangulation
  TriangleSet triangles = DelaunayTri::compute(nodes);


  // Update the way with the new triangulation
  std_msgs::msg::Header header;
  header.stamp = node->now();
  header.frame_id = "map";
  wayComputer->update(triangles, header);
  

  // Publish loop and write tracklimits to a file
  if (wayComputer->isLoopClosed()) {
    pubFull->publish(wayComputer->getPathMessage());
    Logger::loginfo("Loop closed!");
    std::string loopDir = params->main.package_path + "/loops";
    mkdir(loopDir.c_str(), 0777);
    wayComputer->writeWayToFile(loopDir + "/loop.unay");
    if (params->main.shutdown_on_loop_closure) {
      Logger::tock("total");
      Logger::loginfo("Have a good day :)");
      if (params->main.verbose)
        Logger::print_report();
      rclcpp::shutdown();
    }
  }
  // Publish partial
  else {
    pubPartial->publish(wayComputer->getPathMessage());
  }

  Logger::tock("total");
  if (params->main.verbose)
    Logger::print_report();
}

// Main
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>("urinay");

  params = new Params(node.get());
  wayComputer = new WayComputer(params->wayComputer);
  Logger::print_immediately = false;
  Visualization::getInstance().init(node.get(), params->visualization);

  // Subscribers & Publishers
  auto subCones = node->create_subscription<dv_msgs::msg::IndexedTrack>(
      params->main.input_cones_topic, 1, mapCallback);
  auto subPose = node->create_subscription<nav_msgs::msg::Odometry>(
      params->main.input_pose_topic, 1,
      std::bind(&WayComputer::stateCallback, wayComputer, std::placeholders::_1));

  
  pubPartial = node->create_publisher<dv_msgs::msg::PointArray>(params->main.output_partial_topic, 1);
  
  
  rclcpp::QoS full_qos(1);
  full_qos.transient_local();
  

  pubFull = node->create_publisher<dv_msgs::msg::PointArray>(params->main.output_full_topic, full_qos);
  
  
  rclcpp::spin(node);
  rclcpp::shutdown();
}
