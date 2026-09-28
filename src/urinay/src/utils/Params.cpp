/**
 * @file Param.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Param class member functions implementation
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "utils/Params.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

/* ----------------------------- Private Methods ---------------------------- */

/* ----------------------------- Public Methods ----------------------------- */

Params::Params(rclcpp::Node *const nh) {
  auto param = [nh](const std::string &name, auto value) {
    return nh->declare_parameter(name, value);
  };
  // Main
  main.package_path = ament_index_cpp::get_package_share_directory("urinay");
  main.input_cones_topic = param("input_cones_topic", std::string("/AS/P/ccat/cones"));
  main.input_pose_topic = param("input_pose_topic", std::string("/AS/C/state"));
  main.output_full_topic = param("output_full_topic", std::string("/AS/P/tracklimits/full"));
  main.output_partial_topic = param("output_partial_topic", std::string("/AS/P/tracklimits/partial"));
  main.shutdown_on_loop_closure = param("shutdown_on_loop_closure", true);
  main.min_cone_confidence = static_cast<float>(param("min_cone_confidence", 0.0));
  main.verbose = param("verbose", false);

  // WayComputer
  wayComputer.max_triangle_edge_len = param("max_triangle_edge_len", 9.0);
  wayComputer.min_triangle_angle = param("min_triangle_angle", 0.25);
  wayComputer.failsafe_max_way_horizon_size = param("failsafe_max_way_horizon_size", 6);
  wayComputer.general_failsafe = param("general_failsafe", true);
  wayComputer.general_failsafe_safetyFactor = param("general_failsafe_safetyFactor", 1.4);
  // WayComputer::Search
  wayComputer.search.max_way_horizon_size = param("max_way_horizon_size", 0);
  wayComputer.search.tree_search_max_height = param("tree_search_max_height", 7);
  wayComputer.search.search_radius = param("search_radius", 5.0);
  wayComputer.search.max_angle_diff = param("max_angle_diff", 0.6);
  wayComputer.search.min_track_width = param("min_track_width", 2.8);
  wayComputer.search.max_search_options = param("max_search_options", 2);
  wayComputer.search.overnext_midpoint_angle = param("overnext_midpoint_angle", 0.2);
  wayComputer.search.max_next_heuristic = param("max_next_heuristic", 3.0);
  wayComputer.search.heur_dist_weight = static_cast<float>(param("heur_dist_weight", 0.6));
  wayComputer.search.heur_track_width_diff_weight = static_cast<float>(param("heur_track_width_diff_weight", 0.2));
  wayComputer.search.allow_intersection = param("allow_intersection", false);
  wayComputer.search.min_dist_between_midpoints = param("min_dist_between_midpoints", 0.09);
  // WayComputer::Trace
  wayComputer.trace.max_dist_loop_closure = param("max_dist_loop_closure", 1.0);
  wayComputer.trace.max_angle_diff_loop_closure = param("max_angle_diff_loop_closure", 0.6);
  wayComputer.trace.vital_num_midpoints = param("vital_num_midpoints", 5);

  // Visualization
  visualization.publish_markers = param("publish_markers", false);
  visualization.visualize_treeSearch = param("visualize_treeSearch", false);
  visualization.triangulation_topic = param("marker_topics/triangulation", std::string("/AS/P/urinay/markers/triangulation"));
  visualization.way_topic = param("marker_topics/way", std::string("/AS/P/urinay/markers/way"));
  visualization.treeSearch_topic = param("marker_topics/treeSearch", std::string("/AS/P/urinay/markers/treeSearch"));
}