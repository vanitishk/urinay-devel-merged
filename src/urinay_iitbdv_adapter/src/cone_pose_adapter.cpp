#include <dv_msgs/msg/indexed_track.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

class ConePoseAdapter final : public rclcpp::Node {
 public:
  ConePoseAdapter() : Node("urinay_iitbdv_adapter") {
    const std::string input_cones_topic =
        declare_parameter<std::string>("input_cones_topic", "/perception/cones");
    const std::string input_pose_topic =
        declare_parameter<std::string>("input_pose_topic", "/ground_truth/odom");
    const std::string output_cones_topic =
        declare_parameter<std::string>("output_cones_topic", "/urinay/cones_cartesian");
    cone_match_distance_ = declare_parameter<double>("cone_match_distance", 0.4);

    if (!std::isfinite(cone_match_distance_) || cone_match_distance_ <= 0.0) {
      throw std::invalid_argument("cone_match_distance must be a finite value greater than zero");
    }

    cones_pub_ = create_publisher<dv_msgs::msg::IndexedTrack>(
        output_cones_topic, rclcpp::QoS(10));

    cones_sub_ = create_subscription<dv_msgs::msg::IndexedTrack>(
        input_cones_topic, rclcpp::QoS(10),
        std::bind(&ConePoseAdapter::conesCallback, this, std::placeholders::_1));

    pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        input_pose_topic, rclcpp::SensorDataQoS(),
        std::bind(&ConePoseAdapter::poseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "Converting cones from '%s' using pose '%s' to '%s'; "
                "cone_match_distance=%.2f m",
                input_cones_topic.c_str(), input_pose_topic.c_str(),
                output_cones_topic.c_str(), cone_match_distance_);
  }

 private:
  struct TrackedCone {
    double x;
    double y;
    uint32_t id;
  };

  struct MatchCandidate {
    double distance_squared;
    size_t current_index;
    size_t previous_index;
  };

  void poseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    const auto &position = msg->pose.pose.position;
    const auto &orientation = msg->pose.pose.orientation;

    const double quaternion_norm_squared =
        orientation.x * orientation.x + orientation.y * orientation.y +
        orientation.z * orientation.z + orientation.w * orientation.w;

    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(quaternion_norm_squared) || quaternion_norm_squared <= 1e-12) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Ignoring odometry with invalid position or orientation");
      return;
    }

    pose_x_ = position.x;
    pose_y_ = position.y;
    pose_yaw_ = std::atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        quaternion_norm_squared -
            2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
    pose_available_ = true;
  }

  void conesCallback(const dv_msgs::msg::IndexedTrack::ConstSharedPtr msg) {
    if (!pose_available_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Waiting for valid odometry before converting cones");
      return;
    }

    dv_msgs::msg::IndexedTrack converted;
    converted.track.reserve(msg->track.size());

    const double cos_yaw = std::cos(pose_yaw_);
    const double sin_yaw = std::sin(pose_yaw_);

    for (const auto &cone : msg->track) {
      const double range = cone.location.x;
      const double angle = cone.location.y;

      if (!std::isfinite(range) || !std::isfinite(angle) || range < 0.0) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Skipping cone with invalid range or angle");
        continue;
      }

      const double local_x = range * std::cos(angle);
      const double local_y = range * std::sin(angle);

      auto world_cone = cone;
      world_cone.location.x = pose_x_ + cos_yaw * local_x - sin_yaw * local_y;
      world_cone.location.y = pose_y_ + sin_yaw * local_x + cos_yaw * local_y;
      world_cone.location.z = 0.0;
      converted.track.push_back(world_cone);
    }

    if (converted.track.empty()) {
      cones_pub_->publish(converted);
      return;
    }

    assignStableIds(converted);
    cones_pub_->publish(converted);
  }

  void assignStableIds(dv_msgs::msg::IndexedTrack &current) {
    std::vector<MatchCandidate> candidates;
    const double max_distance_squared =
        cone_match_distance_ * cone_match_distance_;

    for (size_t current_index = 0; current_index < current.track.size(); ++current_index) {
      const auto &cone = current.track[current_index];

      for (size_t previous_index = 0; previous_index < previous_cones_.size(); ++previous_index) {
        const double dx = cone.location.x - previous_cones_[previous_index].x;
        const double dy = cone.location.y - previous_cones_[previous_index].y;
        const double distance_squared = dx * dx + dy * dy;

        if (distance_squared <= max_distance_squared) {
          candidates.push_back({distance_squared, current_index, previous_index});
        }
      }
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const MatchCandidate &a, const MatchCandidate &b) {
                if (a.distance_squared != b.distance_squared) {
                  return a.distance_squared < b.distance_squared;
                }
                if (a.current_index != b.current_index) {
                  return a.current_index < b.current_index;
                }
                return a.previous_index < b.previous_index;
              });

    std::vector<bool> current_matched(current.track.size(), false);
    std::vector<bool> previous_matched(previous_cones_.size(), false);

    for (const auto &candidate : candidates) {
      if (current_matched[candidate.current_index] ||
          previous_matched[candidate.previous_index]) {
        continue;
      }

      current.track[candidate.current_index].index =
          previous_cones_[candidate.previous_index].id;
      current_matched[candidate.current_index] = true;
      previous_matched[candidate.previous_index] = true;
    }

    for (size_t i = 0; i < current.track.size(); ++i) {
      if (current_matched[i]) {
        continue;
      }

      if (next_id_ > std::numeric_limits<uint32_t>::max()) {
        RCLCPP_ERROR(get_logger(), "Cone ID space exhausted; dropping this cone map");
        return;
      }

      current.track[i].index = static_cast<uint32_t>(next_id_++);
    }

    std::vector<TrackedCone> next_previous;
    next_previous.reserve(current.track.size());
    for (const auto &cone : current.track) {
      next_previous.push_back(
          {cone.location.x, cone.location.y, cone.index});
    }
    previous_cones_ = std::move(next_previous);
  }

  rclcpp::Subscription<dv_msgs::msg::IndexedTrack>::SharedPtr cones_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Publisher<dv_msgs::msg::IndexedTrack>::SharedPtr cones_pub_;

  std::vector<TrackedCone> previous_cones_;
  double cone_match_distance_{0.4};
  double pose_x_{0.0};
  double pose_y_{0.0};
  double pose_yaw_{0.0};
  uint64_t next_id_{1};
  bool pose_available_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ConePoseAdapter>());
  } catch (const std::exception &exception) {
    RCLCPP_FATAL(rclcpp::get_logger("urinay_iitbdv_adapter"),
                 "Failed to start adapter: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
