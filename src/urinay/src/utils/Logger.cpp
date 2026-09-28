/**
 * @file Logger.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Logger class member functions implementation
 * @version 1.0
 * @date 2025-04-12
 * 
 * @copyright Copyright (c) 2025 TUfast e.V.
 */

#include "utils/Logger.hpp"
#include <sstream>

std::map<std::string, Logger::Task> Logger::tasks_;
std::queue<std::string> Logger::info_msgs_, Logger::warn_msgs_;
bool Logger::print_immediately;

void Logger::tick(const std::string &clockName) {
  auto now = std::chrono::steady_clock::now();
  std::map<std::string, Task>::iterator it = tasks_.find(clockName);
  if (it != tasks_.end()) {
    if (it->second.active) {
      RCLCPP_WARN(rclcpp::get_logger("urinay"), "Called tick() two times with same clockName before calling tock()");
    }
    else {
      it->second.activate(now);
    }
  } else {
    tasks_.emplace(clockName, Task(now));
  }
}

std::chrono::duration<double> Logger::tock(const std::string &clockName) {
  auto now = std::chrono::steady_clock::now();
  std::map<std::string, Task>::iterator it = tasks_.find(clockName);
  if (it == tasks_.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("urinay"), "Called tock() before calling tick()");
    return std::chrono::duration<double>::zero();
  } else {
    std::chrono::duration<double> duration = it->second.stop(now);
    if (print_immediately) RCLCPP_INFO(rclcpp::get_logger("urinay"), "%s has taken: %.3fms", clockName.c_str(), duration.count() * 1e3);
    return duration;
  }
}

void Logger::loginfo(const std::string &msg) {
  if (print_immediately)
    RCLCPP_INFO(rclcpp::get_logger("urinay"), "%s", msg.c_str());
  else
    info_msgs_.push(msg);
}

void Logger::logwarn(const std::string &msg) {
  if (print_immediately)
    RCLCPP_WARN(rclcpp::get_logger("urinay"), "%s", msg.c_str());
  else
    warn_msgs_.push(msg);
}

void Logger::print_report() {
  std::stringstream to_print;
  // Table header
  to_print << "\033[2J\033[H";
  to_print << "+-------------------------------------------------------+\n";
  to_print << "|                      URINAY LOG                       |\n";
  to_print << "+-------------------------------------------------------+\n";
  to_print << std::fixed << std::setprecision(2);
  to_print << "| " << std::setw(21) << std::left << "Task Name"
  << std::setw(11) << "Last (ms)"
  << std::setw(11) << "Max (ms)"
  << std::setw(11) << "Avg (ms)" << "|\n";
  to_print << "+-------------------------------------------------------+\n";
  
  // Print each task time stats
  for (const auto& pair : tasks_) {
    const Task& chrono = pair.second;
    to_print << "| " << std::setw(21) << std::left << pair.first
             << std::setw(11) << chrono.last_duration.count() * 1e3
             << std::setw(11) << chrono.max_duration.count() * 1e3
             << std::setw(11) << (chrono.total_time_active.count() * 1e3) / chrono.count << "|\n";
  }

  to_print << "+-------------------------------------------------------+\n";

  // Print log messages
  if (!info_msgs_.empty() or !warn_msgs_.empty()) {
    to_print << "\033[33m";
    while (!warn_msgs_.empty()) {
      to_print << "| " << std::setw(54) << std::left << "[WARN] " + warn_msgs_.front() << "|\n";
      warn_msgs_.pop();
    }
    to_print << "\033[0m";
    while (!info_msgs_.empty()) {
      to_print << "| " << std::setw(54) << std::left << "[INFO] " + info_msgs_.front() << "|\n";
      info_msgs_.pop();
    }
  } else {
    to_print << "| " << std::setw(54) << std::left << "No logs to show..." << "|\n";
  }

  to_print << "+-------------------------------------------------------+\n";
  
  RCLCPP_INFO(rclcpp::get_logger("urinay"), "%s", to_print.str().c_str());
}