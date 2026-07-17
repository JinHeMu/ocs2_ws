// =============================================================================
// csv_path_visualizer_node.cpp
//
// Read x, y, yaw from a CSV file and publish nav_msgs::msg::Path for RViz2.
//
// Supported CSV headers:
//   time,x,y,yaw,q1,q2,q3,q4,q5,q6
//   x,y,theta,...
//
// The order of columns can be arbitrary. Only x/y are required; yaw is optional.
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

class CsvPathVisualizerNode final : public rclcpp::Node {
 public:
  CsvPathVisualizerNode() : Node("csv_path_visualizer_node") {
    declare_parameter<std::string>("csv_file", "");
    declare_parameter<std::string>("frame_id", "odom");
    declare_parameter<std::string>("path_topic", "/csv_path");
    declare_parameter<double>("z_height", 0.0);
    declare_parameter<double>("publish_period", 1.0);

    csv_file_ = get_parameter("csv_file").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    path_topic_ = get_parameter("path_topic").as_string();
    z_height_ = get_parameter("z_height").as_double();
    publish_period_ = get_parameter("publish_period").as_double();

    if (csv_file_.empty()) {
      throw std::runtime_error(
          "Parameter 'csv_file' is empty. Please provide the CSV file path.");
    }

    if (publish_period_ <= 0.0) {
      RCLCPP_WARN(get_logger(),
                  "publish_period must be positive. Falling back to 1.0 s.");
      publish_period_ = 1.0;
    }

    // Transient-local QoS allows RViz2 started later to receive the latest path.
    const auto qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    path_publisher_ =
        create_publisher<nav_msgs::msg::Path>(path_topic_, qos);

    if (!loadCsv(csv_file_)) {
      throw std::runtime_error("Failed to load CSV path: " + csv_file_);
    }

    publishPath();

    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(publish_period_)),
        std::bind(&CsvPathVisualizerNode::publishPath, this));

    RCLCPP_INFO(get_logger(),
                "Loaded %zu path points from '%s'. Publishing on '%s' "
                "with frame_id='%s'.",
                path_points_.size(), csv_file_.c_str(), path_topic_.c_str(),
                frame_id_.c_str());
  }

 private:
  struct PathPoint {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  static std::string trim(std::string text) {
    const auto not_space = [](unsigned char c) {
      return !std::isspace(c);
    };

    text.erase(text.begin(),
               std::find_if(text.begin(), text.end(), not_space));
    text.erase(std::find_if(text.rbegin(), text.rend(), not_space).base(),
               text.end());

    // Remove UTF-8 BOM if present.
    if (text.size() >= 3 &&
        static_cast<unsigned char>(text[0]) == 0xEF &&
        static_cast<unsigned char>(text[1]) == 0xBB &&
        static_cast<unsigned char>(text[2]) == 0xBF) {
      text.erase(0, 3);
    }

    return text;
  }

  static std::string normalizeHeader(std::string text) {
    text = trim(std::move(text));
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) {
                     return static_cast<char>(std::tolower(c));
                   });
    return text;
  }

  static std::vector<std::string> splitCsvLine(const std::string& line) {
    std::vector<std::string> cells;
    std::stringstream stream(line);
    std::string cell;

    while (std::getline(stream, cell, ',')) {
      cells.push_back(trim(cell));
    }

    // Preserve an empty final cell after a trailing comma.
    if (!line.empty() && line.back() == ',') {
      cells.emplace_back();
    }

    return cells;
  }

  static bool parseDouble(const std::string& text, double& value) {
    try {
      std::size_t parsed = 0;
      value = std::stod(text, &parsed);
      return parsed == text.size() && std::isfinite(value);
    } catch (const std::exception&) {
      return false;
    }
  }

  bool loadCsv(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Cannot open CSV file: %s",
                   file_path.c_str());
      return false;
    }

    std::string line;
    std::vector<std::string> header;

    // Find the first non-empty, non-comment line as the header.
    while (std::getline(file, line)) {
      line = trim(line);
      if (line.empty() || line.front() == '#') {
        continue;
      }
      header = splitCsvLine(line);
      break;
    }

    if (header.empty()) {
      RCLCPP_ERROR(get_logger(), "CSV file has no valid header.");
      return false;
    }

    std::unordered_map<std::string, std::size_t> column_index;
    for (std::size_t i = 0; i < header.size(); ++i) {
      column_index[normalizeHeader(header[i])] = i;
    }

    const auto findColumn =
        [&column_index](const std::vector<std::string>& candidates)
        -> int {
      for (const auto& name : candidates) {
        const auto it = column_index.find(name);
        if (it != column_index.end()) {
          return static_cast<int>(it->second);
        }
      }
      return -1;
    };

    const int x_index = findColumn({"x", "pos_x", "position_x"});
    const int y_index = findColumn({"y", "pos_y", "position_y"});
    const int yaw_index = findColumn({"yaw", "theta", "heading"});

    if (x_index < 0 || y_index < 0) {
      RCLCPP_ERROR(
          get_logger(),
          "CSV header must contain x and y columns. "
          "Optional yaw aliases: yaw/theta/heading.");
      return false;
    }

    path_points_.clear();
    std::size_t line_number = 1;

    while (std::getline(file, line)) {
      ++line_number;
      line = trim(line);

      if (line.empty() || line.front() == '#') {
        continue;
      }

      const auto cells = splitCsvLine(line);
      const std::size_t required_size = static_cast<std::size_t>(
          std::max({x_index, y_index, yaw_index}) + 1);

      if (cells.size() < required_size) {
        RCLCPP_WARN(get_logger(),
                    "Skip CSV line %zu: insufficient columns.",
                    line_number);
        continue;
      }

      PathPoint point;
      if (!parseDouble(cells[static_cast<std::size_t>(x_index)], point.x) ||
          !parseDouble(cells[static_cast<std::size_t>(y_index)], point.y)) {
        RCLCPP_WARN(get_logger(),
                    "Skip CSV line %zu: invalid x or y value.",
                    line_number);
        continue;
      }

      if (yaw_index >= 0) {
        if (!parseDouble(cells[static_cast<std::size_t>(yaw_index)],
                         point.yaw)) {
          RCLCPP_WARN(get_logger(),
                      "CSV line %zu has invalid yaw; use 0 instead.",
                      line_number);
          point.yaw = 0.0;
        }
      }

      path_points_.push_back(point);
    }

    if (path_points_.empty()) {
      RCLCPP_ERROR(get_logger(), "No valid path points found in CSV.");
      return false;
    }

    return true;
  }

  void publishPath() {
    nav_msgs::msg::Path path_message;
    path_message.header.stamp = now();
    path_message.header.frame_id = frame_id_;
    path_message.poses.reserve(path_points_.size());

    for (const auto& point : path_points_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_message.header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.position.z = z_height_;

      tf2::Quaternion quaternion;
      quaternion.setRPY(0.0, 0.0, point.yaw);

      pose.pose.orientation.x = quaternion.x();
      pose.pose.orientation.y = quaternion.y();
      pose.pose.orientation.z = quaternion.z();
      pose.pose.orientation.w = quaternion.w();

      path_message.poses.push_back(std::move(pose));
    }

    path_publisher_->publish(path_message);
  }

  std::string csv_file_;
  std::string frame_id_;
  std::string path_topic_;
  double z_height_{0.0};
  double publish_period_{1.0};

  std::vector<PathPoint> path_points_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<CsvPathVisualizerNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("csv_path_visualizer_node"),
                 "%s", error.what());
  }

  rclcpp::shutdown();
  return 0;
}
