#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gz/transport/Node.hh>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class WaitForGzGui : public rclcpp::Node
{
public:
  WaitForGzGui()
  : Node("wait_for_gz_gui")
  {
    this->declare_parameter<double>("timeout_s", 30.0);
    this->declare_parameter<std::string>(
      "gui_topic", "/gui/camera/pose");

    timeout_s_ = this->get_parameter("timeout_s").as_double();
    gui_topic_ = this->get_parameter("gui_topic").as_string();

    RCLCPP_INFO(
      this->get_logger(),
      "Waiting for Gazebo GUI topic: %s (timeout=%.1fs)",
      gui_topic_.c_str(),
      timeout_s_);
  }

  int run()
  {
    if (timeout_s_ <= 0.0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "timeout_s must be greater than zero");
      return 2;
    }

    const auto timeout =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(timeout_s_));

    const auto deadline = std::chrono::steady_clock::now() + timeout;

    while (rclcpp::ok() &&
      std::chrono::steady_clock::now() < deadline)
    {
      std::vector<std::string> topics;
      gz_node_.TopicList(topics);

      const bool gui_topic_found =
        std::find(topics.begin(), topics.end(), gui_topic_) != topics.end();

      if (gui_topic_found) {
        RCLCPP_INFO(
          this->get_logger(),
          "Gazebo GUI is READY: %s is advertised",
          gui_topic_.c_str());

        return 0;
      }

      std::this_thread::sleep_for(100ms);
    }

    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Interrupted while waiting for Gazebo GUI");
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Timed out waiting for Gazebo GUI topic: %s",
        gui_topic_.c_str());
    }

    return 2;
  }

private:
  double timeout_s_{30.0};
  std::string gui_topic_{"/gui/camera/pose"};

  gz::transport::Node gz_node_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<WaitForGzGui>();
  const int ret = node->run();

  rclcpp::shutdown();
  return ret;
}
