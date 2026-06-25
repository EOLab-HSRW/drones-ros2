#include <chrono>

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/srv/control_world.hpp"

using namespace std::chrono_literals;

class WaitForGzReady : public rclcpp::Node
{
public:
  WaitForGzReady()
  : Node("wait_for_gz_ready")
  {
    // Parameters
    this->declare_parameter<double>("timeout_s", 30.0);
    this->declare_parameter<std::string>("world_name", "default");

    timeout_s_ = this->get_parameter("timeout_s").as_double();
    world_name_ = this->get_parameter("world_name").as_string();

    service_name_ = "/world/" + world_name_ + "/control";

    RCLCPP_INFO(
      this->get_logger(),
      "Waiting for Gazebo service: %s (timeout=%.1fs)",
      service_name_.c_str(), timeout_s_);

    client_ = this->create_client<ros_gz_interfaces::srv::ControlWorld>(service_name_);

  }

  int run()
  {
    // Wait for service
    auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(timeout_s_));

    bool ok = client_->wait_for_service(timeout);

    if (!ok) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for %s", service_name_.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Timed out waiting for %s", service_name_.c_str());
      }
      return 2;
    }

    RCLCPP_INFO(this->get_logger(), "Gazebo is READY");
    return 0;
  }

private:
  double timeout_s_{30.0};
  std::string world_name_{"empty"};
  std::string service_name_;

  rclcpp::Client<ros_gz_interfaces::srv::ControlWorld>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<WaitForGzReady>();
  int ret = node->run();

  rclcpp::shutdown();
  return ret;
}
