#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/srv/control_world.hpp"

using namespace std::chrono_literals;

class WaitForGazebo : public rclcpp::Node
{
public:
  WaitForGazebo()
  : Node("wait_gz")
  {
    mode_ = declare_parameter<std::string>("mode", "world_service");
    timeout_s_ = declare_parameter<double>("timeout_s", 20.0);
    world_name_ = declare_parameter<std::string>("world_name", "empty");
    entity_name_ = declare_parameter<std::string>("entity_name", "");
    topic_name_ = declare_parameter<std::string>("topic_name", "");
  }

  int run()
  {
    if (timeout_s_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "timeout_s must be greater than zero");
      return 1;
    }

    if (mode_ == "world_service") {
      return wait_for_world_service();
    }

    if (mode_ == "entity") {
      return wait_for_entity();
    }

    if (mode_ == "topic") {
      return wait_for_topic();
    }

    RCLCPP_ERROR(
      get_logger(),
      "Unsupported mode '%s'; expected world_service, entity, or topic",
      mode_.c_str());
    return 1;
  }

private:
  template<typename PredicateT>
  int wait_until(PredicateT predicate, const std::string & description)
  {
    const auto timeout =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(timeout_s_));
    const auto deadline = std::chrono::steady_clock::now() + timeout;

    RCLCPP_INFO(
      get_logger(),
      "Waiting for %s (timeout=%.1fs)",
      description.c_str(),
      timeout_s_);

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (predicate()) {
        RCLCPP_INFO(get_logger(), "Ready: %s", description.c_str());
        return 0;
      }
      std::this_thread::sleep_for(50ms);
    }

    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        get_logger(),
        "Interrupted while waiting for %s",
        description.c_str());
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "Timed out waiting for %s",
        description.c_str());
    }
    return 2;
  }

  int wait_for_world_service()
  {
    if (world_name_.empty()) {
      RCLCPP_ERROR(get_logger(), "world_name must not be empty");
      return 1;
    }

    const std::string service_name =
      "/world/" + world_name_ + "/control";
    world_client_ =
      create_client<ros_gz_interfaces::srv::ControlWorld>(service_name);

    return wait_until(
      [this]() {return world_client_->service_is_ready();},
      "Gazebo world service " + service_name);
  }

  int wait_for_entity()
  {
    if (world_name_.empty() || entity_name_.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "world_name and entity_name must not be empty in entity mode");
      return 1;
    }

    const std::string pose_topic =
      "/world/" + world_name_ + "/pose/info";
    if (!gz_node_.Subscribe(
        pose_topic,
        &WaitForGazebo::on_pose_vector,
        this))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to subscribe to Gazebo topic '%s'",
        pose_topic.c_str());
      return 1;
    }

    return wait_until(
      [this]() {return entity_found_.load();},
      "Gazebo entity " + entity_name_);
  }

  int wait_for_topic()
  {
    if (topic_name_.empty()) {
      RCLCPP_ERROR(get_logger(), "topic_name must not be empty in topic mode");
      return 1;
    }

    return wait_until(
      [this]() {
        std::vector<std::string> topics;
        gz_node_.TopicList(topics);
        return std::find(
          topics.begin(), topics.end(), topic_name_) != topics.end();
      },
      "Gazebo topic " + topic_name_);
  }

  static bool ends_with(
    const std::string & value,
    const std::string & suffix)
  {
    return suffix.size() <= value.size() &&
           std::equal(suffix.rbegin(), suffix.rend(), value.rbegin());
  }

  void on_pose_vector(const gz::msgs::Pose_V & message)
  {
    if (entity_found_.load()) {
      return;
    }

    const std::string scoped_suffix = "::" + entity_name_;
    for (int index = 0; index < message.pose_size(); ++index) {
      const std::string & name = message.pose(index).name();
      if (name == entity_name_ || ends_with(name, scoped_suffix)) {
        entity_found_.store(true);
        return;
      }
    }
  }

  std::string mode_;
  double timeout_s_{20.0};
  std::string world_name_;
  std::string entity_name_;
  std::string topic_name_;

  gz::transport::Node gz_node_;
  std::atomic_bool entity_found_{false};
  rclcpp::Client<ros_gz_interfaces::srv::ControlWorld>::SharedPtr world_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<WaitForGazebo>();
  const int result = node->run();
  rclcpp::shutdown();
  return result;
}
