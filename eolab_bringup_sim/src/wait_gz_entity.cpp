#include <rclcpp/rclcpp.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>


class WaitForGzEntity : public rclcpp::Node
{
public:
  WaitForGzEntity()
  : Node("wait_gz_entity"),

    found_(false)
  {
    entity_name_ = this->declare_parameter<std::string>("entity_name", "");
    world_name_  = this->declare_parameter<std::string>("world_name", "default");
    timeout_s_   = this->declare_parameter<double>("timeout_s", 30.0);

    if (entity_name_.empty())

    {
      RCLCPP_ERROR(get_logger(), "Parameter 'entity_name' is empty. Exiting.");
      valid_ = false;
      return;
    }

    pose_topic_ = "/world/" + world_name_ + "/pose/info";

    RCLCPP_INFO(get_logger(),
                "Waiting for entity '%s' in world '%s' (topic: %s), timeout=%.3fs",
                entity_name_.c_str(), world_name_.c_str(), pose_topic_.c_str(), timeout_s_);

    if (!gz_node_.Subscribe(pose_topic_, &WaitForGzEntity::OnPoseV, this))
    {
      RCLCPP_ERROR(get_logger(), "Failed to subscribe to Gazebo Transport topic: %s", pose_topic_.c_str());
      valid_ = false;

      return;

    }

    valid_ = true;
  }

  int wait()
  {
    if (!valid_)
      return 1;

    using namespace std::chrono_literals;

    const auto start = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::duration<double>(timeout_s_);

    while (rclcpp::ok() && !found_.load())
    {
      auto elapsed = std::chrono::steady_clock::now() - start;
      if (elapsed >= timeout)

      {
        RCLCPP_ERROR(get_logger(),

                     "Timeout waiting for entity '%s' in world '%s'.",
                     entity_name_.c_str(), world_name_.c_str());
        return 2;

      }

      // No ROS subscriptions needed here, but this keeps the node responsive
      rclcpp::spin_some(this->get_node_base_interface());
      std::this_thread::sleep_for(50ms);
    }


    RCLCPP_INFO(get_logger(), "Entity found: '%s'", found_name_.c_str());
    return 0;
  }

private:
  static bool ends_with(const std::string &s, const std::string &suffix)

  {
    if (suffix.size() > s.size())
      return false;

    return std::equal(suffix.rbegin(), suffix.rend(), s.rbegin());
  }

  void OnPoseV(const gz::msgs::Pose_V &msg)
  {
    if (found_.load())
      return;

    // Gazebo can publish scoped names like "model::link", so we accept:
    //   - exact match
    //   - "...::entity_name"
    const std::string scoped_suffix = "::" + entity_name_;

    for (int i = 0; i < msg.pose_size(); ++i)
    {
      const auto &p = msg.pose(i);
      const std::string &n = p.name();

      if (n == entity_name_ || ends_with(n, scoped_suffix))
      {
        found_name_ = n;
        found_.store(true);

        return;
      }
    }
  }


  // Parameters
  std::string entity_name_;
  std::string world_name_;
  double timeout_s_{30.0};

  // Gazebo transport
  gz::transport::Node gz_node_;
  std::string pose_topic_;


  // State
  std::atomic_bool found_;
  std::string found_name_;
  bool valid_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<WaitForGzEntity>();
  int rc = node->wait();

  rclcpp::shutdown();
  return rc;

}
