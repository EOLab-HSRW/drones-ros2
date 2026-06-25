#include <memory>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

using std::placeholders::_1;

class OdomVizNode : public rclcpp::Node
{
public:

  static constexpr int DEFAULT_MAX_PATH_LEN = 2000;

  OdomVizNode()
  : Node("odom_viz_node"),
    max_path_len_(DEFAULT_MAX_PATH_LEN)
  {
    max_path_len_ = this->declare_parameter<int>("max_path_len", max_path_len_);
    odom_topic_   = this->declare_parameter<std::string>("odom_topic", "odom");
    pose_topic_   = this->declare_parameter<std::string>("pose_topic", "utils/pose");
    path_topic_   = this->declare_parameter<std::string>("path_topic", "utils/path");


    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data))
                 .best_effort()
                 .keep_last(10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, qos, std::bind(&OdomVizNode::odom_cb, this, _1));

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, qos);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, qos);

    path_msg_.header.frame_id = "odom"; // will be overwritten from odom.header.frame_id anyway
  }

private:
  int max_path_len_;
  std::string odom_topic_, pose_topic_, path_topic_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  nav_msgs::msg::Path path_msg_;
  std::deque<geometry_msgs::msg::PoseStamped> ring_;

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;      // stamp + frame_id
    pose.pose = msg->pose.pose;

    pose_pub_->publish(pose);

    ring_.push_back(pose);
    while (static_cast<int>(ring_.size()) > max_path_len_) {
      ring_.pop_front();
    }

    path_msg_.header = msg->header;
    path_msg_.poses.assign(ring_.begin(), ring_.end());
    path_pub_->publish(path_msg_);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomVizNode>());
  rclcpp::shutdown();
  return 0;
}

