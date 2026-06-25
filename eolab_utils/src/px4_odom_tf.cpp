#include <memory>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"


using std::placeholders::_1;

class PoseNode : public rclcpp::Node
{
  public:
    PoseNode()
      : Node("px4_odom_tf")
    {

      auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)).
        best_effort().
        keep_last(1);

      odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
          "fmu/out/vehicle_attitude",
          qos,
          std::bind(&PoseNode::attitude_callback, this, _1));

      local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
          "fmu/out/vehicle_local_position",
          qos,
          std::bind(&PoseNode::position_callback, this, _1));
    }

  private:

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;

    bool have_attitude_{false};
    std::array<float, 4> q_frd_to_ned_wxyz_{};

    static tf2::Quaternion quat_flu_to_enu_from_px4(const std::array<float,4>& q_frd_to_ned_wxyz)
    {
      // PX4 attitude msg q is (w,x,y,z); interpret as rotation FRD -> NED.
      tf2::Quaternion q_frd_to_ned(
          q_frd_to_ned_wxyz[1], q_frd_to_ned_wxyz[2], q_frd_to_ned_wxyz[3], q_frd_to_ned_wxyz[0]);

      tf2::Matrix3x3 R_ned_frd(q_frd_to_ned);

      // Fixed basis transform: NED -> ENU (components): [E,N,U] = [y,x,-z]
      const tf2::Matrix3x3 R_enu_ned(
          0, 1, 0,
          1, 0, 0,
          0, 0,-1

          );

      // Fixed basis transform: FLU -> FRD (components): [F,R,D] = [x,-y,-z]
      const tf2::Matrix3x3 R_frd_flu(
          1, 0, 0,
          0,-1, 0,

          0, 0,-1

          );

      // Chain rotations: FLU -> FRD -> NED -> ENU
      const tf2::Matrix3x3 R_enu_flu = R_enu_ned * R_ned_frd * R_frd_flu;

      tf2::Quaternion q_flu_to_enu;
      R_enu_flu.getRotation(q_flu_to_enu);
      q_flu_to_enu.normalize();
      return q_flu_to_enu; // rotation child(base_link FLU) -> parent(odom ENU)
    }


    void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
    {
      q_frd_to_ned_wxyz_ = {msg->q[0], msg->q[1], msg->q[2], msg->q[3]};
      have_attitude_ = true;
    }

    void position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {

      // we wait for attitude to be ready
      if (!have_attitude_) return;

      // PX4 local position is NED: x=N, y=E, z=D

      // Convert to ROS ENU: x=E, y=N, z=U
      const double x_enu =  msg->y;
      const double y_enu =  msg->x;
      const double z_enu = -msg->z;


      // PX4 velocity is also in NED (vx=N, vy=E, vz=D)
      const double vx_enu =  msg->vy;
      const double vy_enu =  msg->vx;
      const double vz_enu = -msg->vz;

      const tf2::Quaternion q_flu_to_enu = quat_flu_to_enu_from_px4(q_frd_to_ned_wxyz_);

      const rclcpp::Time stamp = this->get_clock()->now();

      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header.stamp = stamp;
      odom_msg.header.frame_id = "odom";
      odom_msg.child_frame_id = "base_link";

      odom_msg.pose.pose.position.x = x_enu;
      odom_msg.pose.pose.position.y = y_enu;
      odom_msg.pose.pose.position.z = z_enu;

      odom_msg.pose.pose.orientation.x = q_flu_to_enu.x();
      odom_msg.pose.pose.orientation.y = q_flu_to_enu.y();
      odom_msg.pose.pose.orientation.z = q_flu_to_enu.z();
      odom_msg.pose.pose.orientation.w = q_flu_to_enu.w();

      odom_msg.twist.twist.linear.x = vx_enu;
      odom_msg.twist.twist.linear.y = vy_enu;
      odom_msg.twist.twist.linear.z = vz_enu;

      // NOTE: what about angular acceleration ?
      // should we add it?
      odom_pub_->publish(odom_msg);

      // Publish TF: odom -> base_link
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = stamp;
      t.header.frame_id = "odom";
      t.child_frame_id  = "base_link";

      t.transform.translation.x = x_enu;
      t.transform.translation.y = y_enu;
      t.transform.translation.z = z_enu;

      t.transform.rotation.x = q_flu_to_enu.x();
      t.transform.rotation.y = q_flu_to_enu.y();
      t.transform.rotation.z = q_flu_to_enu.z();
      t.transform.rotation.w = q_flu_to_enu.w();

      tf_broadcaster_->sendTransform(t);
    }

};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseNode>());
  rclcpp::shutdown();

  return 0;
}
