#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/sensor_combined.hpp"
#include "sensor_msgs/msg/imu.hpp"

class Px4SensorCombinedToImu : public rclcpp::Node {
public:

  Px4SensorCombinedToImu() : Node("px4_sensor_combined_to_imu") {

    topic_input_ = this->declare_parameter<std::string>("topic_input", "fmu/out/sensor_combined");
    topic_output_ = this->declare_parameter<std::string>("topic_output", "imu/data");

    sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
      topic_input_,
      rclcpp::SensorDataQoS(),
      std::bind(&Px4SensorCombinedToImu::sensor_combined_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      topic_output_,
      rclcpp::SensorDataQoS());

  }

private:
  void sensor_combined_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg) {
    sensor_msgs::msg::Imu imu_msg;

    // Note: We are using base_link for frame_id
    // mainly due to match XACRO defintion and PX4 reference
    // frame for sensor_combined data.
    imu_msg.header.frame_id = "base_link";
    imu_msg.angular_velocity.x =  msg->gyro_rad[0];
    imu_msg.angular_velocity.y = -msg->gyro_rad[1];
    imu_msg.angular_velocity.z = -msg->gyro_rad[2];

    // Accel: [x, y, z]_flu = [ x, -y, -z ]_frd
    imu_msg.linear_acceleration.x =  msg->accelerometer_m_s2[0];
    imu_msg.linear_acceleration.y = -msg->accelerometer_m_s2[1];
    imu_msg.linear_acceleration.z = -msg->accelerometer_m_s2[2];

    imu_msg.orientation_covariance[0] = -1.0;

    pub_->publish(imu_msg);
  }

  std::string topic_input_;
  std::string topic_output_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sub_;

};

int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4SensorCombinedToImu>());
  rclcpp::shutdown();

  return 0;
}
