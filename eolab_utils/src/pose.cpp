#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

using std::placeholders::_1;

class PoseNode : public rclcpp::Node
{
public:
    PoseNode()
    : Node("pose_node")
    {

        auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)).
            best_effort().
            keep_last(1);

        attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "fmu/out/vehicle_attitude",
            qos,
            std::bind(&PoseNode::attitude_callback, this, _1));

        local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "fmu/out/vehicle_local_position",
            qos,
            std::bind(&PoseNode::position_callback, this, _1));

        vehicle_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("utils/pose", qos);
        vehicle_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("utils/path", qos);

    }

private:
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vehicle_path_pub_;

    std::array<float, 4> vehicle_attitude_;
    std::array<float, 3> vehicle_position_;
    std::array<float, 3> vehicle_velocity_;

    nav_msgs::msg::Path vehicle_path_msg_;

    geometry_msgs::msg::PoseStamped vectorToPoseMsg(const std::string &frame_id, const std::array<float, 3> &position, const std::array<float, 4> &attitude)
    {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.frame_id = frame_id;
        pose_msg.pose.orientation.w = attitude[0];
        pose_msg.pose.orientation.x = attitude[1];
        pose_msg.pose.orientation.y = attitude[2];
        pose_msg.pose.orientation.z = attitude[3];
        pose_msg.pose.position.x = position[0];
        pose_msg.pose.position.y = position[1];
        pose_msg.pose.position.z = position[2];
        return pose_msg;
    }

    void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
    {
        vehicle_attitude_[0] = msg->q[0];
        vehicle_attitude_[1] = msg->q[1];
        vehicle_attitude_[2] = -msg->q[2];
        vehicle_attitude_[3] = -msg->q[3];
    }

    void position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        vehicle_position_[0] = msg->x;
        vehicle_position_[1] = -msg->y;
        vehicle_position_[2] = -msg->z;

        vehicle_velocity_[0] = msg->vx;
        vehicle_velocity_[1] = -msg->vy;
        vehicle_velocity_[2] = -msg->vz;

        auto now = this->get_clock()->now();
        auto vehicle_pose_msg = vectorToPoseMsg("map", vehicle_position_, vehicle_attitude_);
        vehicle_pose_msg.header.stamp = now;
        vehicle_pose_pub_->publish(vehicle_pose_msg);

        vehicle_path_msg_.header = vehicle_pose_msg.header;
        vehicle_path_msg_.poses.push_back(vehicle_pose_msg);
        vehicle_path_pub_->publish(vehicle_path_msg_);
    }

};

int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseNode>());
    rclcpp::shutdown();

    return 0;
}
