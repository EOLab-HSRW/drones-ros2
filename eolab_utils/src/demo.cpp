#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

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

class PX4Visualizer : public rclcpp::Node
{
public:
    PX4Visualizer() : Node("px4_visualizer")
    {
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data))
                       .reliable()
                       .keep_last(1);

        attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos, std::bind(&PX4Visualizer::attitudeCallback, this, _1));
        local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos, std::bind(&PX4Visualizer::positionCallback, this, _1));
        setpoint_sub_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", qos, std::bind(&PX4Visualizer::setpointCallback, this, _1));

        vehicle_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/px4_visualizer/vehicle_pose", 10);
        vehicle_vel_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/px4_visualizer/vehicle_velocity", 10);
        vehicle_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/px4_visualizer/vehicle_path", 10);
        setpoint_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/px4_visualizer/setpoint_path", 10);

        vehicle_attitude_ = {1.0, 0.0, 0.0, 0.0};
        vehicle_position_ = {0.0, 0.0, 0.0};
        vehicle_velocity_ = {0.0, 0.0, 0.0};
        setpoint_position_ = {0.0, 0.0, 0.0};

        timer_ = this->create_wall_timer(50ms, std::bind(&PX4Visualizer::cmdloopCallback, this));
    }

private:
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vehicle_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vehicle_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr setpoint_path_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::array<float, 4> vehicle_attitude_;
    std::array<float, 3> vehicle_position_;
    std::array<float, 3> vehicle_velocity_;
    std::array<float, 3> setpoint_position_;

    nav_msgs::msg::Path vehicle_path_msg_;
    nav_msgs::msg::Path setpoint_path_msg_;

    void attitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
    {
        vehicle_attitude_[0] = msg->q[0];
        vehicle_attitude_[1] = msg->q[1];
        vehicle_attitude_[2] = -msg->q[2];
        vehicle_attitude_[3] = -msg->q[3];
    }

    void positionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        vehicle_position_[0] = msg->x;
        vehicle_position_[1] = -msg->y;
        vehicle_position_[2] = -msg->z;

        vehicle_velocity_[0] = msg->vx;
        vehicle_velocity_[1] = -msg->vy;
        vehicle_velocity_[2] = -msg->vz;
    }

    void setpointCallback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg)
    {
        setpoint_position_[0] = msg->position[0];
        setpoint_position_[1] = -msg->position[1];
        setpoint_position_[2] = -msg->position[2];
    }

    visualization_msgs::msg::Marker createArrowMarker(int id, const std::array<float, 3> &tail, const std::array<float, 3> &vector)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.ns = "arrow";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.1;
        marker.scale.y = 0.2;
        marker.scale.z = 0.0;
        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        geometry_msgs::msg::Point tail_point, head_point;
        tail_point.x = tail[0];
        tail_point.y = tail[1];
        tail_point.z = tail[2];

        float dt = 0.3;
        head_point.x = tail[0] + dt * vector[0];
        head_point.y = tail[1] + dt * vector[1];
        head_point.z = tail[2] + dt * vector[2];

        marker.points.push_back(tail_point);
        marker.points.push_back(head_point);
        return marker;
    }

    void cmdloopCallback()
    {
        auto now = this->get_clock()->now();
        auto vehicle_pose_msg = vectorToPoseMsg("map", vehicle_position_, vehicle_attitude_);
        vehicle_pose_msg.header.stamp = now;

        vehicle_pose_pub_->publish(vehicle_pose_msg);

        vehicle_path_msg_.header = vehicle_pose_msg.header;
        vehicle_path_msg_.poses.push_back(vehicle_pose_msg);
        vehicle_path_pub_->publish(vehicle_path_msg_);

        auto setpoint_pose_msg = vectorToPoseMsg("map", setpoint_position_, vehicle_attitude_);
        setpoint_pose_msg.header.stamp = now;

        setpoint_path_msg_.header = setpoint_pose_msg.header;
        setpoint_path_msg_.poses.push_back(setpoint_pose_msg);
        setpoint_path_pub_->publish(setpoint_path_msg_);

        auto velocity_marker = createArrowMarker(1, vehicle_position_, vehicle_velocity_);
        velocity_marker.header.stamp = now;
        vehicle_vel_pub_->publish(velocity_marker);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PX4Visualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

