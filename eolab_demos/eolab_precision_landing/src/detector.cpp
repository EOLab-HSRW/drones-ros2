#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class ArucoDetector : public rclcpp::Node
{
public:
    ArucoDetector() : Node("aruco_detector"), camera_info_received_(false)
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image", 10,
            std::bind(&ArucoDetector::image_callback, this, std::placeholders::_1));

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info", 10,
            std::bind(&ArucoDetector::camera_info_callback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("detected_aruco_markers", 10);
        debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("aruco_debug_image", 10);

        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
        marker_length_ = 0.4; // marker size in meters
    }

private:
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (!camera_info_received_)
        {
            camera_matrix_ = cv::Mat(3, 3, CV_64F, (void *)msg->k.data()).clone();
            dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F);
            for (size_t i = 0; i < msg->d.size(); ++i)
                dist_coeffs_.at<double>(i) = msg->d[i];
            camera_info_received_ = true;
        }
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!camera_info_received_)
            return;

        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(frame, dictionary_, corners, ids);

        visualization_msgs::msg::MarkerArray markers_msg;

        if (!ids.empty())
        {
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, marker_length_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

            for (size_t i = 0; i < ids.size(); ++i)
            {
                visualization_msgs::msg::Marker marker;
                marker.header = msg->header;
                marker.ns = "aruco_markers";
                marker.id = ids[i];
                marker.type = visualization_msgs::msg::Marker::CUBE;
                marker.action = visualization_msgs::msg::Marker::ADD;

                marker.pose.position.x = tvecs[i][0];
                marker.pose.position.y = tvecs[i][1];
                marker.pose.position.z = tvecs[i][2];

                cv::Mat R;
                cv::Rodrigues(rvecs[i], R);
                tf2::Matrix3x3 tfR(
                    R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                    R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                    R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
                tf2::Quaternion q;
                tfR.getRotation(q);
                marker.pose.orientation.x = q.x();
                marker.pose.orientation.y = q.y();
                marker.pose.orientation.z = q.z();
                marker.pose.orientation.w = q.w();

                marker.scale.x = marker_length_;
                marker.scale.y = marker_length_;
                marker.scale.z = 0.001; // flat square

                marker.color.a = 0.8;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;

                markers_msg.markers.push_back(marker);

                // Draw detected markers and axes on debug image
                cv::aruco::drawDetectedMarkers(frame, corners, ids);
                cv::aruco::drawAxis(frame, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], marker_length_ * 0.5);
            }
        }

        marker_pub_->publish(markers_msg);

        auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
        debug_image_pub_->publish(*debug_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Mat camera_matrix_, dist_coeffs_;
    bool camera_info_received_;
    double marker_length_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetector>());
    rclcpp::shutdown();
    return 0;
}
