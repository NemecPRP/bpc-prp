#pragma once
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include "sensor_msgs/msg/compressed_image.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <algorithms/aruco_detector.h>



namespace nodes {
    class CameraNode : public rclcpp::Node {
    public:
        // Constructor
        CameraNode();

        // Destructor (default)
        ~CameraNode() override = default;

        algorithms::ArucoDetector aruco_detector;

    private:
        void on_image_msg(sensor_msgs::msg::CompressedImage::SharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_subscriber_;
    };
}