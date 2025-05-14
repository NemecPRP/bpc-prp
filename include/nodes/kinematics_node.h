#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>

namespace nodes {
    class KinematicsNode : public rclcpp::Node {
    public:
        // Constructor
        KinematicsNode();
        // Destructor (default)
        ~KinematicsNode() override = default;

        float L = 0.13;
        float R = 0.033;
        float v = 0;
        float w = 0;


    private:
        // Publisher, timer
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr kinematics_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        // Subscriber for encoder
        rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr kinematics_subscriber_;

        // Callback - preprocess received message
        void on_encoders_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg);

        void publish_message(const std::vector<uint8_t>& value_to_publish);

        void timer_callback();

        std::vector<double_t> forward(double_t vl, double_t vr);

        std::vector<double_t> inverse(double_t v, double_t w);

    };



}
