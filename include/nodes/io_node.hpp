#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace nodes {
    class IoNode : public rclcpp::Node {
    public:
        // Constructor
        IoNode();
        // Destructor (default)
        ~IoNode() override = default;

        // Function to retrieve the last pressed button value
        int get_button_pressed() const;

        void reset_button();

    private:
        // Variable to store the last received button press value
        int button_pressed_ = -1;

        // Subscriber for button press messages
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_subscriber_;

        void on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg);

    };



}
