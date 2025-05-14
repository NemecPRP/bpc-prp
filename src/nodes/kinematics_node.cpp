//
// Created by student on 10.3.25.
//

#include "nodes/kinematics_node.h"

#include <std_msgs/msg/detail/float32__struct.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>

#include "helper.hpp"

namespace nodes {
    KinematicsNode::KinematicsNode(): Node("kinematics_node") {

        // Initialize the publisher
        kinematics_publisher_ = create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::set_motor_speeds, 1);

        // Initialize the subscriber
        kinematics_subscriber_ = create_subscription<std_msgs::msg::UInt32MultiArray>(
            Topic::encoders, 1, std::bind(&KinematicsNode::on_encoders_callback, this, std::placeholders::_1));

        // Create a timer
        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(3000)),
            std::bind(&KinematicsNode::timer_callback, this));
    }

    void KinematicsNode::publish_message(const std::vector<uint8_t>& value_to_publish) {
        auto msg = std_msgs::msg::UInt8MultiArray();
        msg.data = value_to_publish;
        msg.layout.dim.resize(1);
        kinematics_publisher_->publish(msg);
        RCLCPP_INFO(get_logger(), "Published: %hhu; %hhu", msg.data[0], msg.data[1]);
    }
    // start: 4294760038
    // stop: 4294759460

    void KinematicsNode::timer_callback() {
        RCLCPP_INFO(get_logger(), "Timer triggered..");
        std::vector<uint8_t> stuff = {127, 127};
        publish_message(stuff);
    }


    // Callback - preprocess received message
    void KinematicsNode::on_encoders_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
        //encoders_data = msg->data;
        //std::cout << "Encoders:\tL: " <<  encoders_data[0] << "\tR: " << encoders_data[1] << std::endl;
    }

    std::vector<double_t> KinematicsNode::forward(double_t vl, double_t vr) {
        double wl = vl / R;
        double wr = vr / R;

        //double x_der = 0.5*R * (wr + wl) * std::cos()

    }


    std::vector<double_t> inverse(double_t v, double_t w);


}