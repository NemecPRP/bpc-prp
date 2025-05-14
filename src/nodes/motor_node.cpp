//
// Created by student on 10.3.25.
//

#include "nodes/motor_node.h"

#include <std_msgs/msg/detail/float32__struct.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>

#include "helper.hpp"


std::mutex mtx_motor;

namespace nodes {
    MotorNode::MotorNode(): Node("motor_node"), encoders_data(2, 0), pos_x(0.0), pos_y(0.0), angle(0.0), last_encoders_data_(2, 0) {

        // Initialize the publisher
        motor_publisher_ = create_publisher<std_msgs::msg::UInt8MultiArray>(Topic::set_motor_speeds, 1);

        // Initialize the subscriber
        encoder_subscriber_ = create_subscription<std_msgs::msg::UInt32MultiArray>(
            Topic::encoders, 1, std::bind(&MotorNode::on_encoders_callback, this, std::placeholders::_1));

        // Create a timer
        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(500)),
            std::bind(&MotorNode::timer_callback, this));
    }


    void MotorNode::publish_message(const std::vector<uint8_t>& value_to_publish) {
        auto msg = std_msgs::msg::UInt8MultiArray();
        msg.data = value_to_publish;

        msg.layout.dim.resize(1);

        motor_publisher_->publish(msg);
        //RCLCPP_INFO(get_logger(), "Published:\tL: %hhu\tR: %hhu", msg.data[0], msg.data[1]);
    }


    void MotorNode::set_speeds(const std::vector<uint8_t> &speeds)
    {
        publish_message({127 + speeds[0], 127 + speeds[1]});
    }


    void MotorNode::set_speeds(uint8_t left, uint8_t right)
    {
        publish_message({left, right});
    }


    void MotorNode::stop_motors()
    {
        publish_message({127, 127});
    }


    void MotorNode::timer_callback() {                                      // 0m/s -> 127
        //RCLCPP_INFO(get_logger(), "Timer triggered..");
        //std::vector<uint8_t> stuff = {170, 170};//L: 1690 | R: 4294966130
        //publish_message(stuff);
    }


    // Callback - preprocess received message
    void MotorNode::on_encoders_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_motor);
        ++msg_counter;

        // Get encoder data from message
        encoders_data = msg->data;

        // Return on first callback
        if (first_callback){
            last_encoders_data_ = encoders_data;
            first_callback = false;
            return;
        }

        // Distance per pulse (obvod kola / pulzy na otáčku)
        double dpp = (2 * M_PI * wheel_radius_) / PPR_;

        // Distances traveled since last message
        double dl = (static_cast<double>(encoders_data[0]) - static_cast<double>(last_encoders_data_[0])) * dpp;
        double dr = (static_cast<double>(encoders_data[1]) - static_cast<double>(last_encoders_data_[1])) * dpp;

        // Distance and angle change
        double dd = (dl + dr) / 2.0;
        double da = (dr - dl) / wheel_dist_;

        // Calculate odometry
        pos_x += dd * std::cos(angle + (da / 2.0));
        pos_y += dd * std::sin(angle + (da / 2.0));
        angle += da;

        // Update last encoder data (nebo to dát na začátek? idk)
        last_encoders_data_ = encoders_data;


        // Log into console
        //std::cout << "X: " <<  pos_x << " m\tY: " << pos_y << " m\tTheta: " << (angle * (180.0 / PI)) << " [°]\t" << "{ " << msg_counter <<" }" << std::endl;
        //std::cout << "Encoders:\tL: " <<  encoders_data[0] << "\tR: " << encoders_data[1] << "\n" << std::endl;
    }

}