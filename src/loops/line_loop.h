
#pragma once

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include "nodes/motor_node.h"
#include "nodes/line_node.h"

class LineLoop : public rclcpp::Node {
public:
    // Constructor takes a shared_ptr to an existing node instead of creating one.
    LineLoop(nodes::MotorNode &motor_node, nodes::LineNode &line_node);
    ~LineLoop() = default;

    nodes::MotorNode motor_node;
    nodes::LineNode line_node;


private:
    uint8_t speed_normal = 130;
    uint8_t speed_high = 135;
    uint8_t speed_low = 130;

    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;
};


