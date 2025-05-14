
#pragma once

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include "nodes/motor_node.h"
#include "nodes/lidar_node.h"

class CorridorLoop : public rclcpp::Node {
public:
    // Constructor takes a shared_ptr to an existing node instead of creating one.
    //CorridorLoop(nodes::MotorNode &motor_node, nodes::CorridorLoop &line_node);
    ~CorridorLoop() = default;

    nodes::MotorNode motor_node;
    nodes::LidarNode lidar_node;


private:
    uint8_t speed_normal = 130;
    uint8_t speed_high = 135;
    uint8_t speed_low = 130;

    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;
};


