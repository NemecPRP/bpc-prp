//
// Created by student on 24.3.25.
//

#include "corridor_loop.h"

CorridorLoop::CorridorLoop(nodes::MotorNode &motor_node_, nodes::LidarNode &lidar_node_)
: Node("corridor_loop_node") {



    // Create a timer
    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(100)),
        std::bind(&CorridorLoop::timer_callback, this));
}

void CorridorLoop::timer_callback() {
    auto dir = line_node.get_continuous_line_pose();
    //std::cout << line_node->line_data[0] << std::endl;

    if (dir > 0.0f) motor_node.publish_message({speed_high, speed_low});
    else if (dir < 0.0f) motor_node.publish_message({speed_low, speed_high});
    else motor_node.publish_message({speed_normal, speed_normal});
}
