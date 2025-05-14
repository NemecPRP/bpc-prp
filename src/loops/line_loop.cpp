//
// Created by student on 24.3.25.
//

#include "line_loop.h"

LineLoop::LineLoop(nodes::MotorNode &motor_node_, nodes::LineNode &line_node_)
: Node("line_loop_node") {



    // Create a timer
    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(100)),
        std::bind(&LineLoop::timer_callback, this));
}

void LineLoop::timer_callback() {
    auto dir = line_node.get_continuous_line_pose();
    //std::cout << line_node->line_data[0] << std::endl;

    std::cout << "Line sensors:\tL: " <<  line_node.line_data[0] << "\tR: " << line_node.line_data[1] << std::endl;

    if (dir > 0.0f) motor_node.publish_message({speed_high, speed_low});
    else if (dir < 0.0f) motor_node.publish_message({speed_low, speed_high});
    else motor_node.publish_message({speed_normal, speed_normal});
}
