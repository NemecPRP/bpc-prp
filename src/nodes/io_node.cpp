#include "nodes/io_node.hpp"
#include "helper.hpp"

namespace nodes {
    IoNode::IoNode(): Node("io_node") {
        // Initialize the subscriber
        button_subscriber_ = create_subscription<std_msgs::msg::UInt8>(
            Topic::buttons, 1, std::bind(&IoNode::on_button_callback, this, std::placeholders::_1));

    }

    int IoNode::get_button_pressed() const {
        return button_pressed_;
    }

    void IoNode::reset_button() {
        button_pressed_ = -1;
    }

    void IoNode::on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        button_pressed_ = msg->data;
        //std::cout << "button_pressed_: " << button_pressed_ << std::endl;
    }

}

