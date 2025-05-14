//
// Created by student on 10.3.25.
//

#include "nodes/line_node.h"
#include "LineEstimator.h"
#include <mutex>


#include <std_msgs/msg/u_int16_multi_array.hpp>

#include "helper.hpp"

namespace nodes {
    // LineNode::LineNode() : Node("line_node") {
    //
    // }


    static const double l_min = 15.0;   //20
    static const double l_max = 380.0;  //390
    static const double r_min = 15.0;   //20
    static const double r_max = 310.0;  //140

    static const double threshold = 0.3;

    std::mutex mtx;

    LineNode::LineNode(): Node("line_node"), line_data(2,127) {

        // Initialize the publisher
        //line_publisher_ = create_publisher<std_msgs::msg::UInt16MultiArray>(Topic::set_motor_speeds, 1);

        // Initialize the subscriber
        line_sensors_subscriber_ = create_subscription<std_msgs::msg::UInt16MultiArray>(
             Topic::line_sensors, 1, std::bind(&LineNode::on_line_sensors_msg, this, std::placeholders::_1));

        //// Create a timer
        //timer_ = create_wall_timer(
        //    std::chrono::milliseconds(static_cast<int>(3000)),
        //    std::bind(&LineNode::timer_callback, this));
    }



    void LineNode::on_line_sensors_msg(std::shared_ptr<std_msgs::msg::UInt16MultiArray> msg) {
        std::lock_guard<std::mutex> lock(mtx);
        line_data = msg->data;

        //std::cout << "Line sensors:\tL: " <<  line_data[0] << "\tR: " << line_data[1] << std::endl;
        //LineEstimator::estimate_discrete_line_pose(line_data[0], line_data[1]);
        //LineEstimator::estimate_continuous_line_pose(line_data[0], line_data[1]);

    }


    DiscreteLinePose LineNode::get_discrete_line_pose() const {

        std::lock_guard<std::mutex> lock(mtx);

        // Normalizace
        double left = (line_data[0] - l_min) / (l_max - l_min);
        double right  = (line_data[1] - r_min) / (r_max - r_min);
        //std::cout << "L: " << left << "\tR: " << right << std::endl;

        // Threshold
        if (left > threshold && right > threshold) {
            //std::cout << "Both" << std::endl;
            return DiscreteLinePose::LineBoth;
        }
        if (left > threshold) {
            //std::cout << "OnLeft" << std::endl;
            return DiscreteLinePose::LineOnLeft;
        }
        if (right > threshold) {
            //std::cout << "OnRight" << std::endl;
            return DiscreteLinePose::LineOnRight;
        }
        //std::cout << "None" << std::endl;
        return DiscreteLinePose::LineNone;
    }


    float LineNode::get_continuous_line_pose() const {
        std::lock_guard<std::mutex> lock(mtx);

        //return LineEstimator::estimate_continuous_line_pose(line_data[0], line_data[1]);
        // Normalizace
        double left = (line_data[0] - l_min) / (l_max - l_min);
        double right  = (line_data[1] - r_min) / (r_max - r_min);

        ////std::cout << "Estimation: " << left+right << std::endl;
        //auto stuff = (right - left) / (right + left) * (2.9 / 2.0);
        //std::cout << stuff << " cm" << std::endl;

        // Threshold
        if (left > threshold && right > threshold) {
            //std::cout << "Both" << std::endl;
            return 0.0f;
        }
        if (left > threshold) {
            //std::cout << "OnLeft" << std::endl;
            return -1.0f;
        }
        if (right > threshold) {
            //std::cout << "OnRight" << std::endl;
            return 1.0f;
        }
        //std::cout << "None" << std::endl;
        return 0.0f;
    }



    //void LineNode::publish_message(const std::vector<uint16_t>& value_to_publish) {
    //    auto msg = std_msgs::msg::UInt16MultiArray();
    //    msg.data = value_to_publish;
    //    msg.layout.dim.resize(1);
    //    line_publisher_->publish(msg);
    //    RCLCPP_INFO(get_logger(), "Published:\tL: %hhu\tR: %hhu", msg.data[0], msg.data[1]);
    //}


    //void LineNode::timer_callback() {
    //    //RCLCPP_INFO(get_logger(), "Timer triggered..");
    //    //std::vector<uint16_t> stuff = {255, 255};//L: 1690 | R: 4294966130
    //    //publish_message(stuff);
    //}


    // Callback - preprocess received message
    //void LineNode::on_line_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
    //    line_data = msg->data;
    //    std::cout << "Encoders:\tL: " <<  line_data[0] << "\tR: " << line_data[1] << std::endl;
    //}

}