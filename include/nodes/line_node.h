#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

namespace nodes {
    enum class DiscreteLinePose {
        LineOnLeft,
        LineOnRight,
        LineNone,
        LineBoth,
    };

    class LineNode : public rclcpp::Node {
    public:

        LineNode();

        ~LineNode() = default;

        // relative pose to line in meters
        float get_continuous_line_pose() const;

        DiscreteLinePose get_discrete_line_pose() const;

        std::vector<uint16_t> line_data;


    private:

        rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_sensors_subscriber_;

        void on_line_sensors_msg(std::shared_ptr<std_msgs::msg::UInt16MultiArray> msg);

        float estimate_continuous_line_pose(float left_value, float right_value);

        DiscreteLinePose estimate_descrete_line_pose(float l_norm, float r_norm);




        //// Publisher, timer
        // rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr line_publisher_;
        // rclcpp::TimerBase::SharedPtr timer_;
//
        //// Callback - preprocess received message
        //void on_line_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
//
        //void publish_message(const std::vector<uint16_t>& value_to_publish);
//
        //void timer_callback();
    };

}
