#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>

namespace nodes {
    class MotorNode : public rclcpp::Node {
    public:
        // Constructor
        MotorNode();
        // Destructor (default)
        ~MotorNode() override = default;

        double pos_x;
        double pos_y;
        double angle;

        std::vector<uint32_t> encoders_data;


        void publish_message(const std::vector<uint8_t>& value_to_publish);

        void set_speeds(const std::vector<uint8_t>& speeds);
        
        void set_speeds(uint8_t left, uint8_t right);

        void stop_motors();


    private:
        // Robot parameters
        const int PPR_ = 576;                // pulsy na otacku
        const double wheel_radius_ = 0.033;   // polomer kol
        const double wheel_dist_ = 0.13;      // vzdalenost kol

        int msg_counter = 0;

        // To calculate distance traveled between encoder readings
        std::vector<uint32_t> last_encoders_data_;
        bool first_callback = true;

        
        // Publisher, timer
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        // Subscriber for encoder
        rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr encoder_subscriber_;

        // Callback - preprocess received message
        void on_encoders_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg);

        void timer_callback();
    };



}
