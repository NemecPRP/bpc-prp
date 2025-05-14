#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace nodes {

    class LidarNode : public rclcpp::Node {
    public:

        LidarNode();

        ~LidarNode() = default;

        // Distances to obstacles
        float dist_front;
        float dist_back;
        float dist_left;
        float dist_right;


    private:
        // Angle used to calculate avg. dist. from objects using LIDAR
        const float check_cone_ = 15.0;


        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;

        void on_lidar_sensors_msg(std::shared_ptr<sensor_msgs::msg::LaserScan> msg);

    };

}
