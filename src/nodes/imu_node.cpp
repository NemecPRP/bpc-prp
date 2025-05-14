#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "algorithms/planar_imu_integrator.hpp"
#include "nodes/imu_node.hpp"
#include "helper.hpp"

namespace nodes {


    ImuNode::ImuNode(): Node("imu_node"), prev_time(this->now()) {

        imu_subscriber_ = create_subscription<sensor_msgs::msg::Imu>(
            Topic::imu, 1, std::bind(&ImuNode::on_imu_msg, this, std::placeholders::_1));
    }

    void ImuNode::setMode(const ImuNodeMode setMode) {
        mode = setMode;
        if (mode == ImuNodeMode::INTEGRATE) calibrate();
        else gyro_calibration_samples_.clear();
    }

    float ImuNode::getIntegratedResults() {
        return planar_integrator_.getYaw();
    }

    void ImuNode::reset_imu() {
        planar_integrator_.reset();
    }

    void ImuNode::calibrate() {
        planar_integrator_.setCalibration(gyro_calibration_samples_);

    }

    void ImuNode::integrate() {
    }

    void ImuNode::on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg) {
        double gyro_z = msg->angular_velocity.z;

        if (mode == ImuNodeMode::CALIBRATE) {
            gyro_calibration_samples_.push_back(gyro_z);
            //calibrate();
        }

        else {
            planar_integrator_.update(gyro_z,this->now().seconds() - prev_time.seconds());
            prev_time = this->now();
        }
    }
}
