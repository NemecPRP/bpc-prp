#pragma once


#include <iostream>
#include <cmath>
#include <numeric>

namespace algorithms {

    class PlanarImuIntegrator {
    public:

        PlanarImuIntegrator() : theta_(0.0f), gyro_offset_(0.0f) {}

        // TODO: Call this regularly to integrate gyro_z over time
        void update(float gyro_z, double dt) {
            theta_ += dt * (gyro_z - gyro_offset_);

            //std::cout << "Theta: " << (theta_ * (180.0 / M_PI)) << "\tGyro offset: " << (gyro_offset_ * (180.0 / M_PI)) << std::endl;
            //std::cout << "Theta: " << theta_<< "\tGyro offset: " << gyro_offset_ << std::endl;
        }

        // TODO: Calibrate the gyroscope by computing average from static samples
        void setCalibration(std::vector<float> gyro) {
            gyro_offset_ = std::accumulate(gyro.begin(), gyro.end(), 0.0f) / gyro.size();
        }

        // TODO: Return the current estimated yaw
        [[nodiscard]] float getYaw() const {
            //std::cout << "Angle: " << (theta_ * (180.0 / M_PI)) << std::endl;

            float angle = theta_;
            if(angle > M_PI) angle -= 2 * M_PI;
            if(angle < -M_PI) angle += 2 * M_PI;

            return angle;
            // return theta_;
        }

        // TODO: Reset orientation and calibration
        void reset() {
            theta_ = 0.0f;
            gyro_offset_ = 0.0f;
        }

    private:
        float theta_;       // Integrated yaw angle (radians)
        float gyro_offset_; // Estimated gyro bias
    };
}
