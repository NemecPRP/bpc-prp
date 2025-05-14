
#include <rclcpp/rclcpp.hpp>


enum class DiscreteLinePose {
    LineOnLeft,
    LineOnRight,
    LineNone,
    LineBoth,
};

class LineEstimator {

    LineEstimator() = default;
    ~LineEstimator() = default;



    static DiscreteLinePose estimate_discrete_line_pose(uint16_t left_val, uint16_t right_val) {
        double_t left = (left_val - 25) / (600 - 25);
        double_t right  = (right_val - 25) / (540 - 25);

        //std::cout << "L: " << left << "\tR: " << right << std::endl;
    }

};