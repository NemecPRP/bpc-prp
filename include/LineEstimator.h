
#include <rclcpp/rclcpp.hpp>


enum class DiscreteLinePose {
    LineOnLeft,
    LineOnRight,
    LineNone,
    LineBoth,
};

static const double l_min = 25.0;
static const double l_max = 600.0;
static const double r_min = 25.0;
static const double r_max = 540.0;

static const double threshold = 0.5;

class LineEstimator {

    LineEstimator() = default;
    ~LineEstimator() = default;




public:
    static DiscreteLinePose estimate_discrete_line_pose(uint16_t left_val, uint16_t right_val) {
        // Normalizace
        double left = (left_val - l_min) / (l_max - l_min);
        double right  = (right_val - r_min) / (r_max - r_min);
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


    static float estimate_continuous_line_pose(uint16_t left_val, uint16_t right_val) {
        // Normalizace
        double left = (left_val - l_min) / (l_max - l_min);
        double right  = (right_val - r_min) / (r_max - r_min);

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

        //return 0.0f;
    }

};