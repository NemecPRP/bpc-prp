//
// Created by student on 10.3.25.
//

#include "nodes/lidar_node.h"
#include <mutex>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "helper.hpp"
#include "math.h"


namespace nodes {

    std::mutex mtx2;

    LidarNode::LidarNode(): Node("lidar_node"), dist_front(0.3), dist_back(0), dist_left(0), dist_right(0) {

        // Initialize the subscriber
        lidar_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
             Topic::lidar, 1, std::bind(&LidarNode::on_lidar_sensors_msg, this, std::placeholders::_1));

    }



    void LidarNode::on_lidar_sensors_msg(std::shared_ptr<sensor_msgs::msg::LaserScan> msg) {
        std::lock_guard<std::mutex> lock(mtx2);

        // Get data from message
        std::vector<float> ranges = msg->ranges;
        float angle_min = msg->angle_min;
        float angle_increment = msg->angle_increment;

        // Init
        double angle = 0;
        std::vector<double> dist_front_vect = {};
        std::vector<double> dist_back_vect = {};
        std::vector<double> dist_left_vect = {};
        std::vector<double> dist_right_vect = {};


        // For each LIDAR reading
        for (int i = 0; i < ranges.size(); i++) {

            // Get current distance
            float curr_distance = ranges[i];

            // Set invalid to long distance
            //if (isinf(curr_distance)) curr_distance = 10.0f;
            if (isinf(curr_distance)) continue;
            //if (isnan(curr_distance)) curr_distance = 0.1f;


            // Calculate angle for current distance
            angle = (angle_min + i * angle_increment) * 180.0 / PI;

            // Half size of check cone (haha to nikdo totiž nepochopil)
            double hcc = check_cone_ * 0.5;


            // Check if curr. reading belongs to any of the ranges
            if (angle > (180.0 - hcc) || angle < (-180.0 + hcc))
                dist_front_vect.push_back(curr_distance);

            else if (angle > -hcc && angle < hcc)
                dist_back_vect.push_back(curr_distance);

            else if (angle < (-90.0 + hcc) && angle > (-90.0 - hcc))
                dist_left_vect.push_back(curr_distance);

            else if (angle > (90.0 - hcc) && angle < (90.0 + hcc))
                dist_right_vect.push_back(curr_distance);
        }


        // Calculate average distance to obstacles is all directions
        dist_front = std::accumulate(dist_front_vect.begin(), dist_front_vect.end(), 0.0f) / dist_front_vect.size();
        dist_back = std::accumulate(dist_back_vect.begin(), dist_back_vect.end(), 0.0f) / dist_back_vect.size();
        dist_left = std::accumulate(dist_left_vect.begin(), dist_left_vect.end(), 0.0f) / dist_left_vect.size();
        dist_right = std::accumulate(dist_right_vect.begin(), dist_right_vect.end(), 0.0f) / dist_right_vect.size();


        //if (std::isnan(dist_front)) dist_front = 0.09f;
        //if (std::isnan(dist_back)) dist_back = 0.09f;
        //if (std::isnan(dist_left)) dist_left = 0.09f;
        //if (std::isnan(dist_right)) dist_right = 0.09f;

        //std::cout << "Distances:\tF:" <<  dist_front << "\tB: " << dist_back << "\tL:" << dist_left << "\tR:" << dist_right << std::endl;
    }


}