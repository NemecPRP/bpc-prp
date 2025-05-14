#include <rclcpp/rclcpp.hpp>
#include "RosExampleClass.h"
#include "nodes/io_node.hpp"
#include "nodes/motor_node.h"
#include "nodes/line_node.h"
#include "loops/line_loop.h"
#include "nodes/imu_node.hpp"
#include "nodes/lidar_node.h"
#include "algorithms/pid.h"
#include "nodes/camera_node.hpp"

enum STATE {
    MOVING,
    DECIDING,
    TURNING
};

enum MODE {
    SELECT,
    LINE,
    CORRIDOR,
    MAZE
};



MODE SelectMode(const std::shared_ptr<nodes::IoNode> &io_node) {
    while(rclcpp::ok()) {
        // Line following
        if (io_node->get_button_pressed() == 0) {
            std::cout << "Mode selected:\tLine Following" << std::endl;
            io_node->reset_button();
            return MODE::LINE;
        }

        // Corridor following
        if (io_node->get_button_pressed() == 1) {
            std::cout << "Mode selected:\tCorridor Following" << std::endl;
            io_node->reset_button();
            return MODE::CORRIDOR;
        }

        // Maze escape
        if (io_node->get_button_pressed() == 2) {
            std::cout << "Mode selected:\tMaze escape" << std::endl;
            io_node->reset_button();
            return MODE::MAZE;
        }

        // loops until button input selects mode
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return MODE::SELECT;
}



void LineFollow(const std::shared_ptr<nodes::IoNode> &io_node, const std::shared_ptr<nodes::LineNode> &line_node, const std::shared_ptr<nodes::MotorNode> &motor_node) {
    // Speed settings
    uint8_t speed_normal = 130;
    uint8_t speed_high = 135;
    uint8_t speed_low = 130;



    // Wait at start
    std::cout << "Press button to start" << std::endl;
    while(rclcpp::ok()) {
        if (io_node->get_button_pressed() != -1) {
            io_node->reset_button();
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Main loop
    std::cout << "Line following program started" << std::endl;
    while(rclcpp::ok()) {
        // Get offset direction
        auto dir = line_node->get_continuous_line_pose();

        // Compensate for error
        if (dir > 0.0f) motor_node->publish_message({speed_high, speed_low});
        else if (dir < 0.0f) motor_node->publish_message({speed_low, speed_high});
        else motor_node->publish_message({speed_normal, speed_normal});


        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}



void CorridorFollow(const std::shared_ptr<nodes::IoNode> &io_node, const std::shared_ptr<nodes::LidarNode> &lidar_node, const std::shared_ptr<nodes::MotorNode> &motor_node) {
    // Robot State
    STATE state = MOVING;

    // PID
    algorithms::Pid pid = algorithms::Pid(3.0f, 0.02f, 2.5f);

    // Speed settings
    uint8_t move_speed = 135;
    uint8_t speed_compensation = 20;
    std::vector<uint8_t> speeds = {move_speed, move_speed};

    // Turn settings
    float max_turn_time = 1.9f;
    float turn_speed = 6.0f;

    // Turn timer
    float turn_timer = 0.0f;




    // Wait at start
    std::cout << "Press button to start" << std::endl;
    while(rclcpp::ok()) {
        if (io_node->get_button_pressed() != -1) {
            io_node->reset_button();
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }


    // Main loop
    std::cout << "Corridor following program started" << std::endl;
    while(rclcpp::ok()) {
        // State machine
        switch (state) {
            case MOVING:
                // Obstacle in front
                if (lidar_node->dist_front <= 0.2) {
                    std::cout << "dist_front " << lidar_node->dist_front << std::endl;
                    motor_node->stop_motors();
                    state = DECIDING;
                }

                // Clear path
                else {
                    // TODO: Nějaká lepší regulace udržení uprostřed koridoru
                    float dist_left = std::clamp(lidar_node->dist_left, 0.0f, 0.2f);
                    float dist_right = std::clamp(lidar_node->dist_right, 0.0f, 0.2f);

                    float diff = dist_right - dist_left;
                    uint8_t final_compensation = static_cast<uint8_t>(speed_compensation * abs(pid.step(diff, 0.1f)));


                    if (dist_left < dist_right) speeds = {move_speed + final_compensation, move_speed};
                    else if (dist_right < dist_left) speeds = {move_speed, move_speed + final_compensation};
                    else speeds = {move_speed, move_speed};

                    motor_node->publish_message(speeds);
                }
            break;


            case DECIDING:
                // Left clear
                if (lidar_node->dist_left > lidar_node->dist_right) speeds = {127 - turn_speed, 127 + turn_speed};
                // Right clear
                else speeds = {127 + turn_speed, 127 - turn_speed};

                // Start turning
                turn_timer = 0.0f;
                state = TURNING;
            break;


            case TURNING:
                // Timer still running
                if (turn_timer < max_turn_time) {
                    motor_node->publish_message(speeds);
                    turn_timer += 0.1f;
                }

                // Timer timeout
                else {
                    turn_timer = 0.0f;
                    state = MOVING;
                }
            break;
        }


        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}


bool IsOnIntersection(float dist_front, float dist_left, float dist_right) {
    int paths = 0;
    float cell_dist = std::fmod(dist_front * 100, 40.0f);

    if (dist_front > 0.35f) paths++;
    if (dist_left > 0.35f) paths++;
    if (dist_right > 0.35f) paths++;

    std::cout << "Paths: " << paths << " (" << (paths > 1 && cell_dist >= 15.0f && cell_dist <= 25.0f) << ")" << std::endl;

    // Is on intersection if there is more than 1 path && is in the middle of the tile
    return  paths > 1
            && cell_dist >= 15.0f && cell_dist <= 25.0f;
}


void MazeEscape(const std::shared_ptr<nodes::IoNode> &io_node, const std::shared_ptr<nodes::ImuNode> &imu_node, const std::shared_ptr<nodes::LidarNode> &lidar_node,
    const std::shared_ptr<nodes::MotorNode> &motor_node, const std::shared_ptr<nodes::CameraNode> &camera_node) {

    // Robot State
    STATE state = MOVING;

    // PID
    algorithms::Pid pid = algorithms::Pid(3.0f, 0.02f, 2.5f);

    // Speed settings
    uint8_t move_speed = 138;
    uint8_t speed_compensation = 20;
    std::vector<uint8_t> speeds = {move_speed, move_speed};

    // Turn settings
    float turn_speed = 6.0f;
    float target_rot = 0.0f;
    float max_turn_time = 1.9f;

    // Turn timer
    float turn_timer = 0.0f;



    // Marker queue
    int next_dir_id = 0;
    std::vector<int> markers = std::vector<int>(0);

    float check_intersection_time = 0.1f;
    float movement_timer = 0.0f;

    float decision_cooldown_timer = 0.0f;
    float decision_cooldown = 3.3f;
    bool is_on_intersection = false;



    // Wait at start
    std::cout << "Press button to start calibration" << std::endl;
    while(rclcpp::ok()) {
        if (io_node->get_button_pressed() != -1) {
            imu_node->setMode(nodes::ImuNodeMode::CALIBRATE);
            io_node->reset_button();
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }


    // IMU calibration
    std::cout << "Calibrating, press button to finish..." << std::endl;
    while(rclcpp::ok()) {
        if (io_node->get_button_pressed() != -1) {
            std::cout << "Calibration finished" << std::endl;
            imu_node->setMode(nodes::ImuNodeMode::INTEGRATE);
            io_node->reset_button();
            break;
        }
    }


    // Main loop
    std::cout << "Maze escape program started" << std::endl;
    while(rclcpp::ok()) {

        // State machine
        switch (state) {
            case MOVING:

                // Obstacle in front
                if (lidar_node->dist_front <= 0.2) {
                    std::cout << "Wall in front (" << lidar_node->dist_front << " m)\n" << std::endl;
                    motor_node->stop_motors();
                    state = DECIDING;
                }

                // Clear path
                else {

                    // Get marker id from camera
                    int marker = camera_node->aruco_detector.getId();

                    // Add it to queue
                    if (marker >= 0 && marker <= 2) {
                        // First marker
                        if (markers.empty()) markers.push_back(marker);

                        // Add to queue when it's not same as the last one
                        else if (markers[markers.size() - 1] != marker) markers.push_back(marker);
                    }

                    // Print marker queue
                    for (int i = 0; i < markers.size(); ++i) {
                        if (i == 0) std::cout << "Marker queue: ";
                        std::cout << markers[i] << " ";
                        if (i == markers.size() - 1) std::cout << "(" << next_dir_id << ")" << std::endl;
                    }


                    // Intersection check
                    if (movement_timer >= check_intersection_time && decision_cooldown_timer <= 0.0f) {

                        // Found intersection
                        if (IsOnIntersection(lidar_node->dist_front, lidar_node->dist_left, lidar_node->dist_right)) {
                            std::cout << "INTERSECTION DETECTED" << std::endl;
                            motor_node->stop_motors();
                            state = DECIDING;
                            break;
                        }

                        movement_timer = 0.0f;
                    }


                    // Keep in corridor center
                    float diff;
                    float dist_left;
                    float dist_right;
                    if (lidar_node->dist_right > 0.4f) {
                        dist_left = lidar_node->dist_left;
                        dist_right = lidar_node->dist_right;
                        diff = 0.2f - dist_left;
                    }
                    else if (lidar_node->dist_left > 0.4f) {
                        dist_left = lidar_node->dist_left;
                        dist_right = lidar_node->dist_right;
                        diff = dist_right - 0.2f;
                    }
                    else {
                        dist_left = std::clamp(lidar_node->dist_left, 0.0f, 0.2f);
                        dist_right = std::clamp(lidar_node->dist_right, 0.0f, 0.2f);


                        std::cout << "L: " << dist_left << " R: " << dist_right << std::endl;

                        diff = dist_right - dist_left;
                    //}
                    uint8_t final_compensation = static_cast<uint8_t>(speed_compensation * abs(pid.step(diff, 0.1f)));
                    uint8_t bottom = 0;
                    uint8_t ceilling = 2;
                    final_compensation = std::clamp(final_compensation, bottom, ceilling);

                    if (dist_left < dist_right) speeds = {move_speed + final_compensation, move_speed};
                    else if (dist_right < dist_left) speeds = {move_speed, move_speed + final_compensation};
                    else speeds = {move_speed, move_speed};

                    motor_node->publish_message(speeds);
                }
            break;


            case DECIDING:
                // Intersection check
                is_on_intersection = IsOnIntersection(lidar_node->dist_front, lidar_node->dist_left, lidar_node->dist_right);

                // Intersection
                if (is_on_intersection) {

                    // Get next direction
                    int next_dir = -1;
                    if (!markers.empty() && next_dir_id < markers.size()) next_dir = markers[next_dir_id];
                    std::cout << "DECIDING ON INTERSECTION (" << "marker " << next_dir << ")" << std::endl;

                    // Set rotation speeds
                    switch (next_dir) {
                        // Straight
                        case 0: speeds = {move_speed, move_speed}; break;

                        // Left
                        case 1: speeds = {127 - turn_speed, 127 + turn_speed}; break;

                        // Right
                        case 2: speeds = {127 + turn_speed, 127 - turn_speed}; break;

                        // None -> forward
                        default:
                            if (lidar_node->dist_left > lidar_node->dist_right) speeds = {127 - turn_speed, 127 + turn_speed};

                            // Right clear
                            else speeds = {127 + turn_speed, 127 - turn_speed};


                    }

                    // Reset intersection status
                    is_on_intersection = false;

                    // Mark last dir as completed
                    if (!markers.empty()) markers.back() = 5;

                    // Select next optimal path if there are any
                    if (next_dir_id < markers.size()) next_dir_id++;
                }

                // Wall in front
                else {
                    // Left clear
                    if (lidar_node->dist_left > lidar_node->dist_right) speeds = {127 - turn_speed, 127 + turn_speed};

                    // Right clear
                    else speeds = {127 + turn_speed, 127 - turn_speed};
                }

                // Start turning
                turn_timer = 0.0f;

                state = TURNING;
            break;


            case TURNING:
                // Timer still running
                if (turn_timer < max_turn_time) {
                    motor_node->publish_message(speeds);
                    turn_timer += 0.1f;
                }

                // Timer timeout
                else {
                    // Reset timers
                    turn_timer = 0.0f;
                    movement_timer = 0.0f;
                    decision_cooldown_timer = decision_cooldown;

                    // Switch state
                    state = MOVING;
                }
            break;
        }

        // Increment timers
        movement_timer += 0.1f;
        decision_cooldown_timer -= 0.1f;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}



int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Init nodes
    auto io_node = std::make_shared<nodes::IoNode>();
    auto motor_node = std::make_shared<nodes::MotorNode>();
    auto line_node = std::make_shared<nodes::LineNode>();
    auto lidar_node = std::make_shared<nodes::LidarNode>();
    auto imu_node = std::make_shared<nodes::ImuNode>();
    auto camera_node = std::make_shared<nodes::CameraNode>();

    // Add them to executor
    executor->add_node(io_node);
    executor->add_node(motor_node);
    executor->add_node(line_node);
    executor->add_node(lidar_node);
    executor->add_node(imu_node);
    executor->add_node(camera_node);

    // Run
    auto executor_thread = std::thread([&executor]() { executor->spin(); });



    // Robot Mode -> Choose: Line Following - Button 1
    //                       Corridor Following - Button 2
    //                       Maze Following - Button 3


    // MODE SELECTION LOOP
    std::cout << "Please choose a mode" << std::endl;
    std::cout << "Button 1 - Line Following\t" << "Button 2 - Corridor Following\t" << "Button 3 - Maze" << std::endl;

    MODE mode = SELECT;
    while (mode == SELECT) mode = SelectMode(io_node);



    // PROGRAM LOOP (based on MODE)
    switch (mode) {

        // LINE FOLLOW PROGRAM
        case LINE: LineFollow(io_node, line_node, motor_node);

        // CORRIDOR FOLLOW PROGRAM
        case CORRIDOR: CorridorFollow(io_node, lidar_node, motor_node);

        // MAZE ESCAPE
        //case MAZE: MazeEscapeORIGINAL(io_node, imu_node, lidar_node, motor_node);
        case MAZE: MazeEscape(io_node, imu_node, lidar_node, motor_node, camera_node);

        // ERROR (how?)
        default: break;

    }

    // Shutdown ROS 2
    rclcpp::shutdown();
}

