#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <utility>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "custom_interfaces/srv/get_direction.hpp"

class DirectionService : public rclcpp::Node {
public:
    DirectionService()
        : Node("direction_service_node"), safe_distance_(0.5) {
        
        // Initialize service 
        service_ = this->create_service<custom_interfaces::srv::GetDirection>(
            "/direction_service", 
            std::bind(&DirectionService::direction_service_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "Direction Service Server Ready with safe_distance: %.2f meters", 
                    safe_distance_);
    }

private:
    // Define member variables 
    rclcpp::Service<custom_interfaces::srv::GetDirection>::SharedPtr service_;

    // Safety distances 
    float safe_distance_;

    // Handle the laser data 
    void direction_service_callback(const std::shared_ptr<custom_interfaces::srv::GetDirection::Request> request,
                                    std::shared_ptr<custom_interfaces::srv::GetDirection::Response> response) {

        const auto &laser_data = request->laser_data;

        // Calculate indices for the right, front, and left sectors (each 60 degrees)
        size_t right_laser_start_idx = angle_to_index(-M_PI / 2, laser_data);  // -90 degrees
        size_t right_laser_end_idx = angle_to_index(-M_PI / 6, laser_data);    // -30 degrees

        size_t front_laser_start_idx = angle_to_index(-M_PI / 6, laser_data);  // -30 degrees
        size_t front_laser_end_idx = angle_to_index(M_PI / 6, laser_data);     // +30 degrees

        size_t left_laser_start_idx = angle_to_index(M_PI / 6, laser_data);    // +30 degrees
        size_t left_laser_end_idx = angle_to_index(M_PI / 2, laser_data);      // +90 degrees

        // Sum distances in each sector with bounds checking
        std::pair<float, float> right_result = get_total_and_min_distance_section(laser_data.ranges, right_laser_start_idx, right_laser_end_idx);
        std::pair<float, float> front_result = get_total_and_min_distance_section(laser_data.ranges, front_laser_start_idx, front_laser_end_idx);
        std::pair<float, float> left_result = get_total_and_min_distance_section(laser_data.ranges, left_laser_start_idx, left_laser_end_idx);

        // Decide the safest direction
        response->direction = decide_direction(right_result.first, front_result.second, left_result.first);

        RCLCPP_INFO(this->get_logger(), "Right Total: %.2f, Front Total: %.2f, Left Total: %.2f, Direction chosen: %s", 
            right_result.first, front_result.first, left_result.first, response->direction.c_str());
    }

    // Convert angle to index in the ranges array
    int angle_to_index(double angle, const sensor_msgs::msg::LaserScan &laser_data) {
        double raw_index = (angle - laser_data.angle_min) / laser_data.angle_increment;
        int angle_index = static_cast<int>(std::round(raw_index));
        
        // Clamp index to valid range
        if (angle_index < 0) {
            angle_index = 0;
        }

        if (angle_index >= static_cast<int>(laser_data.ranges.size())) {
            angle_index = laser_data.ranges.size() - 1;
        }

        return angle_index;
    }

    // Calculate total and minimum distances within a sector
    std::pair<float, float> get_total_and_min_distance_section(const std::vector<float> &ranges, int start_idx, int end_idx) {
        float total_distance_sum = 0.0;
        float min_distance = std::numeric_limits<float>::max();

        // Ensure we don't exceed the array bounds
        if (start_idx < 0) {
            start_idx = 0;
        }
        if (end_idx >= static_cast<int>(ranges.size())) {
            end_idx = ranges.size() - 1;
        }       
        
        for (int i = start_idx; i <= end_idx; ++i) {
            float distance = ranges[i];
            if (std::isfinite(distance) && distance > 0.0) {
                total_distance_sum += distance;
                if (distance < min_distance) {
                    min_distance = distance;
                }
            }
        }

        // If no valid readings, set min_distance to 0
        if (min_distance == std::numeric_limits<float>::max()) {
            min_distance = 0.0;
        }

        return std::make_pair(total_distance_sum, min_distance);
    }

    std::string previous_direction = "forward";
    
    // Decide the safest direction based on total and minimum distances 
    std::string decide_direction(float right_total, float front_total, float left_total) {
        // Default direction is forward
        std::string direction = "forward";

        // Check if front is blocked
        if (front_total < safe_distance_) {
            if (left_total > right_total * 1.2 &&  previous_direction != "left") {
                direction = "left";
            }
            else if (right_total > left_total * 1.2 && previous_direction != "right") {
                direction = "right";
            }
            else {
                direction = previous_direction; // Stick to the current direction
            }
        }
    
        previous_direction = direction;
        return direction;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DirectionService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
