#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <chrono>
#include <memory>
#include <algorithm> // Added for std::transform

#include "custom_interfaces/srv/get_direction.hpp"

class Patrol : public rclcpp::Node {
public:
    Patrol()
        : Node("patrol_with_service_node"), laser_data_received_(false) {

        // Create callback groups for mutually exclusive callbacks
        laser_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // Subscription options to assign callback group
        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = laser_callback_group_;

        // Initialize publisher and subscription
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Initialize client
        client_ = this->create_client<custom_interfaces::srv::GetDirection>("/direction_service");

        // Initialize laser subscription
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&Patrol::laser_scan_callback, this, std::placeholders::_1),
            sub_options
        );

        // Initialize timer with callback
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200), std::bind(&Patrol::timer_callback, this), 
            timer_callback_group_
        );
    }

private:
    // ROS communication members
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedPtr client_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Callback groups for mutually exclusive callbacks
    rclcpp::CallbackGroup::SharedPtr laser_callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

    // Latest laser data
    sensor_msgs::msg::LaserScan last_laser_;

    // Robot control parameters
    std::string current_direction_;
    bool laser_data_received_;

    // Fill request with laser data
    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_laser_ = *msg; // Store the latest laser message
        laser_data_received_ = true;
    }

    // Handle response 
    void client_callback(const rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedFuture future) {
        auto response = future.get();

        if (!response) {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive a valid response from the service");
            return;
        }
        
        // Store the current direction 
        current_direction_ = response->direction;

        // Create and set velocity message based on the response direction
        geometry_msgs::msg::Twist vel_msg;
        if (current_direction_ == "right") {
            vel_msg.linear.x = 0.05;
            vel_msg.angular.z = -0.5;
        }
        else if (current_direction_ == "forward") {
            vel_msg.linear.x = 0.05;
            vel_msg.angular.z = 0.0;
        }
        else if (current_direction_ == "left") {
            vel_msg.linear.x = 0.05;
            vel_msg.angular.z = 0.5;
        }

        // Publish the velocity message
        publisher_->publish(vel_msg);

        RCLCPP_INFO(this->get_logger(), "Direction received: %s", response->direction.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing velocities - Linear: %f, Angular: %f", vel_msg.linear.x, vel_msg.angular.z);
    }

    // Control loop 
    void timer_callback() {
        // Wait for service availability 
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Service was not available");
            return;
        }

        if (!laser_data_received_) {
            RCLCPP_WARN(this->get_logger(), "Laser data not yet received");
            return;
        }
        
        // Create a request using the latest laser data
        auto request = std::make_shared<custom_interfaces::srv::GetDirection::Request>();
        request->laser_data = last_laser_; 

        // Call the service asynchronously
        auto future_result = client_->async_send_request(
            request,
            std::bind(&Patrol::client_callback, this, std::placeholders::_1)
        );
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Patrol>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}