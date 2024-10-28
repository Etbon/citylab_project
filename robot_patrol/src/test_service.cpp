#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "custom_interfaces/srv/detail/get_direction__struct.hpp"
#include "custom_interfaces/srv/get_direction.hpp"
#include "rclcpp/client.hpp"


class TestService : public rclcpp::Node {
  public:
    TestService()
        : Node("test_service_node") {

        client_ = this->create_client<custom_interfaces::srv::GetDirection>(
            "/direction_service"
        );
        
        // Initialize subscriber 
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&TestService::laser_callback, this, std::placeholders::_1)
        );
    }

  private:
    // Define member variables 
    rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedPtr client_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Check if the service is available 
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Service not available");
            return;
        }

        // Create a request to call the direction service
        auto request = std::make_shared<custom_interfaces::srv::GetDirection::Request>();
        // Fill the request with appproriate data
        request->laser_data = *msg;

        // Asynchronous call to the service
        auto future_result = client_->async_send_request(
            request,
            std::bind(&TestService::client_callback, this, std::placeholders::_1)
        );
    }

    void client_callback(const rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedFuture future) {
        try {
            auto response = future.get();
            RCLCPP_ERROR(this->get_logger(), "Direction service response: %s", response->direction.c_str());
        } 
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }


};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestService>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}