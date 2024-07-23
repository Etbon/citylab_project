#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <chrono>

class Patrol : public rclcpp::Node {
  public:
    Patrol()
      : Node("patrol_node"),
          safe_distance_(0.7),  // Setting a safe distance
          linear_speed_(0.1),
          angular_speed_factor_(2),
          direction_(0.0) {

        // Create callback groups for mutually exclusive callbacks
        callback_group_1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Subscription options to assign callback group
        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = callback_group_1_;

        // Initialize publisher and subscription
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&Patrol::laserScanCallback, this, std::placeholders::_1), sub_options);

        // Initialize timer with callback
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&Patrol::timerCallback, this), callback_group_2_);
    }

  private:
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        float max_range = 0.0;
        size_t angle_index = msg->ranges.size() / 2;  // Start with forward direction

        // Calculate the indices for -pi/2 to pi/2
        size_t start_index = static_cast<size_t>((-M_PI_2 - msg->angle_min) / msg->angle_increment);
        size_t end_index = static_cast<size_t>((M_PI_2 - msg->angle_min) / msg->angle_increment);

        // Check for obstacles in the front narrow range (e.g., -15 to +15 degrees)
        size_t front_start_index = static_cast<size_t>((-M_PI / 12 - msg->angle_min) / msg->angle_increment);
        size_t front_end_index = static_cast<size_t>((M_PI / 12 - msg->angle_min) / msg->angle_increment);
        float front_range = std::numeric_limits<float>::infinity();

        // Ckeck for obstacle in front 
        for (size_t i = front_start_index; i <= front_end_index; ++i) {
            if (msg->ranges[i] < front_range && msg->ranges[i] != INFINITY) {
                front_range = msg->ranges[i];
            }
        }

        // Check for open space in the wider front range (-pi/2 to pi/2)
        for (size_t i = start_index; i <= end_index; ++i) {
            if (msg->ranges[i] > max_range && msg->ranges[i] != INFINITY) {
                max_range = msg->ranges[i];
                angle_index = i;
            }
        }

        // Calculate direction to turn towards the open space if an obstacle is detected in the narrow front range
        if (front_range < safe_distance_) {
            direction_ = msg->angle_min + angle_index * msg->angle_increment;
            RCLCPP_INFO(get_logger(), "Obstacle detected. Turning to the open space at angle: %f", direction_);
        } 
        else {
            direction_ = 0.0;  // Continue straight if no obstacle detected in the narrow front range
        }
    }

    void timerCallback() {
        geometry_msgs::msg::Twist vel_msg;
        vel_msg.linear.x = linear_speed_;
        vel_msg.angular.z = direction_ / angular_speed_factor_;

        RCLCPP_INFO(this->get_logger(), "Linear Velocity: %f, Angular Velocity: %f", vel_msg.linear.x, vel_msg.angular.z);
        publisher_->publish(vel_msg);
    }

    // ROS communication members
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Callback groups for mutually exclusive callbacks
    rclcpp::CallbackGroup::SharedPtr callback_group_1_;
    rclcpp::CallbackGroup::SharedPtr callback_group_2_;

    // Robot control parameters
    float safe_distance_;
    float linear_speed_;
    float angular_speed_factor_;
    float direction_;
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