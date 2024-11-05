#include <cmath>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "custom_interfaces/action/detail/go_to_pose__struct.hpp"
#include "custom_interfaces/action/go_to_pose.hpp"
#include "geometry_msgs/msg/detail/pose2_d__struct.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"

class GoToPose : public rclcpp::Node {
  public:
    using GoToPoseAction = custom_interfaces::action::GoToPose;
    using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPoseAction>;
    
    explicit GoToPose()
        : Node("go_to_pose_action_node") {
        
        // Initialize action server 
        this->action_server_ = rclcpp_action::create_server<GoToPoseAction>(
            this, "/go_to_pose",
            std::bind(&GoToPose::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&GoToPose::handle_cancel, this, std::placeholders::_1),
            std::bind(&GoToPose::handle_accepted, this, std::placeholders::_1)
        ); 

        // Set the QoS
        rclcpp::QoS qos(10);
        qos.keep_last(10);
        qos.best_effort();
        qos.durability_volatile();

        // Initiliaze  odometry subscriber 
        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 
            qos,
            std::bind(&GoToPose::odom_callback, this, std::placeholders::_1)
        ); 

        // Initialize publisher
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10
        );

    }

  private:
    // Define memeber variables
    rclcpp_action::Server<GoToPoseAction>::SharedPtr action_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;

    geometry_msgs::msg::Pose2D desired_pos_;  // Target goal position
    geometry_msgs::msg::Pose2D current_pos_;  // Current position base on odometry 

    geometry_msgs::msg::Twist cmd_vel; 

    // Handle goal
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, 
                                            std::shared_ptr<const GoToPoseAction::Goal> goal) {
        (void)uuid; 

        RCLCPP_INFO(this->get_logger(), "Received goal request: x= %f, y= %f, theta= %f", 
            goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta
        );
        
        desired_pos_.x = goal->goal_pos.x;
        desired_pos_.y = goal->goal_pos.y;
        desired_pos_.theta = goal->goal_pos.theta;
        
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Handle cancel
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Handle accepted
    void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        std::thread{std::bind(&GoToPose::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Get the x, y
        this->current_pos_.x = msg->pose.pose.position.x;
        this->current_pos_.y = msg->pose.pose.position.y;

        // Extreact quaternion from the odometry message
        double x = msg->pose.pose.orientation.x;
        double y = msg->pose.pose.orientation.y;
        double z = msg->pose.pose.orientation.z;
        double w = msg->pose.pose.orientation.w;

        // Converting from quaternion to Euler to get the yaw (theta)
        tf2::Quaternion q(x,y,z,w);
        tf2::Matrix3x3 m(q);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);  // Convert to Roll, Pitch, Yaw 
        current_pos_.theta = yaw;    // Store only the yaw as theta
    }

    void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        rclcpp::Rate loop_rate(10);   // Control loop frequency 

        auto feedback = std::make_shared<GoToPoseAction::Feedback>();
        auto result = std::make_shared<GoToPoseAction::Result>();
        
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                // Stop robot 
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                velocity_publisher_->publish(cmd_vel);

                result->status = false;   // Indicate goal was canceled 
                RCLCPP_INFO(this->get_logger(), "Goal canceled");

                goal_handle->canceled(result);
                return;
            }
    
            // Calculate distance to goal 
            double dx = desired_pos_.x - current_pos_.x;
            double dy = desired_pos_.y - current_pos_.y;
            double distance_to_goal = std::sqrt(dx*dx + dy*dy);

            if (goal_handle->is_active()) {
                // Update feedback and publish it 
                feedback->current_pos = current_pos_;
                goal_handle->publish_feedback(feedback);
            
            }
            else {
                // The goal is no longer active, exit 
                break; 
            }
            
            if (distance_to_goal < 0.1) {
                // Stop the movment 
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;

                velocity_publisher_->publish(cmd_vel);

                result->status = true;
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                if (goal_handle->is_active()) {
                    goal_handle->succeed(result);
                }
                break;
            }

            // Calculate the angle to the target 
            double target_angle = std::atan2(dy, dx);
            double angle_diff = target_angle - current_pos_.theta;
            angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff));  // Normalize angle

            // Moving to the target position
            cmd_vel.linear.x = 0.2;                 // Constant linear speed
            cmd_vel.angular.z = 1.0 * angle_diff;    // Proportional angualr speed based on agular difference 

            // Publish the velocity 
            velocity_publisher_->publish(cmd_vel);

            loop_rate.sleep();
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto action_server_ = std::make_shared<GoToPose>();
    rclcpp::spin(action_server_);

    rclcpp::shutdown();
    return 0;
}