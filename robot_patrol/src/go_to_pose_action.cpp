#include <cmath>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "custom_interfaces/action/go_to_pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
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
        desired_pos_.theta = goal->goal_pos.theta * M_PI / 180.0;   // Convertig theta from degrees to radians 
        
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
        current_pos_.theta = wrapAngle(yaw);
    }

    // Avoiding sudden jumps in angle calculation 
    double wrapAngle(double angle) {
        while (angle > M_PI) {
            angle -= 2 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2 * M_PI;
        }    
        
        return angle;
    }

    void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        rclcpp::Rate loop_rate(10);   // Control loop frequency 

        auto feedback = std::make_shared<GoToPoseAction::Feedback>();
        auto result = std::make_shared<GoToPoseAction::Result>();

        bool position_reached = false; // Flag to indicate if position is reached 
        const double POSITION_THRESHOLD = 0.05;
        const double ORIENTATION_THRESHOLD = 0.1;
        const double MIN_ANGULAR_SPEED = 0.05;
        
        while (rclcpp::ok()) {
            // Handle gola cancellation
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

            // Handel the feedback
            feedback->current_pos = current_pos_;
            feedback->current_pos.theta = current_pos_.theta * 180.0 / M_PI; // Show theta in degrees but use calculation in radians 
            goal_handle->publish_feedback(feedback);

            // Step 1: go to x and y position 
            if (!position_reached) {
                // Check if close enogh to target position 
                if (distance_to_goal < POSITION_THRESHOLD) {
                    // Stop the movment 
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                    velocity_publisher_->publish(cmd_vel); // Stop linear movement 

                    position_reached = true;
                }
                else {
                    // Calculate the angle to the target 
                    double target_angle = std::atan2(dy, dx);
                    double angle_diff = std::atan2(std::sin(target_angle - current_pos_.theta), 
                                                   std::cos(target_angle - current_pos_.theta));

                    // Moving to the target position
                    cmd_vel.linear.x = 0.1;                                                                                               // Constant linear speed
                    cmd_vel.angular.z = std::max(std::abs(angle_diff), MIN_ANGULAR_SPEED) * (angle_diff >= 0 ? 1 : -1);       // Proportional angualr speed based on agular difference 
                    velocity_publisher_->publish(cmd_vel);
                }            

            }
            // Step 2: rotat in the theta axis
            else {
                // Adjust orientation in z axis(theta)
                double angle_diff = std::atan2(std::sin(desired_pos_.theta - current_pos_.theta),
                                               std::cos(desired_pos_.theta - current_pos_.theta));

                if (std::abs(angle_diff) < ORIENTATION_THRESHOLD) {
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                    velocity_publisher_->publish(cmd_vel);

                    result->status = true;
                    RCLCPP_INFO(this->get_logger(), "Goal reached with correct orientation!");
                    if (goal_handle->is_active()) {
                        goal_handle->succeed(result);
                    }
                    break;
                }
                else {
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.5 * angle_diff;
                    velocity_publisher_->publish(cmd_vel); // Continue adjusting orientation
                }
            }

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