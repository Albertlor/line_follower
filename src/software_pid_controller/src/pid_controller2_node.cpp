#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>


class LineFollowingPIDController : public rclcpp::Node {
public:
    LineFollowingPIDController()
    : Node("line_following_pid_controller"),
      kp_(16.0),
      kd_(0.0001),
      ki_(0.0000001),
      dt_(0.0005), 
      integral_(0.0),
      previous_error_(0.0) {
        error_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "error_state", 10, std::bind(&LineFollowingPIDController::errorCallback, this, std::placeholders::_1));
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(500), std::bind(&LineFollowingPIDController::controlLoop, this));
    }

private:
    float kp_, kd_, ki_, dt_;
    float integral_, previous_error_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr error_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist cmd_vel_;

    void errorCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        float error = msg->data; // The error from the line center
        float derivative = (error - previous_error_) / dt_;
        integral_ += error * dt_;
        float output = kp_ * error + kd_ * derivative + ki_ * integral_;

        // Update the control command
        cmd_vel_.linear.x = 0.5;  // Constant forward speed
        cmd_vel_.angular.z = output;  // Angular velocity proportional to PID output

        // Update previous error
        previous_error_ = error;
    }

    void controlLoop() {
        // Publish the velocity command
        velocity_publisher_->publish(cmd_vel_);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LineFollowingPIDController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}