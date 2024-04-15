#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>


class LineFollowingPIDController : public rclcpp::Node {
public:
    LineFollowingPIDController()
    : Node("line_following_pid_controller"),
      kp_(1.0),
      kd_(0.00001),
      ki_(0.00000001),
      dt_(0.0005), 
      integral_(0.0),
      previous_error_(0.0),
      base_speed(0.3),
      left_speed(0.0),
      right_speed(0.0),
      angular_speed(0.01) {
        error_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "error_state", 10, std::bind(&LineFollowingPIDController::errorCallback, this, std::placeholders::_1));
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(500), std::bind(&LineFollowingPIDController::controlLoop, this));
    }

private:
    float kp_, kd_, ki_, dt_;
    float integral_, previous_error_;
    float base_speed;
    float left_speed, right_speed;
    float angular_speed;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr error_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist cmd_vel_;

    void errorCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        float error = msg->data; // The error from the line center
        float derivative = (error - previous_error_) / dt_;
        integral_ += error * dt_;
        float output = kp_ * error + kd_ * derivative + ki_ * integral_;

        left_speed=base_speed-output;
        right_speed=base_speed+output;

        if(left_speed>1.5)
            left_speed=1.5;
        if(right_speed>1.5)
            right_speed=1.5;
        if(left_speed<-1.5)
            left_speed=-1.5;
        if(right_speed<-1.5)
            right_speed=-1.5;

        angular_speed=right_speed-left_speed;
        if(angular_speed>1.2)
            angular_speed=1.2;
        if(angular_speed<-1.2)
            angular_speed=-1.2;

        // Update the control command
        cmd_vel_.linear.x = (left_speed+right_speed)/2.0;  // Constant forward speed
        cmd_vel_.angular.z = (angular_speed);  // Angular velocity proportional to PID output

        // Update previous error
        previous_error_ = error;

        RCLCPP_INFO(this->get_logger(),"Error: %s",std::to_string(error).c_str());
        RCLCPP_INFO(this->get_logger(),"Right Speed: %s",std::to_string(right_speed).c_str());
        RCLCPP_INFO(this->get_logger(),"Left Speed: %s",std::to_string(left_speed).c_str());
        RCLCPP_INFO(this->get_logger(),"Left Speed: %s",std::to_string(angular_speed).c_str());
        RCLCPP_INFO(this->get_logger(),"\n");
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