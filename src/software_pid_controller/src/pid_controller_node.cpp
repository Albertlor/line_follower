#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>


class PID_Controller_Node: public rclcpp::Node{
public:
    PID_Controller_Node()
        :   Node("pid_controller"),
            dt(0.001),
            linear_Kp(111.111),
            linear_Kd(9.6096),
            linear_Ki(1.11),
            angular_Kp(0.633277),
            angular_Kd(0.05477),
            angular_Ki(0.0006336),
            linear_error(0.1),
            prev_linear_error(0.0),
            linear_error_delta(0.0),
            linear_error_sum(0.0),
            angular_error(0.0),
            prev_angular_error(0.0),
            angular_error_delta(0.0),
            angular_error_sum(0.0),
            equivalent_force(0.0),
            equivalent_torque(0.0){
        subscriber_=this->create_subscription<example_interfaces::msg::String>(
            "error_state",
            10,
            std::bind(&PID_Controller_Node::subscribe_error_state,this,std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),"Node activated");
    }

    float get_equivalent_force(double error,double error_delta,double error_sum){
        equivalent_force = linear_Kp*error + linear_Kd*error_delta/dt + linear_Ki*error_sum*dt;
        return equivalent_force;
    }

    float get_equivalent_torque(double error,double error_delta,double error_sum){
        equivalent_torque = angular_Kp*error + angular_Kd*error_delta/dt + angular_Ki*error_sum*dt;
        return equivalent_torque;
    }

private:
    const float dt;
    const double linear_Kp,linear_Kd,linear_Ki;
    const double angular_Kp,angular_Kd,angular_Ki;
    double linear_error,prev_linear_error,linear_error_delta,linear_error_sum;
    double angular_error,prev_angular_error,angular_error_delta,angular_error_sum;  
    double equivalent_force,equivalent_torque;
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;


    void subscribe_error_state(const example_interfaces::msg::String::SharedPtr msg){
        std::string equivalent_force_msg;
        std::string equivalent_torque_msg;
        angular_error=std::stof(msg->data.c_str());
        angular_error_delta=angular_error-prev_angular_error;
        angular_error_sum+=angular_error;
        equivalent_force_msg=std::to_string(this->get_equivalent_force(linear_error,linear_error_delta,linear_error_sum));
        equivalent_torque_msg=std::to_string(this->get_equivalent_torque(angular_error,angular_error_delta,angular_error_sum));
        prev_linear_error=linear_error;
        prev_angular_error=angular_error;
        RCLCPP_INFO(this->get_logger(),"Equivalent force: %s",equivalent_force_msg.c_str());
        RCLCPP_INFO(this->get_logger(),"Equivalent torque: %s",equivalent_torque_msg.c_str());
    }
};

int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    auto node=std::make_shared<PID_Controller_Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}