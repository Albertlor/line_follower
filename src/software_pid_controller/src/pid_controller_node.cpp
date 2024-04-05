#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>
#include <math.h>
#include <vector>


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
            equivalent_torque(0.0),
            right_torque(0.0),
            left_torque(0.0),
            transform_matrix{{0.035,0.5},{0.035,-0.5}}{
        subscriber_=this->create_subscription<example_interfaces::msg::String>(
            "error_state",
            10,
            std::bind(&PID_Controller_Node::subscribe_error_state,this,std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),"Node activated");
    }

    float get_linear_error_factor(float error){
        if(error<M_PI/8){
            return 1.0;
        }
        else if(error>M_PI/8 && error<M_PI/7){
            return 0.9;
        }
        else if(error>M_PI/7 && error<M_PI/6){
            return 0.8;
        }
        else if(error>M_PI/6 && error<M_PI/5){
            return 0.7;
        }
        else if(error>M_PI/5 && error<M_PI/4){
            return 0.6;
        }
        else{
            return 0.5;
        }
    }

    void get_equivalent_force(float error,float error_delta,float error_sum){
        this->equivalent_force = this->linear_Kp*error + this->linear_Kd*error_delta/dt + this->linear_Ki*error_sum*dt;
    }

    void get_equivalent_torque(float error,float error_delta,float error_sum){
        this->equivalent_torque = this->angular_Kp*error + this->angular_Kd*error_delta/dt + this->angular_Ki*error_sum*dt;
    }

private:
    const float dt;
    const float linear_Kp,linear_Kd,linear_Ki;
    const float angular_Kp,angular_Kd,angular_Ki;
    float linear_error,prev_linear_error,linear_error_delta,linear_error_sum;
    float angular_error,prev_angular_error,angular_error_delta,angular_error_sum;  
    float equivalent_force,equivalent_torque;
    float right_torque,left_torque;
    std::vector<std::vector<float>> transform_matrix;
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;


    void subscribe_error_state(const example_interfaces::msg::String::SharedPtr msg){
        std::string right_torque_msg;
        std::string left_torque_msg;
        float linear_error_factor;

        linear_error_factor=this->get_linear_error_factor(this->angular_error);
        this->linear_error_delta=this->linear_error*linear_error_factor-this->prev_linear_error;
        this->linear_error_sum+=(this->linear_error*linear_error_factor);

        this->angular_error=std::stof(msg->data.c_str());
        this->angular_error_delta=this->angular_error-this->prev_angular_error;
        this->angular_error_sum+=this->angular_error;

        this->get_equivalent_force(this->linear_error*linear_error_factor,this->linear_error_delta,this->linear_error_sum);
        this->get_equivalent_torque(this->angular_error,this->angular_error_delta,this->angular_error_sum);

        this->right_torque=this->transform_matrix[0][0]*this->equivalent_force + this->transform_matrix[0][1]*this->equivalent_torque;
        this->left_torque=this->transform_matrix[1][0]*this->equivalent_force + this->transform_matrix[1][1]*this->equivalent_torque;

        right_torque_msg=std::to_string(this->right_torque);
        left_torque_msg=std::to_string(this->left_torque);

        this->prev_linear_error=this->linear_error;
        this->prev_angular_error=this->angular_error;
        RCLCPP_INFO(this->get_logger(),"Linear Error: %s",std::to_string(linear_error).c_str());
        RCLCPP_INFO(this->get_logger(),"Angular Error: %s",std::to_string(angular_error).c_str());
        RCLCPP_INFO(this->get_logger(),"Linear Error delta: %s",std::to_string(linear_error_delta).c_str());
        RCLCPP_INFO(this->get_logger(),"Angular Error delta: %s",std::to_string(angular_error_delta).c_str());
        RCLCPP_INFO(this->get_logger(),"Linear Error sum: %s",std::to_string(linear_error_sum).c_str());
        RCLCPP_INFO(this->get_logger(),"Angular Error sum: %s",std::to_string(angular_error_sum).c_str());
        RCLCPP_INFO(this->get_logger(),"Equivalent Force: %s",std::to_string(equivalent_force).c_str());
        RCLCPP_INFO(this->get_logger(),"Equivalent Torque: %s",std::to_string(equivalent_torque).c_str());
        RCLCPP_INFO(this->get_logger(),"Right Torque: %s",right_torque_msg.c_str());
        RCLCPP_INFO(this->get_logger(),"Left Torque: %s",left_torque_msg.c_str());
    }
};

int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    auto node=std::make_shared<PID_Controller_Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}