#include "rclcpp/rclcpp.hpp"


class PID_Controller_Node: public rclcpp::Node{
public:
    PID_Controller_Node()
        :   Node("pid_controller"),
            K_p(1.0),
            K_d(1.0),
            K_i(1.0),
            error(0.0),
            error_delta(0.0),
            error_sum(0.0),
            equivalent_force(0.0),
            equivalent_torque(0.0){
        RCLCPP_INFO(this->get_logger(),"Node activated");
    }

    float get_PID_value(float error,float error_delta,float error_sum){
        long temp(0);
        temp = K_p*error + K_d*error_delta + K_i*error_sum;

        return temp;
    }

private:
    const float K_p;
    const float K_d;
    const float K_i;
    float error;
    float error_delta;
    float error_sum;
    float equivalent_force;
    float equivalent_torque;
};

int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    
    auto node=std::make_shared<PID_Controller_Node>();
    float pid = node->get_PID_value(2.5,1.5,3.0);
    std::cout << "PID: " << pid << '\n';

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}