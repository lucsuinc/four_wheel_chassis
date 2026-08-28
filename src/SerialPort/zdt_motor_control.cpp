#include "rclcpp/rclcpp.hpp"
#include "SerialPort.hpp"


//声明一个串口对象
SerialPort SerialP;


class motor_control : public rclcpp::Node {

private:

    rclcpp::TimerBase::SharedPtr Serial_send_timer_;
    rclcpp::TimerBase::SharedPtr Serial_receive_timer_;


public:
    motor_control() {
        SerialP.open("/dev/ttyUSB0",115200);
    }


};



int main(int argc,char** argv) {
    rclcpp::init(argc,argv);
    auto s1 = std::make_shared<motor_control>("motor_control");
    rclcpp::spin(s1);
    rclcpp::shutdown();
    return 0;
}