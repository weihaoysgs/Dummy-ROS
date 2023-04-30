#include "stepping_motor_control/socket_can.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stepping_motor_control::SocketCAN>("can_bus_loop_node"));
  rclcpp::shutdown();
}