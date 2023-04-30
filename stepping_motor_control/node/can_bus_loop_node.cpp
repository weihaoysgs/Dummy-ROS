#include "stepping_motor_control/socket_can.hpp"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "can_bus_loop_node");
  stepping_motor_control::SocketCAN socket_can;
  ROS_INFO("Can Start Send And Receive!!!");
  ros::spin();
  return 0;
}
