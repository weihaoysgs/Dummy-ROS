#include "stepping_motor_control/socket_can.hpp"
#include "stepping_motor_control/emm42_can.hpp"

namespace stepping_motor_control {


SocketCAN::SocketCAN()
{
  can0_ = std::make_shared<CanInterface>();
  // self define the deconstructor, in order to avoid double free
  can0_->interface_name_ = "can0";
  if (can0_->interface_name_ != "")
  {
    // LOG(INFO) << RM_COUT_BLUE << "Open Can 0" << RM_COUT_TAIL;
    ROS_INFO("Open Can 0");
    this->OpenCanDevice(can0_);
    std::thread can_receive_t =
        std::thread(&SocketCAN::tReceiveCanFrameThread, this,
                    std::ref(can0_->socket_can_fd_));
    can_receive_t.detach();
  }
}

bool SocketCAN::OpenCanDevice(CanInterface::SharedPtr &can)
{
  can->socket_can_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can->socket_can_fd_ == -1)
  {
    ROS_INFO("Error: Unable to create a CAN socket");
    // LOG(FATAL) << "Error: Unable to create a CAN socket";
    return false;
  }
  char name[16] = {};  // avoid stringop-truncation
  strncpy(name, can->interface_name_.c_str(), can->interface_name_.size());
  strncpy(can->interface_request_.ifr_name, name, IFNAMSIZ);
  // Get the index of the network interface
  if (ioctl(can->socket_can_fd_, SIOCGIFINDEX, &can->interface_request_) == -1)
  {
    this->Close(can);
    ROS_INFO("Unable to select CAN interface: %s, I/O control error.", name);
    // LOG(FATAL) << "Unable to select CAN interface:" << name
    //            << " I/O control error";
    // Invalidate unusable socket
    return false;
  }
  // Bind the socket to the network interface
  can->address_.can_family = AF_CAN;
  can->address_.can_ifindex = can->interface_request_.ifr_ifindex;
  int rc = bind(can->socket_can_fd_,
                reinterpret_cast<struct sockaddr *>(&(can->address_)),
                sizeof(can->address_));
  if (rc == -1)
  {
    // LOG(FATAL) << "Failed to bind socket to " << name << " network interface";
    ROS_INFO("Failed to bind socket to %s network interface", name);
    this->Close(can);
    return false;
  }

  // ros::Rate r(1);
  
  // while(ros::ok()) {
  Emm42CanMotor asdasd(1);
  std::unique_ptr<can_frame> asd = asdasd.GetPostionControlCanFrame(-1279, 100, 3200*27);
  int n = write(can0_->socket_can_fd_, asd.get(), sizeof(can_frame));
  // LOG_IF(ERROR, n == -1) << "Send Error";
  ROS_INFO("n %d", n);
  // rclcpp::Clock::sleep_for(rclcpp::Duration());
  // r.sleep();
  // rclcpp::sleep_for(std::chrono::nanoseconds(1000*1000*2));
  // }
  return true;
}

void SocketCAN::Close(CanInterface::SharedPtr &can)
{
  if (!IsOpened(can)) return;
  // Close the file descriptor for our socket
  ::close(can->socket_can_fd_);
  can->socket_can_fd_ = -1;
}

void SocketCAN::tReceiveCanFrameThread(int &socket_can_fd) const
{
  // Holds the set of descriptors, that 'select' shall monitor
  fd_set descriptors;
  // Highest file descriptor in set
  int maxfd = socket_can_fd;
  // How long 'select' shall wait before returning with timeout
  struct timeval timeout
  {
  };
  // Buffer to store incoming frame

  while (ros::ok())
  {
    can_frame rx_frame{};
    timeout.tv_sec = 1.;  // Should be set each loop
    // Clear descriptor set
    FD_ZERO(&descriptors);
    // Add socket descriptor
    FD_SET(socket_can_fd, &descriptors);
    // Wait until timeout or activity on any descriptor
    if (select(maxfd + 1, &descriptors, nullptr, nullptr, &timeout))
    {
      ssize_t len = read(socket_can_fd, &rx_frame, CAN_MTU);
      if (len < 0) continue;
      this->CanFrameReceptionHandler(rx_frame, socket_can_fd);
    }
  }
}

void SocketCAN::CanFrameReceptionHandler(can_frame &frame,
                                         const int &socket_can_fd) const
{
  if (socket_can_fd != can0_->socket_can_fd_) return;
  ROS_INFO("rc : %d", frame.can_id);
}


}  // namespace rm_hardware

// int CanSendReceive::WriteDataToSocketCanDeviceControlMotor(
//     const int &socket_can_fd, const canid_t &can_id, const int16_t int_speed) {
//   can_frame frame;
//   frame.can_id = can_id;
//   frame.can_dlc = 8;
//   frame.data[0] = 0x00;
//   frame.data[1] = 0x00;
//   frame.data[2] = 0x00;
//   frame.data[3] = 0x00;
//   // frame.data[4] = int_speed >> 8;
//   // frame.data[5] = int_speed;
//   frame.data[4] = 0x00;
//   frame.data[5] = 0x00;
//   frame.data[6] = int_speed;
//   frame.data[7] = int_speed >> 8;

//   int n = write(socket_can_fd, &frame, sizeof(frame));
//   LOG_IF(ERROR, n == -1) << "Send Error";
// }