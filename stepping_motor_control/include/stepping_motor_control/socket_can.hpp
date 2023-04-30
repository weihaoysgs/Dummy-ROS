#ifndef __SOCKET_CAN_HPP_
#define __SOCKET_CAN_HPP_

#include <linux/can.h>
#include <net/if.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <utility>

namespace stepping_motor_control {


struct CanInterface
{
  int socket_can_fd_{-1};
  ifreq interface_request_{};
  sockaddr_can address_{};
  std::string interface_name_{};
  using SharedPtr = std::shared_ptr<CanInterface>;
  using ConstSharedPtr = std::shared_ptr<CanInterface const>;
};

class SocketCAN : public rclcpp::Node
{
 public:
  explicit SocketCAN(const std::string &node_name);
  ~SocketCAN() = default;
  inline bool IsOpened(CanInterface::SharedPtr &can) const { return can->socket_can_fd_ != -1; }
  inline bool OpenCanDevice(CanInterface::SharedPtr &can);
  inline void Close(CanInterface::SharedPtr &can);
  void tReceiveCanFrameThread(int &socket_can_fd) const;
  inline void CanFrameReceptionHandler(can_frame &frame,
                                       const int &socket_can_fd) const;
  
  // WriteDataToSocket
  // int CanSend()
 private:
  CanInterface::SharedPtr can0_;
};


}  // namespace stepping_motor_control

#endif  // __SOCKET_CAN_HPP_