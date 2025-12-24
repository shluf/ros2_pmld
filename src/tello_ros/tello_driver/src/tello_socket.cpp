#include "tello_driver_node.hpp"

namespace tello_driver
{

  void TelloSocket::listen()
  {
    thread_ = std::thread(
      [this]()
      {
        for (;;) {
          try {
            size_t r = socket_.receive(asio::buffer(buffer_));
            if (r == 0) {
              // Socket closed or no data, break to allow clean shutdown
              break;
            }
            process_packet(r);
          } catch (const std::exception &e) {
            // Log and break; do not allow exception to crash the process
            if (driver_) {
              RCLCPP_ERROR(driver_->get_logger(), "Socket receive error: %s", e.what());
            }
            break;
          }
        }
      });
  }

  bool TelloSocket::receiving()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return receiving_;
  }

  rclcpp::Time TelloSocket::receive_time()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return receive_time_;
  }

  void TelloSocket::timeout()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    receiving_ = false;
  }

  void TelloSocket::stop()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    try {
      if (socket_.is_open()) {
        socket_.close();
      }
    } catch (const std::exception &e) {
      if (driver_) RCLCPP_WARN(driver_->get_logger(), "Error closing socket: %s", e.what());
    }

    if (thread_.joinable()) {
      try { thread_.join(); } catch (...) {}
    }
    receiving_ = false;
  }

} // namespace tello_driver
