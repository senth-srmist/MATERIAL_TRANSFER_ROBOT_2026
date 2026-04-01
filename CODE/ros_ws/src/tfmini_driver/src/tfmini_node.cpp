#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstring>

class TFminiNode : public rclcpp::Node {
public:
  TFminiNode() : Node("tfmini_node"), serial_fd_(-1) {
    port_ = declare_parameter("port", "/dev/ttyUSB1");
    frame_id_ = declare_parameter("frame_id", "tfmini_link");

    // Pre-populate static message fields (avoid repeated assignment)
    range_msg_.radiation_type = sensor_msgs::msg::Range::INFRARED;
    range_msg_.field_of_view = 0.04f;
    range_msg_.min_range = 0.1f;
    range_msg_.max_range = 12.0f;
    range_msg_.header.frame_id = frame_id_;

    publisher_ = create_publisher<sensor_msgs::msg::Range>(
        "/range_rear",
        rclcpp::SensorDataQoS()); // BEST_EFFORT, suitable for sensors

    if (!openSerial()) {
      RCLCPP_FATAL(get_logger(), "Cannot open serial port %s, shutting down",
                   port_.c_str());
      throw std::runtime_error("Serial port open failed");
    }

    // Timer-based polling instead of blocking loop
    // 10ms = 100Hz check rate, TFmini outputs at ~100Hz
    timer_ = create_wall_timer(std::chrono::milliseconds(10),
                               std::bind(&TFminiNode::timerCallback, this));

    RCLCPP_INFO(get_logger(), "TFmini node started on %s", port_.c_str());
  }

  ~TFminiNode() {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      RCLCPP_DEBUG(get_logger(), "Serial port closed");
    }
  }

private:
  static constexpr size_t FRAME_SIZE = 9;
  static constexpr uint8_t HEADER_BYTE = 0x59;

  std::string port_;
  std::string frame_id_;
  int serial_fd_;

  std::array<uint8_t, FRAME_SIZE * 2>
      ring_buffer_{}; // Small ring for partial frames
  size_t buffer_len_ = 0;

  sensor_msgs::msg::Range range_msg_; // Reuse message object
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool openSerial() {
    serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "open() failed: %s", strerror(errno));
      return false;
    }

    struct termios tty{};
    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr() failed: %s", strerror(errno));
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    cfmakeraw(&tty); // Raw mode, no echo, no canonical processing
    cfsetspeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cc[VMIN] = 0; // Non-blocking
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr() failed: %s", strerror(errno));
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    tcflush(serial_fd_, TCIOFLUSH); // Clear any garbage
    RCLCPP_DEBUG(get_logger(), "Serial port configured successfully");
    return true;
  }

  void timerCallback() {
    if (serial_fd_ < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Serial port not open");
      return;
    }

    // Read available bytes into buffer
    size_t space = ring_buffer_.size() - buffer_len_;
    if (space == 0) {
      // Buffer full but no valid frame found - discard oldest byte
      RCLCPP_DEBUG(get_logger(), "Buffer overflow, discarding data");
      std::memmove(ring_buffer_.data(), ring_buffer_.data() + 1, --buffer_len_);
      space = 1;
    }

    ssize_t n = read(serial_fd_, ring_buffer_.data() + buffer_len_, space);
    if (n < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_ERROR(get_logger(), "read() error: %s", strerror(errno));
      }
      return;
    }
    buffer_len_ += static_cast<size_t>(n);

    // Process complete frames
    processBuffer();
  }

  void processBuffer() {
    while (buffer_len_ >= FRAME_SIZE) {
      // Find header
      if (ring_buffer_[0] != HEADER_BYTE || ring_buffer_[1] != HEADER_BYTE) {
        // Shift buffer to find sync
        auto it = std::find(ring_buffer_.begin() + 1,
                            ring_buffer_.begin() + buffer_len_, HEADER_BYTE);
        size_t skip = std::distance(ring_buffer_.begin(), it);
        if (skip > 0) {
          RCLCPP_DEBUG(get_logger(), "Skipping %zu bytes to resync", skip);
          std::memmove(ring_buffer_.data(), ring_buffer_.data() + skip,
                       buffer_len_ - skip);
          buffer_len_ -= skip;
        }
        continue;
      }

      if (buffer_len_ < FRAME_SIZE)
        break;

      // Validate checksum (sum of bytes 0-7 should equal byte 8, lower 8 bits)
      uint8_t checksum = 0;
      for (size_t i = 0; i < FRAME_SIZE - 1; ++i) {
        checksum += ring_buffer_[i];
      }

      if (checksum != ring_buffer_[8]) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Checksum mismatch, skipping frame");
        std::memmove(ring_buffer_.data(), ring_buffer_.data() + 1,
                     --buffer_len_);
        continue;
      }

      // Parse valid frame
      uint16_t distance_cm = ring_buffer_[2] | (ring_buffer_[3] << 8);
      uint16_t strength = ring_buffer_[4] | (ring_buffer_[5] << 8);

      // Consume frame
      std::memmove(ring_buffer_.data(), ring_buffer_.data() + FRAME_SIZE,
                   buffer_len_ - FRAME_SIZE);
      buffer_len_ -= FRAME_SIZE;

      // Strength < 100 or distance 0 means unreliable reading
      if (strength < 100 || distance_cm == 0) {
        RCLCPP_DEBUG(get_logger(), "Weak signal (strength=%u), skipping",
                     strength);
        continue;
      }

      float distance_m = distance_cm * 0.01f;

      range_msg_.header.stamp = now();
      range_msg_.range = distance_m;
      publisher_->publish(range_msg_);

      RCLCPP_DEBUG(get_logger(), "Distance: %.2f m, strength: %u", distance_m,
                   strength);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<TFminiNode>();
    RCLCPP_INFO(node->get_logger(), "Spinning...");
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("tfmini"), "Exception: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
