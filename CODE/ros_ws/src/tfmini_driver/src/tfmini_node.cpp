#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

class TFminiNode : public rclcpp::Node
{
public:
    TFminiNode() : Node("tfmini_node")
    {
        port_ = this->declare_parameter("port", "/dev/ttyUSB0");
        baudrate_ = this->declare_parameter("baudrate", 115200);

        publisher_ = this->create_publisher<sensor_msgs::msg::Range>("/range_rear", 10);

        openSerial();
    }

private:
    std::string port_;
    int baudrate_;
    int serial_fd_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;

    void openSerial()
    {
        serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY);

        if (serial_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
            return;
        }

        struct termios tty;
        memset(&tty, 0, sizeof tty);

        tcgetattr(serial_fd_, &tty);

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;

        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;

        tty.c_cflag &= ~CRTSCTS;

        tcsetattr(serial_fd_, TCSANOW, &tty);

        RCLCPP_INFO(this->get_logger(), "TF Mini connected on %s", port_.c_str());

        readLoop();
    }

    void readLoop()
    {
        uint8_t buffer[9];

        while (rclcpp::ok())
        {
            int n = read(serial_fd_, buffer, 9);

            if (n == 9 && buffer[0] == 0x59 && buffer[1] == 0x59)
            {
                int distance_cm = buffer[2] + buffer[3] * 256;
                float distance_m = distance_cm / 100.0;

                sensor_msgs::msg::Range msg;
                msg.header.stamp = this->now();
                msg.header.frame_id = "tfmini_link";

                msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
                msg.field_of_view = 0.04;
                msg.min_range = 0.1;
                msg.max_range = 12.0;
                msg.range = distance_m;

                publisher_->publish(msg);

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "Distance: %.2f m", distance_m);
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFminiNode>());
    rclcpp::shutdown();
    return 0;
}