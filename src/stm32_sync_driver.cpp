#include <cstdio>
#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class Stm32SyncDriver : public rclcpp::Node {
public:
  Stm32SyncDriver() : Node("stm32_sync_driver") {
    this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    this->declare_parameter<std::string>("frame_id", "imu_link");

    std::string port_name = this->get_parameter("port").as_string();
    fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY);
    
    if (fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Nedá sa otvoriť port %s. Skontroluj práva (sudo chmod 666 %s)", 
                   port_name.c_str(), port_name.c_str());
      return;
    }

    setup_uart();
    
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    
    timer_ = this->create_wall_timer(10ms, std::bind(&Stm32SyncDriver::read_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "STM32 Sync Driver bol úspešne spustený na porte %s", port_name.c_str());
  }

  ~Stm32SyncDriver() { if (fd_ >= 0) close(fd_); }

private:
  void setup_uart() {
    struct termios tty;
    if(tcgetattr(fd_, &tty) != 0) return;

    cfsetispeed(&tty, B38400);
    cfsetospeed(&tty, B38400);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_lflag |= ICANON; 
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); 

    tcflush(fd_, TCIFLUSH);
    tcsetattr(fd_, TCSANOW, &tty);
  }

  void read_callback() {
    char buf[512];
    int n = read(fd_, buf, sizeof(buf) - 1);
    
    if (n > 0) {
      buf[n] = '\0';
      parse_and_publish(std::string(buf));
    }
  }

  void parse_and_publish(const std::string& raw_data) {
    std::stringstream ss(raw_data);
    std::string item;
    std::vector<double> values;

    while (std::getline(ss, item, ',')) {
      try {
        values.push_back(std::stod(item));
      } catch (...) { return; } 
    }

    if (values.size() >= 6) {
      auto msg = sensor_msgs::msg::Imu();
      msg.header.stamp = this->now();
      msg.header.frame_id = this->get_parameter("frame_id").as_string();

      msg.linear_acceleration.x = values[0];
      msg.linear_acceleration.y = values[1];
      msg.linear_acceleration.z = values[2];

      msg.angular_velocity.x = values[3];
      msg.angular_velocity.y = values[4];
      msg.angular_velocity.z = values[5];

      msg.orientation_covariance[0] = -1.0; // Neznáma orientácia

      imu_pub_->publish(msg);
    }
  }

  int fd_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Stm32SyncDriver>());
  rclcpp::shutdown();
  return 0;
}
