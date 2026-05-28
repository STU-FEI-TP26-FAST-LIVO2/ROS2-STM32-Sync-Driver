#include <cstdio>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class Stm32SyncDriver : public rclcpp::Node {
public:
    Stm32SyncDriver() : Node("stm32_sync_driver") {
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0"); //ttyACM0, ttyUSB0
        this->declare_parameter<std::string>("frame_id", "imu_link");

        std::string port_name = this->get_parameter("port").as_string();
        fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY);
        
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Nedá sa otvoriť port %s.", port_name.c_str());
            return;
        }

        setup_uart();
        
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SystemDefaultsQoS());
        timer_ = this->create_wall_timer(5ms, std::bind(&Stm32SyncDriver::read_callback, this));
        
        tcflush(fd_, TCIOFLUSH);
        
        RCLCPP_INFO(this->get_logger(), "STM32 Driver spustený");
    }

private:
    void setup_uart() {
        struct termios tty;
        if(tcgetattr(fd_, &tty) != 0) return;
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNBRK);
        tty.c_oflag &= ~OPOST;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1; 
        tcflush(fd_, TCIFLUSH);
        tcsetattr(fd_, TCSANOW, &tty);
    }

    void read_callback() {
    uint8_t buf[1024];
    int n = read(fd_, buf, sizeof(buf));
    if (n <= 0) return;

    binary_buffer_.insert(binary_buffer_.end(), buf, buf + n);

    while (binary_buffer_.size() >= 20) {
        // Ak sú na konci (index 18 a 19) značky, znamená to, že index 0 je ZAČIATOK správy
        if (binary_buffer_[18] == '\n' && binary_buffer_[19] == '\0') {
            parse_and_publish_binary(binary_buffer_.data());
            binary_buffer_.erase(binary_buffer_.begin(), binary_buffer_.begin() + 20);
        } 
        else {
            // Ak na konci nie sú značky, znamená to, že to, čo považujeme za začiatok (index 0), začiatok nie je. Zahodíme teda jeden bajt a celé sa to posunie.
            binary_buffer_.erase(binary_buffer_.begin());
        }
    }
}

    void parse_and_publish_binary(const uint8_t* data) {
        auto to_int16 = [](uint8_t msb, uint8_t lsb) {
            return static_cast<int16_t>((msb << 8) | lsb);
        };

        //countery
        uint16_t major = data[14] | (data[15] << 8);
        uint16_t minor = data[16] | (data[17] << 8);

        // KONSTANTY ICM40609D vyplyvajuce z kodu
        const double accel_lsb_per_g = 8192.0; 
        const double gyro_lsb_per_dps = 16.384;

        const double g_to_ms2 = 9.80665;
        const double dps_to_rads = M_PI / 180.0;

        auto msg = sensor_msgs::msg::Imu();
        uint32_t real_seconds = static_cast<uint32_t>(major) + 1678364100;
        msg.header.stamp.sec = static_cast<int32_t>(real_seconds);

        double nsec_calc = (static_cast<double>(minor) / 60000.0) * 1e9;
        msg.header.stamp.nanosec = static_cast<uint32_t>(nsec_calc);

        //msg.header.stamp = this->now(); // zatial ros2 cas
        msg.header.frame_id = this->get_parameter("frame_id").as_string();

        msg.linear_acceleration.x = (to_int16(data[2], data[3]) / accel_lsb_per_g)*g_to_ms2;
        msg.linear_acceleration.y = (to_int16(data[4], data[5]) / accel_lsb_per_g)*g_to_ms2;
        msg.linear_acceleration.z = (to_int16(data[6], data[7]) / accel_lsb_per_g)*g_to_ms2;

        msg.angular_velocity.x = dps_to_rads*(to_int16(data[8], data[9]) / gyro_lsb_per_dps) ;
        msg.angular_velocity.y = dps_to_rads*(to_int16(data[10], data[11]) / gyro_lsb_per_dps);
        msg.angular_velocity.z = dps_to_rads*(to_int16(data[12], data[13]) / gyro_lsb_per_dps) ;

        imu_pub_->publish(msg);
        
        //RCLCPP_INFO(this->get_logger(), "Major: %u, Minor: %u", major, minor);
    }

    int fd_;
    std::vector<uint8_t> binary_buffer_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Stm32SyncDriver>());
    rclcpp::shutdown();
    return 0;
}
