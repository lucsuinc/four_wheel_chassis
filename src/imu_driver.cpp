// ============================================================
// src/imu_driver.cpp
// WHEELTEC N300Pro (HiPNUC 超核协议) 串口 IMU 驱动
//
// 功能：
//   1. 打开串口（默认 /dev/robot_imu，波特率默认 115200）
//   2. 逐字节喂给 hipnuc_dec.c 的 HiPNUC 解码器
//   3. 收到完整 0x91 数据包后发布 sensor_msgs/Imu 到 /imu/data
//
// 协议要点（详见 hipnuc_dec.c）：
//   帧头 0x5A 0xA5，6 字节头（含 2 字节长度），CRC16 校验
//   0x91 包内容：
//     acc[3] 单位 g   -> 乘 9.8 得到 m/s^2
//     gyr[3] 单位 度/秒 -> 乘 PI/180 得到 rad/s
//     quat[4] 顺序 [w, x, y, z]，可直接填进 Imu.orientation
//
// 坐标说明：
//   官方 SDK 直接按传感器自身坐标系发布，不做任何轴交换。
//   如果装车后前进/转向方向不对，需要把 IMU 转 90 度安装，
//   或者在下面 publishImu() 里手动交换 x/y 轴（见注释）。
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <string>

// HiPNUC 官方解码器（C 语言），hipnuc.h 内部已经用 extern "C" 包好了
#include "hipnuc.h"

namespace
{
constexpr double kGravity = 9.8;                     // g -> m/s^2
constexpr double kDegToRad = 0.017453292519943295;   // 度 -> 弧度
}  // namespace

class IMUDriver : public rclcpp::Node
{
public:
  IMUDriver()
    : Node("imu_driver")
  {
    // ------- 参数 -------
    // serial_port 可以通过 launch 参数覆盖（例如 imu_port:=/dev/ttyUSB0）
    this->declare_parameter<std::string>("serial_port", "/dev/robot_imu");
    this->declare_parameter<std::string>("frame_id", "imu_link");
    // N300Pro 支持 115200 / 460800 / 921600
    this->declare_parameter<int>("baud_rate", 115200);
    this->get_parameter("serial_port", serial_port_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("baud_rate", baud_rate_);

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

    if (!openSerial()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "打开串口 %s 失败，请检查设备是否插入、权限和 udev 别名",
        serial_port_.c_str());
      return;
    }

    // 2ms 轮询一次：N300Pro 最高 200Hz 输出也来得及
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(2),
      std::bind(&IMUDriver::pollSerial, this));

    RCLCPP_INFO(
      this->get_logger(),
      "N300Pro IMU 驱动启动成功：%s @ %d baud -> /imu/data (frame=%s)",
      serial_port_.c_str(), baud_rate_, frame_id_.c_str());
  }

  ~IMUDriver()
  {
    if (serial_fd_ > 0) close(serial_fd_);
  }

private:
  std::string serial_port_;
  std::string frame_id_;
  int baud_rate_ = 115200;
  int serial_fd_ = -1;

  // HiPNUC 解码器状态（每字节喂一次，内部自动找帧头/算 CRC）
  hipnuc_raw_t raw_{};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 打开串口：8N1、非阻塞原始模式（和官方 SDK 一致）
  bool openSerial()
  {
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ == -1) return false;

    struct termios options;
    memset(&options, 0, sizeof(options));
    tcgetattr(serial_fd_, &options);

    speed_t speed;
    switch (baud_rate_) {
      case 460800: speed = B460800; break;
      case 921600: speed = B921600; break;
      default:     speed = B115200; break;
    }
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag &= ~PARENB;    // 无校验
    options.c_cflag &= ~CSTOPB;    // 1 停止位
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;        // 8 数据位
    options.c_cflag &= ~CRTSCTS;   // 无硬件流控
    options.c_cflag |= CREAD | CLOCAL;

    options.c_iflag &= ~(IXON | IXOFF | IXANY);  // 无软件流控
    options.c_iflag &= ~(INLCR | ICRNL);         // 不转换换行符

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // 原始模式
    options.c_oflag &= ~OPOST;

    options.c_cc[VMIN] = 0;        // 非阻塞读
    options.c_cc[VTIME] = 0;
    tcsetattr(serial_fd_, TCSANOW, &options);
    tcflush(serial_fd_, TCIOFLUSH);
    return true;
  }

  // 定时器回调：非阻塞读串口，逐字节喂解码器
  void pollSerial()
  {
    if (serial_fd_ < 0) return;

    uint8_t buf[256];
    const int n = read(serial_fd_, buf, sizeof(buf));
    if (n <= 0) return;

    for (int i = 0; i < n; ++i) {
      const int ret = hipnuc_input(&raw_, buf[i]);
      // ret > 0 = 收到一帧完整数据且 CRC 正确
      if (ret > 0 && raw_.hi91.tag == 0x91) {
        publishImu();
      }
    }
  }

  // 把 0x91 数据包填进 sensor_msgs/Imu 并发布
  void publishImu()
  {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;

    // 四元数（官方顺序：w, x, y, z）
    msg.orientation.w = raw_.hi91.quat[0];
    msg.orientation.x = raw_.hi91.quat[1];
    msg.orientation.y = raw_.hi91.quat[2];
    msg.orientation.z = raw_.hi91.quat[3];

    // 角速度：度/秒 -> 弧度/秒
    msg.angular_velocity.x = raw_.hi91.gyr[0] * kDegToRad;
    msg.angular_velocity.y = raw_.hi91.gyr[1] * kDegToRad;
    msg.angular_velocity.z = raw_.hi91.gyr[2] * kDegToRad;

    // 线加速度：g -> m/s^2
    msg.linear_acceleration.x = raw_.hi91.acc[0] * kGravity;
    msg.linear_acceleration.y = raw_.hi91.acc[1] * kGravity;
    msg.linear_acceleration.z = raw_.hi91.acc[2] * kGravity;

    // 协方差不能全 0，EKF 靠它评估数据置信度
    msg.orientation_covariance = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};
    msg.angular_velocity_covariance = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};
    msg.linear_acceleration_covariance = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};

    // 如果装车后发现前进方向不对（传感器 X 与车头不一致），
    // 可在此交换 x/y 并取反，例如：
    //   msg.linear_acceleration.x = raw_.hi91.acc[1] * kGravity;
    //   msg.linear_acceleration.y = -raw_.hi91.acc[0] * kGravity;
    // 角速度/四元数也要做同样的轴交换，需要写转换函数。

    imu_pub_->publish(msg);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IMUDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
