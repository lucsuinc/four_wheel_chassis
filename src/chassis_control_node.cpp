// ============================================================
// src/chassis_control_node.cpp
// 四轮差速底盘控制节点（两个 ZLAC8015D，每个双通道 = 四个轮子）
//
// 输入:  /cmd_vel   (geometry_msgs/Twist)
// 输出:  /odom      (nav_msgs/Odometry) + odom->base_footprint TF
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <utility>

#include "ZLAC8015D.h"
#include "ZLAC_mapping.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class chassis_control_node : public rclcpp::Node
{
public:
  chassis_control_node()
    : Node("chassis_control_node"),
      // 驱动器构造参数：接口名、节点ID、目标速度系数、实际速度系数、通道1是否左轮、SDO超时ms
      // target_unit_per_rpm: 一般 1.0（RPM * 1.0 = 写进 0x60FF 的值）
      // actual_unit_per_rpm: 一般 10.0（0x606C 返回值 / 10 = RPM），以手册为准
      front_wheel(
        this->declare_parameter("can_interface", "can0"),
        static_cast<uint8_t>(this->declare_parameter("front_node_id", 1)),
        1.0, 10.0, true, 100),
      behind_wheel(
        // can_interface 已由 front_wheel 声明，这里直接读取，避免重复声明
        this->get_parameter("can_interface").as_string(),
        static_cast<uint8_t>(this->declare_parameter("rear_node_id", 2)),
        1.0, 10.0, true, 100),
      // 运动学参数从 ROS 参数读取，默认值兜底
      kin_(
        this->declare_parameter("wheel_radius", 0.15),
        this->declare_parameter("wheel_track", 0.5),
        this->declare_parameter("max_linear_speed", 1.0),
        this->declare_parameter("max_angular_speed", 1.2)
      ),
      // 前后驱动器映射：方向系数从参数读，轮子转向不对就改成 -1.0
      four_mapper_(
        ylhb_base::ZlacChannelMapping(true,
          this->declare_parameter("front_dir_low", 1.0),
          this->declare_parameter("front_dir_high", 1.0)),
        ylhb_base::ZlacChannelMapping(true,
          this->declare_parameter("rear_dir_low", 1.0),
          this->declare_parameter("rear_dir_high", 1.0))
      ),
      tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    // ========== 1. 初始化驱动器，带失败校验 ==========
    if (!front_wheel.open() || !behind_wheel.open()) {
      RCLCPP_FATAL(this->get_logger(), "CAN驱动器打开失败！请检查CAN设备和节点ID");
      rclcpp::shutdown();
      return;
    }

    // 使能电机（内部自动设置同步模式+心跳+速度模式+控制字三步）
    // 参数：加速度、减速度（单位：驱动器内部单位，根据实际固件调整）
    if (!front_wheel.enableMotor(100, 100) || !behind_wheel.enableMotor(100, 100)) {
      RCLCPP_FATAL(this->get_logger(), "电机使能失败！请检查驱动器状态");
      rclcpp::shutdown();
      return;
    }

    // ========== 2. ROS 话题订阅发布 ==========
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&chassis_control_node::cmd_vel_callback, this, _1)
    );
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // ========== 3. 定时器 ==========
    can_poll_timer_ = this->create_wall_timer(
      10ms, std::bind(&chassis_control_node::can_poll_timer_callback, this));
    feedback_timer_ = this->create_wall_timer(
      20ms, std::bind(&chassis_control_node::feedback_timer_callback, this));
    fault_timer_ = this->create_wall_timer(
      500ms, std::bind(&chassis_control_node::fault_timer_callback, this));
    status_timer_ = this->create_wall_timer(
      1000ms, std::bind(&chassis_control_node::status_timer_callback, this));

    // 初始化时间戳
    last_cmd_vel_time_ = this->now();
    last_feedback_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "四轮底盘控制节点启动成功！");
    init_ok_ = true;
  }

  // CAN/电机初始化成功后才允许进入 spin，失败时 main 会直接退出
  bool ok() const { return init_ok_; }

  ~chassis_control_node()
  {
    // 析构时自动停电机、关闭CAN（未打开时调用也安全）
    front_wheel.disableMotor();
    behind_wheel.disableMotor();
    front_wheel.close();
    behind_wheel.close();
  }

private:
  bool init_ok_ = false;

  // ===== 硬件驱动对象 =====
  CanopenNode front_wheel;
  CanopenNode behind_wheel;

  // ===== 运动学与通道映射 =====
  ylhb_base::DifferentialDriveKinematics kin_;
  ylhb_base::FourDualZlacMapping four_mapper_;

  // ===== ROS 通信对象 =====
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // ===== 定时器句柄 =====
  rclcpp::TimerBase::SharedPtr can_poll_timer_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::TimerBase::SharedPtr fault_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  // ===== 状态变量 =====
  ylhb_base::LogicalWheelRpm target_logical_rpm_{0.0, 0.0};
  ylhb_base::LogicalWheelRpm actual_logical_rpm_{0.0, 0.0};
  rclcpp::Time last_cmd_vel_time_;
  rclcpp::Time last_feedback_time_;

  // 里程计累计位姿
  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  double odom_yaw_ = 0.0;

  // 超时阈值：500ms 没收到 cmd_vel 自动停车
  static constexpr double CMD_VEL_TIMEOUT_S = 0.5;
  // 心跳超时：1s 没收到驱动器心跳视为离线
  static constexpr double HEARTBEAT_TIMEOUT_S = 1.0;

  // ========== cmd_vel 回调 ==========
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_vel_time_ = this->now();
    target_logical_rpm_ = kin_.twist_to_wheel_rpm(msg->linear.x, msg->angular.z);
  }

  // ========== 10ms 定时器：收帧 + 安全检查 + 下发速度 ==========
  void can_poll_timer_callback()
  {
    // 1. 先把两台驱动器的应答/心跳全收掉（非阻塞，不卡控制循环）
    front_wheel.pollOnce();
    behind_wheel.pollOnce();

    // 2. cmd_vel 超时停车
    const double cmd_dt = (this->now() - last_cmd_vel_time_).seconds();
    if (cmd_dt > CMD_VEL_TIMEOUT_S) {
      target_logical_rpm_.left = 0.0;
      target_logical_rpm_.right = 0.0;
    }

    // 3. 驱动器心跳离线停车
    if (!front_wheel.heartbeatAlive(HEARTBEAT_TIMEOUT_S) ||
        !behind_wheel.heartbeatAlive(HEARTBEAT_TIMEOUT_S)) {
      target_logical_rpm_.left = 0.0;
      target_logical_rpm_.right = 0.0;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "驱动器心跳丢失，已紧急停车！");
    }

    // 4. 逻辑左右转速 -> 两台驱动器的通道转速
    auto [front_ch, rear_ch] = four_mapper_.logical_to_two_drivers(target_logical_rpm_);

    // 5. 下发到两台驱动器（只发不等应答，高频）
    front_wheel.setTargetRpm(front_ch.low, front_ch.high);
    behind_wheel.setTargetRpm(rear_ch.low, rear_ch.high);
  }

  // ========== 20ms 反馈定时器：请求 + 读缓存 + 里程计积分 ==========
  void feedback_timer_callback()
  {
    // 1. 发读请求（下一条 SDO 应答会被 10ms pollOnce 收进缓存）
    front_wheel.requestActualRpm();
    behind_wheel.requestActualRpm();

    // 2. 从缓存读实际转速
    double f_l, f_r, r_l, r_r;
    const bool ok_front = front_wheel.readActualRpm(f_l, f_r);
    const bool ok_rear  = behind_wheel.readActualRpm(r_l, r_r);
    if (!ok_front || !ok_rear) return;

    // 3. 封装为通道结构，还原为统一的逻辑左右轮转速（前后取平均）
    const ylhb_base::ChannelRpm front_chan{f_l, f_r};
    const ylhb_base::ChannelRpm rear_chan{r_l, r_r};
    actual_logical_rpm_ = four_mapper_.two_drivers_to_logical(front_chan, rear_chan);

    // 4. 正运动学：轮速 -> 底盘速度
    auto [vx, wz] = kin_.wheel_rpm_to_twist(actual_logical_rpm_);

    // 5. 时间差 + 欧拉积分更新位姿
    const rclcpp::Time current_time = this->now();
    const double dt = (current_time - last_feedback_time_).seconds();
    last_feedback_time_ = current_time;

    if (dt > 0.0 && dt < 0.1) {
      odom_x_   += vx * std::cos(odom_yaw_) * dt;
      odom_y_   += vx * std::sin(odom_yaw_) * dt;
      odom_yaw_ += wz * dt;
    }

    // 6. 发布 odom 和 TF
    publish_odom(current_time, vx, wz);
    publish_odom_tf(current_time);
  }

  // ========== 发布 odom 消息 ==========
  void publish_odom(const rclcpp::Time& stamp, double vx, double wz)
  {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";

    odom_msg.pose.pose.position.x = odom_x_;
    odom_msg.pose.pose.position.y = odom_y_;
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, odom_yaw_);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.angular.z = wz;

    odom_msg.pose.covariance = {
      0.001, 0, 0, 0, 0, 0,
      0, 0.001, 0, 0, 0, 0,
      0, 0, 0.001, 0, 0, 0,
      0, 0, 0, 0.001, 0, 0,
      0, 0, 0, 0, 0.001, 0,
      0, 0, 0, 0, 0, 0.01
    };
    odom_msg.twist.covariance = odom_msg.pose.covariance;

    odom_pub_->publish(odom_msg);
  }

  // ========== 发布 odom -> base_footprint TF ==========
  void publish_odom_tf(const rclcpp::Time& stamp)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_footprint";

    tf_msg.transform.translation.x = odom_x_;
    tf_msg.transform.translation.y = odom_y_;
    tf_msg.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, odom_yaw_);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);
  }

  // ========== 500ms 故障查询 ==========
  void fault_timer_callback()
  {
    front_wheel.requestFault();
    behind_wheel.requestFault();

    if (front_wheel.hasFault()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "前驱动器故障码: 0x%04lX", static_cast<unsigned long>(front_wheel.faultCode()));
    }
    if (behind_wheel.hasFault()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "后驱动器故障码: 0x%04lX", static_cast<unsigned long>(behind_wheel.faultCode()));
    }
  }

  // ========== 1s 状态日志 ==========
  void status_timer_callback()
  {
    RCLCPP_INFO(this->get_logger(),
      "目标RPM(左/右): %.2f / %.2f | 实际RPM(左/右): %.2f / %.2f | 心跳: %s",
      target_logical_rpm_.left, target_logical_rpm_.right,
      actual_logical_rpm_.left, actual_logical_rpm_.right,
      front_wheel.heartbeatAlive(HEARTBEAT_TIMEOUT_S) &&
      behind_wheel.heartbeatAlive(HEARTBEAT_TIMEOUT_S) ? "正常" : "丢失");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<chassis_control_node>();
  if (!node->ok()) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
