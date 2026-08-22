// ============================================================
// include/ZLAC_mapping.hpp
// ============================================================
#ifndef ZLAC_MAPPING_HPP
#define ZLAC_MAPPING_HPP

#include <algorithm>
#include <cmath>
#include <utility>

namespace ylhb_base
{

// 逻辑左右轮转速（车体坐标系）
struct LogicalWheelRpm
{
  double left = 0.0;
  double right = 0.0;
};

// 一台 ZLAC8015D 的两个通道转速
struct ChannelRpm
{
  double low = 0.0;   // 通道1（打包在 0x60FF 低 16 位）
  double high = 0.0;  // 通道2（打包在高 16 位）
};

// 差速运动学：/cmd_vel (m/s, rad/s) <-> 左右轮 RPM
class DifferentialDriveKinematics
{
public:
  DifferentialDriveKinematics(double wheel_radius, double wheel_track,
                              double max_linear_speed, double max_angular_speed)
    : wheel_radius_(wheel_radius), wheel_track_(wheel_track),
      max_linear_speed_(max_linear_speed), max_angular_speed_(max_angular_speed) {}

  // 逆运动学：线速度/角速度 -> 左右轮 RPM
  LogicalWheelRpm twist_to_wheel_rpm(double vx, double wz) const
  {
    const double max_v = max_linear_speed_ > 0 ? max_linear_speed_ : 1.0;
    const double max_w = max_angular_speed_ > 0 ? max_angular_speed_ : 1.0;
    vx = std::clamp(vx, -max_v, max_v);
    wz = std::clamp(wz, -max_w, max_w);

    // 左右轮线速度
    const double v_l = vx - wz * wheel_track_ / 2.0;
    const double v_r = vx + wz * wheel_track_ / 2.0;

    // 线速度 -> 角速度(rad/s) -> RPM
    // v = w * r, w = v / r; RPM = w * 60 / (2*PI)
    const double rpm_l = v_l / wheel_radius_ * 60.0 / (2.0 * M_PI);
    const double rpm_r = v_r / wheel_radius_ * 60.0 / (2.0 * M_PI);

    return LogicalWheelRpm{rpm_l, rpm_r};
  }

  // 正运动学：左右轮 RPM -> (vx, wz)
  std::pair<double, double> wheel_rpm_to_twist(const LogicalWheelRpm& rpm) const
  {
    const double w_l = rpm.left  * 2.0 * M_PI / 60.0;  // rad/s
    const double w_r = rpm.right * 2.0 * M_PI / 60.0;

    const double vx = (w_l + w_r) / 2.0 * wheel_radius_;
    const double wz = (w_r - w_l) / wheel_track_ * wheel_radius_;
    return {vx, wz};
  }

private:
  double wheel_radius_;
  double wheel_track_;
  double max_linear_speed_;
  double max_angular_speed_;
};

// 单台驱动器的通道映射：通道顺序 + 方向系数
class ZlacChannelMapping
{
public:
  ZlacChannelMapping(bool low_is_left, double dir_low, double dir_high)
    : low_is_left_(low_is_left), dir_low_(dir_low), dir_high_(dir_high) {}

  ChannelRpm logical_to_channel(const LogicalWheelRpm& logical) const
  {
    const double low  = (low_is_left_ ? logical.left : logical.right) * dir_low_;
    const double high = (low_is_left_ ? logical.right : logical.left) * dir_high_;
    return ChannelRpm{low, high};
  }

  LogicalWheelRpm channel_to_logical(const ChannelRpm& ch) const
  {
    const double low  = ch.low  / dir_low_;
    const double high = ch.high / dir_high_;

    LogicalWheelRpm out;
    out.left  = low_is_left_ ? low  : high;
    out.right = low_is_left_ ? high : low;
    return out;
  }

private:
  bool low_is_left_;
  double dir_low_;
  double dir_high_;
};

// 前后两台驱动器的整体映射（四轮差速）
class FourDualZlacMapping
{
public:
  FourDualZlacMapping(const ZlacChannelMapping& front, const ZlacChannelMapping& rear)
    : front_(front), rear_(rear) {}

  // 逻辑左右轮 -> 前、后两台驱动器的通道转速
  std::pair<ChannelRpm, ChannelRpm> logical_to_two_drivers(const LogicalWheelRpm& logical) const
  {
    return {front_.logical_to_channel(logical), rear_.logical_to_channel(logical)};
  }

  // 前、后通道转速 -> 逻辑左右轮（前后取平均，消除机械误差）
  LogicalWheelRpm two_drivers_to_logical(const ChannelRpm& front_ch,
                                         const ChannelRpm& rear_ch) const
  {
    const LogicalWheelRpm f = front_.channel_to_logical(front_ch);
    const LogicalWheelRpm r = rear_.channel_to_logical(rear_ch);
    return LogicalWheelRpm{(f.left + r.left) / 2.0, (f.right + r.right) / 2.0};
  }

private:
  ZlacChannelMapping front_;
  ZlacChannelMapping rear_;
};

}  // namespace ylhb_base

#endif  // ZLAC_MAPPING_HPP
