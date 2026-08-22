// ============================================================
// src/canopen_node.cpp
// ZLAC8015D 驱动器 CANopen 收发实现（SocketCAN + SDO）
// ============================================================

#include "ZLAC8015D.h"

#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>

namespace {

// ZLAC8015D 对象字典（换别的 CANopen 驱动器先核对手册！）
constexpr uint16_t kVendorControlMode    = 0x200F;  // 设为同步模式（1=同步）
constexpr uint16_t kHeartbeatProducer    = 0x1017;  // 心跳产生时间（ms）
constexpr uint16_t kFaultCode            = 0x603F;  // 故障码
constexpr uint16_t kControlWord          = 0x6040;  // 控制字
constexpr uint16_t kStatusWord           = 0x6041;  // 状态字
constexpr uint16_t kModesOfOperation     = 0x6060;  // 运行模式（3=速度模式）
constexpr uint16_t kActualVelocity       = 0x606C;  // 实际速度（双通道打包）
constexpr uint16_t kTargetVelocity       = 0x60FF;  // 目标速度（双通道打包）
constexpr uint16_t kProfileAcceleration  = 0x6083;  // 加速度
constexpr uint16_t kProfileDeceleration  = 0x6084;  // 减速度

// CANopen SDO 命令字
constexpr uint8_t kSdoWrite1B = 0x2F;   // 写 1 字节
constexpr uint8_t kSdoWrite2B = 0x2B;   // 写 2 字节
constexpr uint8_t kSdoWrite4B = 0x23;   // 写 4 字节
constexpr uint8_t kSdoReadReq = 0x40;   // 读请求
constexpr uint8_t kSdoAbort   = 0x80;   // 出错应答

}  // namespace

// ===================== CanSocket =====================

bool CanSocket::open(const std::string& ifname)
{
  close();

  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) return false;

  // 通过接口名（can0）找到内核里的 ifindex
  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    close();
    return false;
  }

  // 绑定到 can0 这个接口
  struct sockaddr_can addr {};
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    close();
    return false;
  }
  return true;
}

void CanSocket::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool CanSocket::send(uint32_t id, const uint8_t* data, uint8_t len)
{
  if (fd_ < 0 || len > 8) return false;

  struct can_frame frame {};
  frame.can_id  = id & CAN_SFF_MASK;   // 只发标准帧（11 位）
  frame.can_dlc = len;
  if (data != nullptr && len > 0) {
    std::memcpy(frame.data, data, len);
  }
  return send(frame);
}

bool CanSocket::send(const struct can_frame& frame)
{
  if (fd_ < 0) return false;
  const ssize_t n = ::write(fd_, &frame, sizeof(frame));
  return n == static_cast<ssize_t>(sizeof(frame));
}

bool CanSocket::receive(struct can_frame& frame, int timeout_ms)
{
  if (fd_ < 0) return false;

  // poll 等数据：timeout_ms <= 0 时立即返回（非阻塞）
  struct pollfd pfd {};
  pfd.fd     = fd_;
  pfd.events = POLLIN;

  const int ret = ::poll(&pfd, 1, timeout_ms);
  if (ret <= 0) return false;
  if (!(pfd.revents & POLLIN)) return false;

  const ssize_t n = ::read(fd_, &frame, sizeof(frame));
  return n == static_cast<ssize_t>(sizeof(frame));
}

// ===================== CanopenNode =====================

CanopenNode::CanopenNode(const std::string& ifname, uint8_t node_id,
                         double target_unit_per_rpm, double actual_unit_per_rpm,
                         bool low_channel_is_left, int sdo_timeout_ms)
  : ifname_(ifname),
    node_id_(node_id),
    target_unit_per_rpm_(target_unit_per_rpm),
    actual_unit_per_rpm_(actual_unit_per_rpm),
    low_channel_is_left_(low_channel_is_left),
    sdo_timeout_ms_(sdo_timeout_ms) {}

bool CanopenNode::open()  { return can_.open(ifname_); }
void CanopenNode::close() { can_.close(); }
bool CanopenNode::isOpen() const { return can_.isOpen(); }

// ---------- NMT：COB-ID 0x000，data[0]=命令，data[1]=节点号 ----------
bool CanopenNode::sendNmt(uint8_t command)
{
  uint8_t data[2] = {command, node_id_};
  return can_.send(0x000, data, 2);
}

bool CanopenNode::nmtStart()               { return sendNmt(0x01); }
bool CanopenNode::nmtStop()                { return sendNmt(0x02); }
bool CanopenNode::nmtEnterPreOperational() { return sendNmt(0x80); }
bool CanopenNode::nmtResetNode()           { return sendNmt(0x81); }

// ---------- SDO 写：COB-ID 0x600+节点号 ----------
// 布局: [命令字] [索引低] [索引高] [子索引] [数据低...高]
bool CanopenNode::sendSdoWrite(uint16_t index, uint8_t sub,
                               uint8_t command, uint32_t value)
{
  struct can_frame frame {};
  frame.can_id  = 0x600 + node_id_;
  frame.can_dlc = 8;
  frame.data[0] = command;
  frame.data[1] = index & 0xFF;
  frame.data[2] = (index >> 8) & 0xFF;
  frame.data[3] = sub;
  frame.data[4] = value & 0xFF;
  frame.data[5] = (value >> 8) & 0xFF;
  frame.data[6] = (value >> 16) & 0xFF;
  frame.data[7] = (value >> 24) & 0xFF;
  return can_.send(frame);
}

// 写 + 等应答（初始化/使能用，保证驱动器真的收到）
bool CanopenNode::sdoWrite8(uint16_t index, uint8_t sub, uint8_t value)
{
  if (!sendSdoWrite(index, sub, kSdoWrite1B, value)) return false;
  uint32_t ack;
  return waitForSdoResponse(index, sub, ack, sdo_timeout_ms_);
}

bool CanopenNode::sdoWrite16(uint16_t index, uint8_t sub, uint16_t value)
{
  if (!sendSdoWrite(index, sub, kSdoWrite2B, value)) return false;
  uint32_t ack;
  return waitForSdoResponse(index, sub, ack, sdo_timeout_ms_);
}

bool CanopenNode::sdoWrite32(uint16_t index, uint8_t sub, uint32_t value)
{
  if (!sendSdoWrite(index, sub, kSdoWrite4B, value)) return false;
  uint32_t ack;
  return waitForSdoResponse(index, sub, ack, sdo_timeout_ms_);
}

// ---------- SDO 读请求：只发，不等 ----------
bool CanopenNode::sendReadRequest(uint16_t index, uint8_t sub)
{
  struct can_frame frame {};
  frame.can_id  = 0x600 + node_id_;
  frame.can_dlc = 8;
  frame.data[0] = kSdoReadReq;
  frame.data[1] = index & 0xFF;
  frame.data[2] = (index >> 8) & 0xFF;
  frame.data[3] = sub;
  return can_.send(frame);
}

// ---------- SDO 读：发请求 + 阻塞等应答（一次性用）----------
bool CanopenNode::sdoRead(uint16_t index, uint8_t sub, uint32_t& value)
{
  if (!sendReadRequest(index, sub)) return false;
  return waitForSdoResponse(index, sub, value, sdo_timeout_ms_);
}

// 主动读请求（高频用）：结果由 pollOnce 收进缓存
bool CanopenNode::requestActualRpm() { return sendReadRequest(kActualVelocity, 0x03); }
bool CanopenNode::requestStatus()    { return sendReadRequest(kStatusWord, 0x00); }
bool CanopenNode::requestFault()     { return sendReadRequest(kFaultCode, 0x00); }

// 等应答：只认 0x580+节点号 且索引/子索引匹配的帧
// 期间收到心跳帧顺手更新心跳状态，不打断等待
bool CanopenNode::waitForSdoResponse(uint16_t index, uint8_t sub,
                                     uint32_t& value, int timeout_ms)
{
  const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::milliseconds(timeout_ms);

  struct can_frame frame {};
  while (true) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now()).count();
    if (remaining <= 0) return false;  // 超时

    if (!can_.receive(frame, static_cast<int>(remaining))) return false;

    const canid_t cob = frame.can_id & CAN_SFF_MASK;

    // 心跳帧：更新状态，继续等 SDO 应答
    if (cob == 0x700 + node_id_) {
      handleHeartbeat(frame.can_dlc > 0 ? frame.data[0] : 0);
      continue;
    }

    // 不是本节点的 SDO 应答，忽略
    if (cob != 0x580 + node_id_ || frame.can_dlc < 8) continue;

    // 0x80 = 对方拒绝（索引不存在/写保护等）
    if (frame.data[0] == kSdoAbort) return false;

    const uint16_t rsp_index = frame.data[1] | (frame.data[2] << 8);
    const uint8_t  rsp_sub   = frame.data[3];
    if (rsp_index != index || rsp_sub != sub) continue;  // 别的 SDO，跳过

    value = frame.data[4] | (frame.data[5] << 8)
          | (frame.data[6] << 16) | (frame.data[7] << 24);
    return true;
  }
}

// ---------- 收到心跳帧 ----------
void CanopenNode::handleHeartbeat(uint8_t state)
{
  heartbeat_state_   = state;
  heartbeat_seen_    = true;
  last_heartbeat_tp_ = std::chrono::steady_clock::now();
}

// ---------- 收到 SDO 应答：按索引/子索引解析进缓存 ----------
void CanopenNode::handleSdoResponse(const struct can_frame& frame)
{
  if (frame.can_dlc < 8 || frame.data[0] == kSdoAbort) return;

  const uint16_t idx = frame.data[1] | (frame.data[2] << 8);
  const uint8_t  sub = frame.data[3];
  const uint32_t val = frame.data[4] | (frame.data[5] << 8)
                     | (frame.data[6] << 16) | (frame.data[7] << 24);

  if (idx == kActualVelocity && sub == 0x03) {
    // 低16位=通道1，高16位=通道2，原始值 / 系数 = RPM
    const int16_t low  = static_cast<int16_t>(val & 0xFFFF);
    const int16_t high = static_cast<int16_t>((val >> 16) & 0xFFFF);

    cached_left_rpm_  = (low_channel_is_left_ ? low : high) / actual_unit_per_rpm_;
    cached_right_rpm_ = (low_channel_is_left_ ? high : low) / actual_unit_per_rpm_;
    actual_rpm_valid_ = true;
    last_actual_rpm_tp_ = std::chrono::steady_clock::now();
  } else if (idx == kStatusWord && sub == 0x00) {
    status_word_ = static_cast<uint16_t>(val);
  } else if (idx == kFaultCode && sub == 0x00) {
    fault_code_ = val;
  }
  // 其它索引（比如速度写应答 0x60FF）不需要缓存，直接忽略
}

// ---------- 非阻塞收帧：把缓冲区里属于本节点的帧全部处理掉 ----------
// 必须在高频定时器里调用！否则速度写应答会占满 socket 缓冲区
void CanopenNode::pollOnce()
{
  struct can_frame frame {};
  while (can_.receive(frame, 0)) {
    const canid_t cob = frame.can_id & CAN_SFF_MASK;
    if (cob == 0x700 + node_id_) {
      handleHeartbeat(frame.can_dlc > 0 ? frame.data[0] : 0);
    } else if (cob == 0x580 + node_id_) {
      handleSdoResponse(frame);
    }
    // 其它 COB-ID 暂时忽略
  }
}

// ---------- 同步模式：0x200F = 1 ----------
bool CanopenNode::setSyncMode()
{
  return sdoWrite16(kVendorControlMode, 0x00, 0x0001);
}

// ---------- 加减速 ----------
// 0x6083:01/02 = 通道1/2 加速度，0x6084:01/02 = 通道1/2 减速度
// 存在 RAM 里，断电丢失，每次上电要重设
bool CanopenNode::setAcceleration(int32_t accel)
{
  return sdoWrite32(kProfileAcceleration, 0x01, static_cast<uint32_t>(accel))
      && sdoWrite32(kProfileAcceleration, 0x02, static_cast<uint32_t>(accel));
}

bool CanopenNode::setDeceleration(int32_t decel)
{
  return sdoWrite32(kProfileDeceleration, 0x01, static_cast<uint32_t>(decel))
      && sdoWrite32(kProfileDeceleration, 0x02, static_cast<uint32_t>(decel));
}

bool CanopenNode::setAccelDecel(int32_t accel, int32_t decel)
{
  return setAcceleration(accel) && setDeceleration(decel);
}

// ---------- 电机使能 ----------
// 顺序：
//   1. 0x200F=1 开同步控制（不同步时 0x60FF:03 不生效）
//   2. 0x1017=50 配置 50ms 心跳（不配就收不到心跳，安全停车会误触发）
//   3. 0x6060=3 速度模式
//   4. 控制字三步：0006(SHUTDOWN) -> 0007(SWITCH_ON) -> 000F(OPERATION_ENABLE)
//   5. NMT Start 进入运行态，心跳开始发
bool CanopenNode::enableMotor()
{
  if (!setSyncMode()) return false;
  if (!sdoWrite16(kHeartbeatProducer, 0x00, 50)) return false;
  if (!sdoWrite8(kModesOfOperation, 0x00, 3)) return false;
  if (!sdoWrite16(kControlWord, 0x00, 0x0006)) return false;
  if (!sdoWrite16(kControlWord, 0x00, 0x0007)) return false;
  if (!sdoWrite16(kControlWord, 0x00, 0x000F)) return false;
  return nmtStart();
}

// 带加减速的使能：先配斜坡，再三步使能
bool CanopenNode::enableMotor(int32_t accel, int32_t decel)
{
  return setAccelDecel(accel, decel) && enableMotor();
}

bool CanopenNode::disableMotor()
{
  return sdoWrite16(kControlWord, 0x00, 0x0000);
}

bool CanopenNode::clearFault()
{
  return sdoWrite16(kControlWord, 0x00, 0x0080);
}

// 保存参数到 EEPROM（0x2010 = 1），掉电后同步模式/心跳配置还在
bool CanopenNode::saveParameters()
{
  return sdoWrite8(0x2010, 0x00, 0x01);
}

// ---------- 目标速度：0x60FF:03，低16位=通道1，高16位=通道2 ----------
// 只发不等应答：速度指令 100Hz 高频发，等应答会阻塞控制循环
bool CanopenNode::setTargetRpm(double left_rpm, double right_rpm)
{
  const double low_rpm  = low_channel_is_left_ ? left_rpm  : right_rpm;
  const double high_rpm = low_channel_is_left_ ? right_rpm : left_rpm;

  const int16_t low  = clampToInt16(low_rpm  * target_unit_per_rpm_);
  const int16_t high = clampToInt16(high_rpm * target_unit_per_rpm_);

  // 两个 int16 打包进一个 int32：低16位=通道1，高16位=通道2
  const uint32_t packed = static_cast<uint32_t>(static_cast<uint16_t>(low))
                        | (static_cast<uint32_t>(static_cast<uint16_t>(high)) << 16);
  return sendSdoWrite(kTargetVelocity, 0x03, kSdoWrite4B, packed);
}

// ---------- 实际速度：读缓存（由 requestActualRpm + pollOnce 更新）----------
bool CanopenNode::readActualRpm(double& left_rpm, double& right_rpm)
{
  if (!actual_rpm_valid_) return false;

  // 缓存太旧就算失败（默认按 5 个 SDO 超时算）
  const auto age = std::chrono::steady_clock::now() - last_actual_rpm_tp_;
  if (age > std::chrono::milliseconds(sdo_timeout_ms_ * 5)) return false;

  left_rpm  = cached_left_rpm_;
  right_rpm = cached_right_rpm_;
  return true;
}

// ---------- 心跳 ----------
bool CanopenNode::heartbeatAlive(double timeout_sec) const
{
  if (!heartbeat_seen_) return false;
  const auto age = std::chrono::steady_clock::now() - last_heartbeat_tp_;
  return age < std::chrono::milliseconds(
                   static_cast<int64_t>(timeout_sec * 1000.0));
}

int16_t CanopenNode::clampToInt16(double v) const
{
  const double rounded = std::round(v);
  if (rounded > std::numeric_limits<int16_t>::max()) return std::numeric_limits<int16_t>::max();
  if (rounded < std::numeric_limits<int16_t>::min()) return std::numeric_limits<int16_t>::min();
  return static_cast<int16_t>(rounded);
}
