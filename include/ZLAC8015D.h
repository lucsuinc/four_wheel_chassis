// ============================================================
// include/ZLAC8015D.h
// ============================================================
#ifndef ZLAC8015D_H
#define ZLAC8015D_H

#include <linux/can.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

// ============================================================
// CanSocket：Linux SocketCAN 收发封装
// 一个 CanSocket 对应一块 USB-CAN 设备（can0 / can1）
// ============================================================
class CanSocket
{
public:
  CanSocket() : fd_(-1) {}
  ~CanSocket() { close(); }

  bool open(const std::string& ifname);
  void close();
  bool isOpen() const { return fd_ >= 0; }

  // 发送一帧（最常用的重载）：
  //   id   - 11 位标准帧 ID（CANopen 里就是 COB-ID，如 0x600 | 1）
  //   data - 数据指针，最多 8 字节
  //   len  - 数据长度（0~8），超过 8 会被截断
  bool send(uint32_t id, const uint8_t* data, uint8_t len);

  // 底层发送：直接传内核的 can_frame 结构体
  bool send(const struct can_frame& frame);

  // 接收一帧：timeout_ms <= 0 表示非阻塞（立即返回）
  bool receive(struct can_frame& frame, int timeout_ms);

  // 禁止拷贝：fd 是系统资源，复制对象会导致两个对象指向同一个 fd，
  // 析构时重复 close 会崩溃
  CanSocket(const CanSocket&) = delete;
  CanSocket& operator=(const CanSocket&) = delete;

private:
  int fd_;
};

// ============================================================
// CanopenNode：一台 ZLAC8015D 驱动器的 CANopen 封装
// ============================================================
class CanopenNode
{
public:
  // 构造函数参数：
  //   ifname              - CAN 接口名（can0 / can1）
  //   node_id             - 驱动器拨码节点号（1~127）
  //   target_unit_per_rpm - 目标速度系数，RPM * 系数 = 写进 0x60FF 的值（一般 1.0）
  //   actual_unit_per_rpm - 实际速度系数，0x606C 返回值 / 系数 = RPM（一般 10.0）
  //   low_channel_is_left - true 表示通道1（打包低16位）是左轮
  //   sdo_timeout_ms      - SDO 应答超时时间
  CanopenNode(const std::string& ifname, uint8_t node_id,
              double target_unit_per_rpm = 1.0,
              double actual_unit_per_rpm = 10.0,
              bool low_channel_is_left = true,
              int sdo_timeout_ms = 100);

  bool open();
  void close();
  bool isOpen() const;

  // ---------- NMT 网络管理 ----------
  bool nmtStart();                // 进入运行态（Operational），心跳开始发
  bool nmtStop();                 // 停止态
  bool nmtEnterPreOperational();  // 预运行态
  bool nmtResetNode();            // 复位节点

  // ---------- SDO 读写（阻塞等应答，初始化/使能用）----------
  bool sdoWrite8(uint16_t index, uint8_t sub, uint8_t value);
  bool sdoWrite16(uint16_t index, uint8_t sub, uint16_t value);
  bool sdoWrite32(uint16_t index, uint8_t sub, uint32_t value);
  bool sdoRead(uint16_t index, uint8_t sub, uint32_t& value);

  // ---------- 电机控制 ----------
  bool setSyncMode();                  // 0x200F=1 开启同步控制
  bool setAcceleration(int32_t accel); // 0x6083:01/02 加速度
  bool setDeceleration(int32_t decel); // 0x6084:01/02 减速度
  bool setAccelDecel(int32_t accel, int32_t decel);
  bool enableMotor();                  // 使能（同步模式+心跳+速度模式+控制字+NMT）
  bool enableMotor(int32_t accel, int32_t decel);  // 带加减速的使能
  bool disableMotor();
  bool clearFault();
  bool saveParameters();               // 0x2010=1 存 EEPROM，掉电保留配置

  // 目标速度：0x60FF:03 低16位=通道1，高16位=通道2，只发不等应答
  bool setTargetRpm(double left_rpm, double right_rpm);

  // 实际速度：读缓存（由 requestActualRpm + pollOnce 更新）
  bool readActualRpm(double& left_rpm, double& right_rpm);

  // ---------- 主动读请求：只发请求，结果由 pollOnce 收进缓存 ----------
  bool requestActualRpm();   // 0x606C:03 实际速度
  bool requestStatus();      // 0x6041:00 状态字
  bool requestFault();       // 0x603F:00 故障码

  // ---------- 非阻塞收帧：必须在高频定时器里调用（10ms 一次）----------
  void pollOnce();

  // ---------- 状态查询 ----------
  bool heartbeatAlive(double timeout_sec) const;  // 心跳是否还活着
  bool hasFault() const { return fault_code_ != 0; }
  uint16_t statusWord() const { return status_word_; }
  uint32_t faultCode() const { return fault_code_; }

private:
  bool sendNmt(uint8_t command);
  bool sendSdoWrite(uint16_t index, uint8_t sub, uint8_t command, uint32_t value);
  bool sendReadRequest(uint16_t index, uint8_t sub);
  bool waitForSdoResponse(uint16_t index, uint8_t sub, uint32_t& value, int timeout_ms);
  void handleHeartbeat(uint8_t state);
  void handleSdoResponse(const struct can_frame& frame);
  int16_t clampToInt16(double v) const;

  std::string ifname_;
  uint8_t node_id_;
  double target_unit_per_rpm_;
  double actual_unit_per_rpm_;
  bool low_channel_is_left_;
  int sdo_timeout_ms_;

  CanSocket can_;

  // 心跳缓存
  bool heartbeat_seen_ = false;
  uint8_t heartbeat_state_ = 0;
  std::chrono::steady_clock::time_point last_heartbeat_tp_;

  // SDO 应答缓存（pollOnce 更新）
  bool actual_rpm_valid_ = false;
  double cached_left_rpm_ = 0.0;
  double cached_right_rpm_ = 0.0;
  std::chrono::steady_clock::time_point last_actual_rpm_tp_;

  uint16_t status_word_ = 0;
  uint32_t fault_code_ = 0;
};

#endif  // ZLAC8015D_H
