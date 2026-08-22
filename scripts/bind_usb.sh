#!/bin/bash
set -euo pipefail

echo "=============================================="
echo "  小车 USB 串口 udev 别名安装器"
echo "=============================================="

if [ "$(id -u)" -ne 0 ]; then
  echo "ERROR: 请用 sudo 运行: sudo bash scripts/bind_usb.sh" >&2
  exit 1
fi

# N300Pro IMU：CP2102 (10c4:ea60)，序列号 0003 -> /dev/robot_imu
# RPLIDAR 雷达：CP2102 (10c4:ea60)，非 0003 序列号 -> /dev/robot_lidar
# 两条规则用 serial 区分，避免 IMU 和雷达都被映射成同一个名字。
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="robot_imu"' > /etc/udev/rules.d/99-robot-imu.rules
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}!="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="robot_lidar"' > /etc/udev/rules.d/99-robot-lidar.rules

echo "- 重新加载 udev 规则..."
udevadm control --reload-rules || service udev reload
udevadm trigger || true
sleep 1

echo "=============================================="
echo "完成！检查结果："
ls -l /dev/robot_imu /dev/robot_lidar 2>/dev/null || true
echo ""
echo "提示：如果 /dev/robot_imu 不存在，请先拔插 IMU 再执行一次本脚本，"
echo "或检查 IMU 的序列号（lsusb -v | grep -i serial）。"
echo ""
echo "CAN 接口还需要手动配置（以 can0 500k 为例）："
echo "  sudo ip link set can0 down"
echo "  sudo ip link set can0 up type can bitrate 500000"
echo "=============================================="
