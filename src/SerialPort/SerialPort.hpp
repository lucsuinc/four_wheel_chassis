#ifndef SERIALPORT_HPP
#define SERIALPORT_HPP

#include <string>
#include <cstdint>

class SerialPort
{
public:
    SerialPort();
    ~SerialPort();

    // 打开串口：设备名，波特率
    bool open(const std::string& dev_name, int baudrate);
    void close();

    // 发送数据
    int write(const uint8_t* buf, int len);

    // 读取数据，阻塞/超时读取
    int read(uint8_t* buf, int max_len, int timeout_ms = 100);

    bool isOpen() const { return m_fd >= 0; }

private:
    int m_fd{-1};
    // 设置波特率、8N1
    bool setTermios(int baudrate);
};

#endif
