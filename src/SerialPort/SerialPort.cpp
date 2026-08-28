#include "SerialPort.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <cstring>
#include <cerrno>
#include <iostream>

SerialPort::SerialPort()
{
}

SerialPort::~SerialPort()
{
    close();
}

bool SerialPort::open(const std::string& dev_name, int baudrate)
{
    close();
    // O_RDWR读写，O_NOCTTY不成为控制终端
    m_fd = ::open(dev_name.c_str(), O_RDWR | O_NOCTTY);
    if(m_fd < 0)
    {
        std::cerr << "open " << dev_name << " failed: " << strerror(errno) << "\n";
        return false;
    }
    if(!setTermios(baudrate))
    {
        close();
        return false;
    }
    tcflush(m_fd, TCIOFLUSH); // 清空缓冲区
    return true;
}

void SerialPort::close()
{
    if(m_fd >=0)
    {
        ::close(m_fd);
        m_fd = -1;
    }
}

bool SerialPort::setTermios(int baudrate)
{
    struct termios opt{};
    if(tcgetattr(m_fd, &opt) != 0)
        return false;

    speed_t speed;
    switch (baudrate)
    {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        default: speed = B9600;
    }
    cfsetispeed(&opt, speed);
    cfsetospeed(&opt, speed);

    opt.c_cflag &= ~PARENB;    // 无校验
    opt.c_cflag &= ~CSTOPB;    // 1停止位
    opt.c_cflag &= ~CSIZE;
    opt.c_cflag |= CS8;        // 8数据位
    opt.c_cflag |= CLOCAL | CREAD;

    // 原始模式，关闭回显、规范模式
    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    opt.c_iflag &= ~(IXON | IXOFF | IXANY);
    opt.c_oflag &= ~OPOST;

    // 最小读取字节0，超时0，交给select做超时
    opt.c_cc[VMIN]  = 0;
    opt.c_cc[VTIME] = 0;

    if(tcsetattr(m_fd, TCSANOW, &opt) != 0)
        return false;
    return true;
}

int SerialPort::write(const uint8_t *buf, int len)
{
    if(!isOpen()) return -1;
    int ret = ::write(m_fd, buf, len);
    tcdrain(m_fd); // 等待全部数据发送完毕，RS485非常关键
    return ret;
}

int SerialPort::read(uint8_t *buf, int max_len, int timeout_ms)
{
    if(!isOpen()) return -1;

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(m_fd, &fds);

    struct timeval tv;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int sel = select(m_fd + 1, &fds, nullptr, nullptr, &tv);
    if(sel <= 0)
        return 0; // 超时或者错误

    return ::read(m_fd, buf, max_len);
}
