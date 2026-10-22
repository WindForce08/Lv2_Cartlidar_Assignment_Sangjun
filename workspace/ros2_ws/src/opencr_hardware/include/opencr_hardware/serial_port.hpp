// POSIX termios 시리얼 포트 (USB CDC). 비차단 읽기 · 원시 모드 · 독점 열기.
//   독점(TIOCEXCL + flock): 다른 프로세스가 /dev/opencr 를 같이 열면 패킷이 섞인다 (Xmen control.py exclusive=True 와 같은 이유)
#pragma once

#include <fcntl.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <string>

namespace opencr_hardware
{

class SerialPort
{
public:
  ~SerialPort() {close();}

  bool open(const std::string & path, speed_t baud, std::string & err)
  {
    close();
    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {err = path + ": " + std::strerror(errno); return false;}
    if (::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
      err = path + ": 다른 프로세스가 사용 중 (flock)";
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    ::ioctl(fd_, TIOCEXCL);   // 이후 다른 open() 도 거부
    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {err = std::strerror(errno); close(); return false;}
    cfmakeraw(&tio);
    cfsetispeed(&tio, baud);
    cfsetospeed(&tio, baud);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {err = std::strerror(errno); close(); return false;}
    tcflush(fd_, TCIOFLUSH);
    return true;
  }

  void close()
  {
    if (fd_ >= 0) {
      ::ioctl(fd_, TIOCNXCL);
      ::flock(fd_, LOCK_UN);
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool is_open() const {return fd_ >= 0;}

  // 읽은 바이트 수. 0 = 데이터 없음, -1 = 오류(장치 분리 등)
  ssize_t read_some(uint8_t * buf, size_t n)
  {
    if (fd_ < 0) {return -1;}
    const ssize_t r = ::read(fd_, buf, n);
    if (r < 0) {return (errno == EAGAIN || errno == EWOULDBLOCK) ? 0 : -1;}
    return r;
  }

  bool write_all(const uint8_t * buf, size_t n)
  {
    if (fd_ < 0) {return false;}
    size_t off = 0;
    int spins = 0;
    while (off < n) {
      const ssize_t w = ::write(fd_, buf + off, n - off);
      if (w > 0) {off += static_cast<size_t>(w); continue;}
      if (w < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {return false;}
      if (++spins > 1000) {return false;}   // 출력 버퍼가 계속 가득 참
      usleep(50);
    }
    return true;
  }

private:
  int fd_ = -1;
};

}  // namespace opencr_hardware
