#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <sys/select.h>
#include <cmath>

//串口通信类
class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort() { close(); }

    bool open(const std::string& device, int baud = 115200) {
        close();

        fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            std::cerr << "Failed to open " << device << ": " << std::strerror(errno) << "\n";
            return false;
        }

        termios tty{};
        if (tcgetattr(fd_, &tty) != 0) {
            std::cerr << "tcgetattr failed: " << std::strerror(errno) << "\n";
            close();
            return false;
        }

        // Raw mode
        cfmakeraw(&tty);

        // 8N1
        tty.c_cflag &= ~PARENB; // no parity
        tty.c_cflag &= ~CSTOPB; // 1 stop bit
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;     // 8 bits
        tty.c_cflag |= (CLOCAL | CREAD);

        // No flow control
        tty.c_cflag &= ~CRTSCTS;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);

        // Read settings: non-blocking with select()
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 0;

        speed_t spd = B115200;
        if (baud != 115200) {
            std::cerr << "Only 115200 implemented in this minimal example.\n";
        }
        cfsetispeed(&tty, spd);
        cfsetospeed(&tty, spd);

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            std::cerr << "tcsetattr failed: " << std::strerror(errno) << "\n";
            close();
            return false;
        }

        int flags = fcntl(fd_, F_GETFL, 0);
        if (flags >= 0) {
            fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);
        }

        return true;
    }

    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    bool is_open() const { return fd_ >= 0; }

    bool write_all(const std::string& s) {

        if (fd_ < 0) return 
            false;

        const char* buf = s.c_str();
        size_t total = 0;

        while (total < s.size()) {
            ssize_t n = ::write(fd_, buf + total, s.size() - total);
            if (n < 0) {

                if (errno == EINTR) continue;
                std::cerr << "write failed: " << std::strerror(errno) << "\n";
                return false;
            }
            total += static_cast<size_t>(n);
        }
        // Optional: flush output
        tcdrain(fd_);
        return true;
    }

    // Read available bytes with timeout (ms). Returns string if any bytes read.
    std::optional<std::string> read_some(int timeout_ms = 50) {
        if (fd_ < 0) return std::nullopt;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd_, &rfds);

        timeval tv{};
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;

        int ret = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
        if (ret < 0) {
            if (errno == EINTR) return std::nullopt;
            std::cerr << "select failed: " << std::strerror(errno) << "\n";
            return std::nullopt;
        }
        if (ret == 0) return std::nullopt; // timeout

        char buf[512];
        ssize_t n = ::read(fd_, buf, sizeof(buf));
        if (n <= 0) return std::nullopt;

        return std::string(buf, buf + n);
    }

private:
    int fd_{-1};
};

extern SerialPort sp;

int serial_communicantiom(std::array<double,6>& d, int ms);