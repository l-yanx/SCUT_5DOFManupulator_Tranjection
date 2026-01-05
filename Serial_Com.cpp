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

static int clamp_int(int v, int low, int high) {
    if (v < low) return low;
    if (v > high) return high;
    return v;
}

// 关节角度映射PWM脉冲
static int deg_to_pulse(double deg) {
    if (deg < -135.0) {
        deg = -135.0;
    }
    if (deg >  135.0) {
        deg =  135.0;
    }
    
    double p = 1500.0 + (2000.0 / 270.0) * deg;
    int pi = static_cast<int>(std::lround(p));
    return clamp_int(pi, 500, 2500);
}

//单个关节角度封装
static std::string format_single_servo_cmd(int id, int pulse, int time_ms) {
    id = clamp_int(id, 0, 999);
    pulse = clamp_int(pulse, 500, 2500);
    time_ms = clamp_int(time_ms, 0, 9999);

    std::ostringstream oss;
    // {#000P1500T0500!}
    oss << "#"
        << std::setw(3) << std::setfill('0') << id
        << "P" << std::setw(4) << std::setfill('0') << pulse
        << "T" << std::setw(4) << std::setfill('0') << time_ms
        << "!";
    return oss.str();
}

//多个角度集成封装
static std::string format_frame_6j(const std::array<int, 6>& pulses, int time_ms) {
    time_ms = clamp_int(time_ms, 0, 9999);

    std::ostringstream oss;
    oss << "{";
    for (int i = 0; i < 6; ++i) {
        oss << format_single_servo_cmd(i, pulses[i], time_ms);
    }
    oss << "}";
    return oss.str();
}

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

static void print_help() {
    std::cout <<
R"(Commands:
  set  <deg0> <deg1> <deg2> <deg3> <deg4> <deg5> <ms>     - Set joint by angle in degrees (-135..135). 
  home                                            - Set joints 0..n-1 to P1500. Default n=6. Example: home 800 7
  raw  <string>                                           - Send raw string exactly (useful for testing).
  quit                                                    - Exit.
)";
}

int main(int argc, char** argv) {
    std::string dev = "/dev/ttyUSB0";
    if (argc >= 2) dev = argv[1];

    SerialPort sp;
    if (!sp.open(dev, 115200)) {
        std::cerr << "Hint: if permission denied, run: sudo usermod -aG dialout $USER (then re-login)\n";
        return 1;
    }
    std::cout << "Opened " << dev << " at 115200 8N1.\n";
    print_help();

    std::string line;
    
    while (true) {
        std::cout << "> " << std::flush;

        if (!std::getline(std::cin, line)) 
            break;
        if (line.empty()) 
            continue;

        std::istringstream iss(line);
        std::string cmd;
        iss >> cmd;

//处理
        if (cmd == "quit" || cmd == "exit") {
            break;
        } else if (cmd == "help") {
            print_help();
            continue;
        } 
 /************************************************************************/       
        else if (cmd == "set") {
            double d0, d1, d2, d3, d4, d5;
            int ms;

            if (!(iss >> d0 >> d1 >> d2 >> d3 >> d4 >> d5 >> ms)) {
                std::cerr << "Usage: set <deg0> <deg1> <deg2> <deg3> <deg4> <deg5> <ms>\n";
                continue;
                }

            std::array<int, 6> pulses = {
                deg_to_pulse(d0),
                deg_to_pulse(d1),
                deg_to_pulse(d2),
                deg_to_pulse(d3),
                deg_to_pulse(d4),
                deg_to_pulse(d5),
                };

            std::string frame = format_frame_6j(pulses, ms);
            std::cout << "TX: " << frame << "\n";

            sp.write_all(frame);
        }  else if (cmd == "home") {

            std::ostringstream oss;

            oss << "{";
            for (int id = 0; id < 6; ++id) {
                oss << "#"
                    << std::setw(3) << std::setfill('0') << id
                    << "P1500T" << std::setw(4) << std::setfill('0') << "1000"
                    << "!";
            }
            oss << "}";

            std::string frame = oss.str();
            std::cout << "TX: " << frame << "\n";

            sp.write_all(frame);
        } else {
            std::cerr << "Unknown command. Type 'help'.\n";
        }

    }

    std::cout << "Bye.\n";
    return 0;
}
