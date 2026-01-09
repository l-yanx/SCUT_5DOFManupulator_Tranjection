#include <../inc/arm_serial_node.hpp>

SerialPort sp;

static int clamp_int(int v, int low, int high)
{
    if (v < low)
        return low;
    if (v > high)
        return high;
    return v;
}

// 关节角度映射PWM脉冲
static int deg_to_pulse(double deg)
{
    if (deg < -135.0)
    {
        deg = -135.0;
    }
    if (deg > 135.0)
    {
        deg = 135.0;
    }

    double p = 1500.0 + (2000.0 / 270.0) * deg;
    int pi = static_cast<int>(std::lround(p));
    return clamp_int(pi, 500, 2500);
}

// 单个关节角度封装
static std::string format_single_servo_cmd(int id, int pulse, int time_ms)
{
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

// 多个角度集成封装
static std::string format_frame_6j(const std::array<int, 6> &pulses, int time_ms)
{
    time_ms = clamp_int(time_ms, 0, 9999);

    std::ostringstream oss;
    oss << "{";
    for (int i = 0; i < 6; ++i)
    {
        oss << format_single_servo_cmd(i, pulses[i], time_ms);
    }
    oss << "}";
    return oss.str();
}

int serial_communicantiom(std::array<double, 6> &d, int ms)
{

    std::array<int, 6> pulses = {
        deg_to_pulse(d[0]),
        deg_to_pulse(d[1]),
        deg_to_pulse(d[2]),
        deg_to_pulse(d[3]),
        deg_to_pulse(d[4]),
        deg_to_pulse(d[5]),
    };

    std::string frame = format_frame_6j(pulses, ms);
    std::cout << "TX: " << frame << " is done" << "\n";
    std::cout << "-  -  -  -  -  -  -  -  -  -  -\n";

    sp.write_all(frame);

    return 1;
}