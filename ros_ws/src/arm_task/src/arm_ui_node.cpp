#include <array>
#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <../../arm_control/inc/arm_control.hpp>
#include <../../arm_driver/inc/arm_serial_node.hpp>
#include <../../arm_planning/inc/arm_planning.hpp>

class TerminalNoEcho
{
public:
    TerminalNoEcho()
    {
        tcgetattr(STDIN_FILENO, &orig_);
        termios raw = orig_;

        raw.c_lflag &= ~(ICANON | ECHO); // 非规范模式 + 关闭回显
        raw.c_cc[VMIN] = 0;              // 允许 0 字节返回
        raw.c_cc[VTIME] = 0;

        tcsetattr(STDIN_FILENO, TCSANOW, &raw);

        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    ~TerminalNoEcho()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
    }

private:
    termios orig_;
};

bool poll_key_noecho(char &key)
{
    unsigned char c;
    ssize_t n = read(STDIN_FILENO, &c, 1);
    if (n == 1)
    {
        key = static_cast<char>(c);
        return true;
    }
    return false;
}
int main()
{
    std::string dev = "/dev/ttyUSB0";
    if (!sp.open(dev, 115200))
    {
        std::cerr << "Hint: if permission denied, run: sudo usermod -aG dialout $USER (then re-login)\n";
        return 1;
    }

    while (true)
    {
        std::cout << "\n\n=============================\n\n";
        std::cout << "请选择模式：\n";
        std::cout << "  1) 手动操纵模式（按键调关节）\n";
        std::cout << "  2) 不定量插值模式（多点）\n";
        std::cout << "  3) 画圆模式(任意平面),固定pitch&roll\n";
        std::cout << "  0) 退出\n";
        std::cout << "输入 0/1/2/3: \n";
        std::cout << ">";

        int mode = -1;
        std::cin >> mode;

        if (mode == 0)
            break;
        if (mode == 1)
        {
            TerminalNoEcho term; // 进入无回显按键模式（作用域内有效）

            bool stop = false;
            std::array<double, 6> t{};

            while (!stop)
            {
                char key;
                const int a=8;
                if (poll_key_noecho(key))
                {
                    if (key == 'q')
                    {
                        t[0] += a;
                    }
                    else if (key == 'w')
                    {
                        t[0] -= a;
                    }
                    else if (key == 'e')
                    {
                        t[1] += a;
                    }
                    else if (key == 'r')
                    {
                        t[1] -= a;
                    }
                    else if (key == 't')
                    {
                        t[2] += a;
                    }
                    else if (key == 'y')
                    {
                        t[2] -= a;
                    }
                    else if (key == 'u')
                    {
                        t[3] += a;
                    }
                    else if (key == 'i')
                    {
                        t[3] -= a;
                    }
                    else if (key == 'o')
                    {
                        t[4] += a;
                    }
                    else if (key == 'p')
                    {
                        t[4] -= a;
                    }
                    else if (key == 'a')
                    {
                        t[5] += a;
                    }
                    else if (key == 's')
                    {
                        t[5] -= a;
                    }
                    else if (key == 'z')
                    {
                        stop = true;
                    }

                    if(serial_communicantiom(t,300))
                    {
                        fk(t);
                    };
                }
            }
        }
        else if (mode == 2)
        {
            execute_waypoints_jointspace();
        }
        else if (mode == 3)
        {
            generate_circle_waypoints();
        }
        else
            std::cout << "无效输入。\n";
    }

    std::cout << "Exit.\n";
    return 0;
}
