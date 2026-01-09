#include <../../arm_control/inc/arm_control.hpp>
#include <../../arm_driver/inc/arm_serial_node.hpp>
#include <../inc/arm_planning.hpp>

// 五次时间标定
double quintic(double t, double T)
{
    if (t <= 0)
        return 0;
    if (t >= T)
        return 1;
    double tau = t / T;
    double tau2 = tau * tau, tau3 = tau2 * tau, tau4 = tau3 * tau, tau5 = tau4 * tau;
    // return 10 * tau - 15 * tau2 + 6 * tau3;
    return 10 * tau3 - 15 * tau4 + 6 * tau5;
}

// 直线插值
std::array<double, 5> linear_trajectory(std::array<double, 5> start, std::array<double, 5> end, double u)
{
    return {start[0] + u * (end[0] - start[0]),
            start[1] + u * (end[1] - start[1]),
            start[2] + u * (end[2] - start[2]),
            start[3] + u * (end[3] - start[3]),
            start[4] + u * (end[4] - start[4])};
}

int trajectory2joint()
{
    double u = 0;
    std::array<double, 5> s, e, send_msg;
    std::array<double, 6> q{};
    std::cout << "请输入起点位姿 (x y z pitch roll):\n";
    for (int i = 0; i < 5; ++i)
    {
        std::cin >> s[i];
    }

    std::cout << "请输入终点位姿 (x y z pitch roll):\n";
    for (int i = 0; i < 5; ++i)
    {
        std::cin >> e[i];
    }

    double T, dt;
    std::cout << "请输入全程时间以及单点时间:\n";
    std::cin >> T;
    std::cin >> dt;

    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();
    auto next = t0;
    int N = static_cast<int>(T / dt) + 1;

    for (int k = 0; k < N; ++k)
    {
        double t = k * dt;
        /*------------------------------------------------------------------*/
        // 插分系数
        u = quintic(t, T);
        // 轨迹点
        send_msg = linear_trajectory(s, e, u);
        // ik
        q = arm_calc.inverse_kinmastics(send_msg);
        // 串口
        if (serial_communicantiom(q, 1000))
        {
            fk(q);
        }
        std::cout << "规划轨迹点:" << send_msg[0] << "  " << send_msg[1] << "  " << send_msg[2] << "  " << send_msg[3] << "  " << send_msg[4] << "\n";
        std::cout << "关节轨迹点:" << q[0] << "  " << q[1] << "  " << q[2] << "  " << q[3] << "  " << q[4] << "\n";
        std::cout << "当前时间：" << t << "\n";
        std::cout << "-----------------------------------------------------------------\n";
        /*------------------------------------------------------------------*/
        next = t0 + std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>((k + 1) * dt));
        if (clock::now() < next)
        {
            std::this_thread::sleep_until(next);
        }
        else
        {
            printf("Cycle overrun at k=%d\n", k);
            return 0;
        }
    }
    return 1;
}

// 任务1-------------------------------------------------------------------------------------------------
//  多行读值
static bool read_waypoints(std::vector<std::array<double, 5>> &wps)
{
    std::cout << "每行输入一个点: x y z pitch roll (deg)，输入 done 结束\n";
    std::cout << "> ";

    std::string line;
    std::getline(std::cin, line); // 清掉上一次 >> 残留的换行（如果你之前用过cin>>）
    while (true)
    {
        std::cout << "> ";
        if (!std::getline(std::cin, line))
            return false;
        if (line == "done" || line == "end" || line == "quit")
            break;
        if (line.empty())
            continue;

        std::istringstream iss(line);
        std::array<double, 5> p{};
        if (!(iss >> p[0] >> p[1] >> p[2] >> p[3] >> p[4]))
        {
            std::cerr << "格式错误，请输入 5 个数或 done\n";
            continue;
        }
        wps.push_back(p);
    }

    if (wps.size() < 2)
    {
        std::cerr << "至少需要 2 个点（起点+终点）\n";
        return false;
    }
    return true;
}

// waypoints求ik
static bool compute_joint_waypoints(
    const std::vector<std::array<double, 5>> &wps,
    std::vector<std::array<double, 6>> &q_wps)
{
    q_wps.clear();
    q_wps.reserve(wps.size());

    for (size_t i = 0; i < wps.size(); ++i)
    {
        auto target = wps[i]; // 注意 inverse_kinmastics 参数是引用，会改 pitch/roll 为rad，最好传副本
        std::array<double, 6> q = arm_calc.inverse_kinmastics(target);

        // 建议：这里加 joint_check(q)（单位要一致：你的 q 返回的是 deg）
        // if (!arm_calc.joint_check(q)) return false;

        q_wps.push_back(q);
    }
    return true;
}

// 插值
static std::array<double, 6> lerp_joint(const std::array<double, 6> &a,
                                        const std::array<double, 6> &b,
                                        double u)
{
    std::array<double, 6> out{};
    for (int i = 0; i < 6; ++i)
        out[i] = a[i] + u * (b[i] - a[i]); // deg
    return out;
}

// 执行
int execute_waypoints_jointspace()
{
    std::vector<std::array<double, 5>> wps;
    if (!read_waypoints(wps))
        return 0;

    double T_total, dt;
    std::cout << "请输入全程时间 T_total 和采样间隔 dt(秒）:\n";
    std::cin >> T_total >> dt;

    // 1) 先求每个 waypoint 的关节角
    std::vector<std::array<double, 6>> q_wps;
    if (!compute_joint_waypoints(wps, q_wps))
        return 0;

    //    这里给最简单版本：均分
    const int segs = (int)q_wps.size() - 1;
    const double T_seg = T_total / segs;

    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();
    auto next = t0;

    int k_global = 0;
    for (int s = 0; s < segs; ++s)
    {
        int N = (int)(T_seg / dt) + 1;
        for (int k = 0; k < N; ++k, ++k_global)
        {
            double t = k * dt;
            double u = quintic(t, T_seg);

            std::array<double, 6> q_cmd = lerp_joint(q_wps[s], q_wps[s + 1], u);
            if (k == N - 1)
            {
                q_cmd[5] = 90;
            }

            // 下发：你 serial_communicantiom 当前签名看起来是 serial_communicantiom(q,1000)
            if (serial_communicantiom(q_cmd, 500))
            {
                fk(q_cmd);
            }
            std::cout << "当前时间：" << t << "\n";
            std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n";
            next = t0 + std::chrono::duration_cast<clock::duration>(
                            std::chrono::duration<double>((k_global + 1) * dt));
            if (clock::now() < next)
                std::this_thread::sleep_until(next);
            else
            {
                std::cerr << "Cycle overrun\n";
                return 0;
            }
        }
    }
    return 1;
}

// 任务二-----------------------------------------------------------------------------------------------------------
struct Vec3 {
    double x, y, z;
};

static Vec3 operator+(const Vec3& a, const Vec3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

static Vec3 operator*(double s, const Vec3& v) {
    return {s * v.x, s * v.y, s * v.z};
}

static Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

static double norm(const Vec3& v) {
    return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}

static Vec3 normalize(const Vec3& v) {
    double n = norm(v);
    return {v.x/n, v.y/n, v.z/n};
}

//画圆
int generate_circle_waypoints()
{
    std::vector<std::array<double,5>> wps;
    wps.clear();

    Vec3 center, normal;
    double radius;
    double pitch_deg, roll_deg;
    int num_points;

    std::cout << "请输入圆心 (x y z):\n";
    std::cin >> center.x >> center.y >> center.z;

    std::cout << "请输入平面法向量 (nx ny nz):\n";
    std::cin >> normal.x >> normal.y >> normal.z;

    if (norm(normal) < 1e-6) {
        std::cerr << "法向量非法\n";
        return 0;
    }

    std::cout << "请输入半径 r:\n";
    std::cin >> radius;

    std::cout << "请输入固定 pitch 和 roll (deg):\n";
    std::cin >> pitch_deg >> roll_deg;

    std::cout << "请输入离散点数量:\n";
    std::cin >> num_points;

    if (num_points < 10 || radius <= 0) {
        std::cerr << "参数非法\n";
        return 0;
    }

    // 单位法向
    Vec3 n = normalize(normal);

    // 构造平面正交基
    Vec3 tmp = (std::fabs(n.z) < 0.9) ? Vec3{0,0,1} : Vec3{0,1,0};
    Vec3 u = normalize(cross(tmp, n));
    Vec3 v = cross(n, u);

    // 生成圆轨迹
    for (int i = 0; i <= num_points; ++i) {
        double theta = 2.0 * M_PI * i / num_points;

        Vec3 p =
            center +
            radius * std::cos(theta) * u +
            radius * std::sin(theta) * v;

        wps.push_back({
            p.x,
            p.y,
            p.z,
            pitch_deg,
            roll_deg
        });
    }
    

    std::cout << "已生成圆轨迹点数: " << wps.size() << "\n";

    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();
    auto next = t0;
    int N = wps.size();
    const double dt=0.8;

    for (int k = 0; k < N; ++k)
    {
        /*------------------------------------------------------------------*/
        std::array<double, 6> q{};
        // ik
        auto pose=wps[k];
        q = arm_calc.inverse_kinmastics(pose);
        // 串口
        if (serial_communicantiom(q, 1000))
        {
            fk(q);
        }
        std::cout << "第"<<k+1<<"个点---------------------------------------------------------------\n";
        /*------------------------------------------------------------------*/
        next = t0 + std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>((k+1)*dt));
        if (clock::now() < next)
        {
            std::this_thread::sleep_until(next);
        }
        else
        {
            printf("Cycle overrun at k=%d\n", k);
            return 0;
        }
    }

    return 1;
}


