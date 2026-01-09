#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <algorithm>

void fk(std::array<double, 6> &deg);

inline double clamp(double x)
{
    return std::max(-1.0, std::min(1.0, x));
}

// 矩阵基础运算--------------------------------------------------------------------//

// 齐次变换矩阵
struct Mat4
{
    double m[4][4]{};

    static Mat4 identity()
    {
        Mat4 I;
        for (int i = 0; i < 4; ++i)
            I.m[i][i] = 1.0;
        return I;
    }
};

// 矩阵乘法
static Mat4 mul(const Mat4 &A, const Mat4 &B)
{
    Mat4 C{};
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            double s = 0.0;
            for (int k = 0; k < 4; ++k)
                s += A.m[i][k] * B.m[k][j];
            C.m[i][j] = s;
        }
    }
    return C;
}

// 角度弧度映射
static double deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}

static double rad2deg(double rad)
{
    return rad * 180.0 / M_PI;
}
class Arm_5DOF
{

public:
    // 机械臂结构体------------------------------------------------------//
    struct arm_5DOF
    {
        // dh参数
        std::array<double, 6> a = {
            /* a1 */ 0.0,
            /* a2 */ 10.0,
            /* a3 */ 105.0,
            /* a4 */ 98.0,
            /* a5 */ 0.0,
            /* a */ 0.0};
        std::array<double, 6> alpha = {
            /* alpha1 */ 0.0,
            /* alpha2 */ -M_PI_2,
            /* alpha3 */ M_PI,
            /* alpha4 */ 0,
            /* alpha5 */ -M_PI_2,
            /* alpha6 */ 0};
        std::array<double, 6> d = {
            /* d1 */ 0.0,
            /* d2 */ 0.0,
            /* d3 */ 0.0,
            /* d4 */ 0.0,
            /* d5 */ 0.0,
            /* d6 */ 165};
        std::array<double, 6> theta_offset = {
            /* off1 */ 0.0,
            /* off2 */ -M_PI_2,
            /* off3 */ 0.0,
            /* off4 */ -M_PI_2,
            /* off5 */ 0.0,
            /* off6 */ 0.0};
        std::array<double, 6> joint = {};

        // 关节参数
        Mat4 T_joint[7] = {};
    };

    // 关节角度检查
    bool joint_check(std::array<double, 6> &q)
    {
        double lim_135 = deg2rad(135);
        double lim_90 = deg2rad(90);
        if (q[0] >= -1 * lim_135 && q[0] <= lim_135)
        {
            if (q[1] >= -1 * lim_90 && q[1] <= lim_90)
            {
                if (q[2] >= -1 * lim_135 && q[2] <= lim_135)
                {
                    if (q[3] >= -1 * lim_135 && q[3] <= lim_135)
                    {
                        if (q[4] >= -1 * lim_135 && q[4] <= lim_135)
                        {
                            return true;
                        }
                    }
                }
            }
        }

        return false;
    }

    // 正运动学-------------------------------------------------------------------------//

    // dh i-1关节到i关节齐次矩阵
    static Mat4 dh(double a, double alpha, double d, double theta)
    {
        const double ct = std::cos(theta);
        const double st = std::sin(theta);
        const double ca = std::cos(alpha);
        const double sa = std::sin(alpha);

        Mat4 T = Mat4::identity();
        T.m[0][0] = ct;
        T.m[0][1] = -st;
        T.m[0][2] = 0;
        T.m[0][3] = a;
        T.m[1][0] = st * ca;
        T.m[1][1] = ct * ca;
        T.m[1][2] = -sa;
        T.m[1][3] = -d * st;
        T.m[2][0] = st * sa;
        T.m[2][1] = ct * sa;
        T.m[2][2] = ca;
        T.m[2][3] = d * ca;
        T.m[3][0] = 0.0;
        T.m[3][1] = 0.0;
        T.m[3][2] = 0.0;
        T.m[3][3] = 1.0;
        return T;
    }

    // 正运动学计算末端坐标系
    Mat4 forward_kinematics(std::array<double, 6> &deg)
    {
        deg[5] = 0;

        Mat4 T = Mat4::identity();
        arm_data.T_joint[0] = T;
        for (int i = 0; i < 6; ++i)
        {
            const double theta = deg2rad(deg[i]) + arm_data.theta_offset[i];
            T = mul(T, dh(arm_data.a[i], arm_data.alpha[i], arm_data.d[i], theta));
            arm_data.T_joint[i + 1] = T;
        }

        return T;
    }

    // 逆运动学-------------------------------------------------------------------------//

    // 运动代价
    int correct = 1;
    int cost_compare(const double q[2][2])
    {
        double cost1 =
            100 * std::pow(correct*rad2deg(q[0][0]) - arm_data.joint[1], 2) +
            1 * std::pow(correct*rad2deg(q[0][1]) - arm_data.joint[2], 2);

        double cost2 =
            100 * std::pow(correct*rad2deg(q[1][0]) - arm_data.joint[1], 2) +
            1 * std::pow(correct*rad2deg(q[1][1]) - arm_data.joint[2], 2);

        //std::cout << "代价比较：" << cost1 << "(" << rad2deg(q[0][0]) << "  " << rad2deg(q[0][1]) << ")" << "   " << cost2 << "(" << rad2deg(q[1][0]) << "  " << rad2deg(q[1][1]) << ")"
                                                                                                                                                                                     "-----------------\n";
        return (cost1 < cost2) ? 0 : 1;
    }
    
    std::array<double, 6> inverse_kinmastics(std::array<double, 5> &target)
    {   
        double pitch,roll;
        pitch = deg2rad(target[3]);
        roll = deg2rad(target[4]);

        std::array<double, 6> q{};
        double z_2 = 0;
        double R = 0;

        // 确定q0(转向)
         // 投影修正
        double q0_temp = 0;
        q0_temp = std::atan2(target[1], target[0]);
        // std::cout<<"q0_temp:"<<q0_temp<<"\n";
        correct=1;
        if (q0_temp > deg2rad(135) || q0_temp < deg2rad(-135))
        {

            q0_temp = q0_temp - M_PI;
            // std::cout<<"correct q0_temp:"<<q0_temp<<"\n";
            correct = -1;
        }
        q[0] = q0_temp;

        // 肩肘关节确定
        double r_3, r_2;
        r_3 = std::sqrt(target[0] * target[0] + target[1] * target[1]);
        // 减去末端得到平面二连杆
        r_2 = r_3 - 165 * std::cos(pitch) - correct * 10;
        z_2 = target[2] - 165 * std::sin(pitch);

        R = std::sqrt(z_2 * z_2 + r_2 * r_2);
        // std::cout << "R : " << R << "-------\n";

        if (R > 213)
        {
            std::cout << "target wrong!!!!!!!!!!!!!!!!!!!!\n";
            return arm_data.joint;
        }
        // 余弦定理算角度
        //  关节2关节角
        double c_2 = -1 * (R * R - (105 * 105 + 98 * 98)) / (2 * 105 * 98);
        c_2 = clamp(c_2);
        double s_2 = std::sqrt(1 - c_2 * c_2);
        double i = std::atan2(s_2, c_2);
        // 仅为数值大小
        q[2] = M_PI - i;

        double c_j = -1 * (98 * 98 - R * R - 105 * 105) / (2 * R * 105);
        c_j = clamp(c_j);
        double s_j = std::sqrt(1 - c_j * c_j);
        double j = std::atan2(s_j, c_j);

        double v = std::atan2(z_2, r_2);

        // 存肘的两组解
        double q_temp[2][2] = {};
        q_temp[0][0] = M_PI_2 - (v - j);
        q_temp[0][1] = q[2];
        q_temp[1][0] = M_PI_2 - (v + j);
        q_temp[1][1] = -1 * q[2];

        // 代价比较
        int best = cost_compare(q_temp);
        double q3_temp = M_PI_2 + q_temp[best][1] - q_temp[best][0] - pitch;
        //std::cout << "待核验的值" << q_temp[best][0] << "  " << q_temp[best][1] << "  " << q3_temp << "\n";
        if (q_temp[best][0] <= deg2rad(90) && q_temp[best][0] >= deg2rad(-90) &&
            q_temp[best][1] <= deg2rad(135) && q_temp[best][1] >= deg2rad(-135) &&
            q3_temp >= deg2rad(-135) && q3_temp <= deg2rad(135))
        {
            q[1] = q_temp[best][0];
            q[2] = q_temp[best][1];
        }
        else
        {
            std::cout << "q1/q2/q3越界了!     " << q3_temp << "\n";
            q[1] = q_temp[!best][0];
            q[2] = q_temp[!best][1];
        }
        q[3] = -1 * correct * (M_PI_2 + q[2] - q[1] - pitch);
        q[1] = correct * q[1];
        q[2] = correct * q[2];
        q[4] = roll;
        q[5] = 0;

        // 记录数据+角度简单

        if (joint_check(q))
        {
            for (int i = 0; i < 5; i++)
            {
                q[i] = rad2deg(q[i]);
                arm_data.joint[i] = q[i];
                // std::cout<<arm_data.joint[i]<<" ";
            }
        }
        else
        {
            q[3]=arm_data.joint[3];
        }
        return q;
    }

private:
    arm_5DOF arm_data;
};

extern Arm_5DOF arm_calc;