// 正运动学计算末端坐标
#include <../inc/arm_control.hpp>

Arm_5DOF arm_calc;

void fk(std::array<double, 6> &deg)
{

    std::cout << deg[0] << " " << deg[1] << " " << deg[2] << " " << deg[3] << " " << deg[4] << "\n";

    Mat4 T = Mat4::identity();

    T = arm_calc.forward_kinematics(deg);
    // 位置坐标
    double position[3] = {};
    position[0] = T.m[0][3];
    position[1] = T.m[1][3];
    position[2] = T.m[2][3];

    double oriention[2] = {};
    oriention[0] = 90 - (deg[1] + deg[3] - deg[2]);
    oriention[1] = deg[4];

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "End-effector position (x y z pitch roll): " << position[0] << "  " << position[1] << "  " << position[2] << "  " << oriention[0] << "  " << oriention[1] << "\n";
    std::cout << "-  -  -  -  -  -  -  -  -  -  -\n";
    // Rotation matrix
    // std::cout << "End-effector rotation R (3x3):\n";
    // for (int r = 0; r < 3; ++r)
    // {
    //     std::cout << "  ";
    //     for (int c = 0; c < 3; ++c)
    //     {
    //         std::cout << std::setw(12) << T.m[r][c] << (c == 2 ? "" : " ");
    //     }
    //     std::cout << "\n";
    //     std::cout << "--------------------------------------------------------------\n";
    // }
}
