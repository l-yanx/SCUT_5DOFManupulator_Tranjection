cmake_minimum_required(VERSION 3.16)
project(ros_ws_simple LANGUAGES CXX)

# ===== 基本编译选项 =====
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 可选：你想看更清晰的警告就开；不想看就注释
add_compile_options(-Wall -Wextra -Wpedantic)

# ===== 路径 =====
set(ARM_CONTROL_DIR   ${CMAKE_SOURCE_DIR}/src/arm_control)
set(ARM_DRIVER_DIR    ${CMAKE_SOURCE_DIR}/src/arm_driver)
set(ARM_PLANNING_DIR  ${CMAKE_SOURCE_DIR}/src/arm_planning)
set(ARM_TASK_DIR      ${CMAKE_SOURCE_DIR}/src/arm_task)

# ===== arm_control 静态库 =====
add_library(arm_control STATIC
    ${ARM_CONTROL_DIR}/src/forward_kinematics.cpp
)
target_include_directories(arm_control PUBLIC
    ${ARM_CONTROL_DIR}/inc
)

# ===== arm_driver 静态库 =====
add_library(arm_driver STATIC
    ${ARM_DRIVER_DIR}/src/arm_serial_node.cpp
)
target_include_directories(arm_driver PUBLIC
    ${ARM_DRIVER_DIR}/inc
)

# ===== arm_planning 静态库 =====
add_library(arm_planning STATIC
    ${ARM_PLANNING_DIR}/src/arm_planning.cpp
)
target_include_directories(arm_planning PUBLIC
    ${ARM_PLANNING_DIR}/inc
)

# 关键：arm_planning 里调用 arm_control / arm_driver 的符号
# 用 PUBLIC 让依赖自动传递给最终可执行文件
target_link_libraries(arm_planning PUBLIC
    arm_control
    arm_driver
)

# ===== 最终可执行文件（main 在 arm_ui.cpp） =====
add_executable(arm_ui_node
    ${ARM_TASK_DIR}/src/arm_ui_node.cpp
)

# arm_ui.cpp 里如果 include 了其他包的头文件，这里加 include 路径
# 用 arm_planning 的 PUBLIC include 会传递过来，
# 但 arm_task 自己的 include 如果将来有，也可以加
target_include_directories(arm_ui_node PRIVATE
    ${ARM_TASK_DIR}/inc
)

# 链接 planning（依赖会自动带上 control/driver）
target_link_libraries(arm_ui_node PRIVATE
    arm_planning
)
