#pragma once

namespace uav_control
{
    enum class CommandState : int
    {
        OTHER = -1,      // 其他/未知命令
        UNLOCK = 0,     // 解锁
        TAKEOFF = 1,    // 起飞
        GO_TO_WAYPOINT = 2, // 飞向指定航点
        LAND = 3,        // 降落
        LANDING = -2,     // 降落中（内部状态，非输入命令）
        HOVER = 4,        // 悬停（保持当前位置）
        TRAJ = 5,           // 轨迹跟踪
        CHANGECONTROLLER = 6,   // 切换控制器
        PLANNER = 7,           // 由规划器生成的轨迹跟踪
    };
    enum class TarjKind : int
    {
        OTHER = -1,      // 其他/未知轨迹
        CIRCLE = 0,     // 圆轨迹
        POLY5 = 1,      // 五次多项式轨迹
        LISSAJOUS = 2,  // 李萨如曲线轨迹
    };
    enum class ControlerKind : int
    {
        OTHER = -1,      // 其他/未知控制器
        PX4 = 0,         // PX4控制器
        SO3 = 1,         // SO3控制器
    };
}
