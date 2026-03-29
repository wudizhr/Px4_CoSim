# CoSim
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/24.04/)

本项目是一个使用 GZ Sim 进行无人机动力学仿真，和使用 MARSIM 进行雷达仿真的PX4联合模拟器。

## 项目动机
PX4 官方支持的 GZ Sim 在 ROS 2 系统中可以十分方便地对 PX4 无人机进行仿真，但是由于 PX4 仓库强耦合的结构以及新版 Gazebo 的各种问题，使添加新的无人机模型以及制定的传感器都十分困难，~~经过三天尝试使用新版gazebo仿真mid360后果断选择放弃~~，转而选择 MARSIM 作为 Lidar 的数值仿真器。

## 安装
```
// 下载代码
git clone https://github.com/wudizhr/Px4_CoSim.git

// 编译
cd Px4_CoSim && colcon build

// 记得source仓库 (可以添加到 ~/.bashrc 中)
source install/setup.bash
```

## 使用方法
本项目需要已经完成好PX4 ROS2 Gazebo仿真的环境配置如果还没有配置可以参考阿木实验室的教程：
https://www.amovlab.com/news/detail?id=329

1. 启动PX4 SITL仿真
    ```
    cd PX4-Autopilot

    make px4_sitl gz_x500
    //可使用无头模式启动节省性能
    HEADLESS=1 make px4_sitl gz_x500
    ```
2. 启动MicroXRCEAgent
    ```
    MicroXRCEAgent udp4 -p 8888
    ```
3. 启动QGC地面站
4. 启动无人机控制终端
    ```
    ros2 launch uav_control uav_control_terminal.launch.py
    ```
5. 启动Lidar仿真
    ```
    ros2 launch test_interface single_px4drone_simple.launch.py 
    ```
6. 启动ego planner
    ```
    ros2 launch ego_planner px4_marsim_lidar_min.launch.py

    ```

## ToDo
1. 添加数值仿真部分的碰撞检测(离线)
2. 添加 ego-planner 模块进行测试
3. rviz指点飞行
4. 一键启动脚本？

## 参考
MARSIM: https://github.com/hku-mars/MARSIM

ego-planner: https://github.com/ZJU-FAST-Lab/ego-planner-swarm.git

