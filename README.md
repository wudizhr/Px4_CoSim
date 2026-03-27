# CoSim
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/24.04/)

本项目是一个使用 PX4 官方支持的 GZ Sim 进行无人机动力学仿真，和使用 MARSIM 进行传感器仿真的联合模拟器。

## 项目动机
PX4 官方支持的 GZ Sim 在 ROS 2 系统中可以十分方便地对 PX4 无人机进行仿真，但是由于 PX4 仓库强耦合的结构以及新版 Gazebo 的各种问题，使添加新的无人机模型以及制定的传感器都十分困难，~~经过三天尝试使用新版gazebo仿真mid360后果断选择放弃~~，转而选择 MARSIM 作为 Lidar 的数值仿真器。

## ToDo
1. 添加数值仿真部分的碰撞检测
2. 添加 ego-planner 模块进行测试
3. rviz指点飞行

## 参考
MARSIM: https://github.com/hku-mars/MARSIM

