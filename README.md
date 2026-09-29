# Dobot Nova 5 / MoveIt / Gazebo 学习与实践记录

跟随导师 NIU Xuezhi 的指导,practice Dobot 官方 [`DOBOT_6Axis_ROS2_V3`](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V3) ROS2 SDK(基于 Nova 5 机械臂),作为 WARA Robotics Challenge 2026 项目(manipulation-planning pipeline 提案)的前期准备工作。

## 项目背景

这个仓库记录的是"从环境搭建到能真正操作 Nova 5(先仿真、后真机)"这条学习路径上的完整过程——包括遇到的每一个具体问题、怎么排查、怎么解决,而不只是最终能跑通的结果。目的是:

1. 给导师 NIU Xuezhi 提供一份可查阅的、真实反映动手过程的进度记录
2. 为后续真正开发 WARA 项目的"位姿输入 → 抓取姿态计算 → MoveIt 路径规划 → 执行 → 异常处理"这条 pipeline 打基础

## 学习/实践记录

- [第1篇:Docker 环境搭建 + RViz/MoveIt/Gazebo 联动仿真跑通](./01_docker_environment_and_first_simulation.md)
  记录了在 Ubuntu 24.04 主机上,用 Docker(Ubuntu 22.04 + ROS2 Humble)运行 Dobot SDK 的完整过程,包括 7 个具体问题(依赖缺失、权限问题、环境变量丢失、控制器插件缺失等)的排查和修复,以及最终成功跑通"RViz 规划 → MoveIt 计算 → Gazebo 物理仿真执行"的完整闭环。

（后续每次新的实践进展,继续在这里追加链接）

## 环境

- 开发主机:Ubuntu 24.04.4 LTS
- 目标机型:Dobot Nova 5(实验室硬件,概念上接近 WARA 挑战赛使用的 ABB GoFa)
- 运行环境:Docker 容器(Ubuntu 22.04 + ROS2 Humble),因为 Dobot SDK 官方仅支持这个组合,与主机的 ROS2 Jazzy/Ubuntu 24.04 不兼容
- 图形界面转发:[rocker](https://github.com/osrf/rocker)(处理 X11/XWayland 转发)
- SDK 来源:[Dobot-Arm/DOBOT_6Axis_ROS2_V3](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V3)

## 目录结构

\`\`\`
dobot_nova5_practice/
├── README.md                                   本文件
├── 01_docker_environment_and_first_simulation.md   环境搭建 + 首次联动仿真跑通记录
├── Dockerfile                                  最终可用的容器构建文件
└── start_dobot.sh                              一键启动/进入容器脚本
\`\`\`
