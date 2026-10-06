# Dobot Nova 5 / MoveIt / Gazebo 学习与实践记录

跟随导师 NIU Xuezhi 的指导,practice Dobot 官方 [`DOBOT_6Axis_ROS2_V3`](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V3) ROS2 SDK(基于 Nova 5 机械臂),作为 WARA Robotics Challenge 2026 项目([manipulation-planning pipeline 提案](../wara-robotics-proposal))的前期准备工作。

## 项目背景

这个仓库记录的是"从环境搭建到能真正操作 Nova 5(先仿真、后真机)"这条学习路径上的完整过程——包括遇到的每一个具体问题、怎么排查、怎么解决,而不只是最终能跑通的结果。目的是:

1. 给导师 NIU Xuezhi 提供一份可查阅的、真实反映动手过程的进度记录
2. 为后续真正开发 WARA 项目的"位姿输入 → 抓取姿态计算 → MoveIt 路径规划 → 执行 → 异常处理"这条 pipeline 打基础

## 学习/实践记录

- [第1篇:Docker 环境搭建 + RViz/MoveIt/Gazebo 联动仿真跑通](./01_docker_environment_and_first_simulation.md)
  记录了在 Ubuntu 24.04 主机上,用 Docker(Ubuntu 22.04 + ROS2 Humble)运行 Dobot SDK 的完整过程,包括 7 个具体问题(依赖缺失、权限问题、环境变量丢失、控制器插件缺失等)的排查和修复,以及最终成功跑通"RViz 规划 → MoveIt 计算 → Gazebo 物理仿真执行"的完整闭环。

- [第2篇:MoveIt / ros2_control 配置体系精读 + 真机执行链路完整侦破](./02_moveit_ros2_control_architecture_and_real_hardware_bridge.md)
  逐个精读 `nova5_moveit/config/` 下的配置文件(SRDF、kinematics.yaml、moveit_controllers.yaml、ros2_controllers.yaml 等),并通过完整下载源码排查,定位到 Dobot SDK 真正用来驱动真机的桥梁代码(`action_move_server.py`),搞清楚它跟标准 ros2_control 架构的具体差异和已知弱点,并据此提出一个可用于 WARA 项目定量评估的分析方向(规划路径 vs 实际执行路径的轨迹跟踪误差对比)。

- [第3篇:手写 MoveGroup Action Client 实战 + demo 环境踩坑排查全记录](./03_hand_written_action_client_and_demo_debugging.md)
  不依赖 RViz 鼠标拖拽,自己写 Python 代码(rclpy action client)直接调用 MoveIt 的标准 action 接口,逐字段讲清楚 `MotionPlanRequest`/`PlanningOptions`/`Constraints` 等消息结构背后的含义;并完整记录了一次从 `CONTROL_FAILED(-4)` 报错到最终跑通成功的环境排查过程(缺包、终端/进程环境未刷新、容器镜像与 Dockerfile 不同步),沉淀出一套可复用的排查方法论,以及 demo/Gazebo/真机三条执行链路的最终完整对比。

- [第4篇:重建镜像 + Gazebo 链路验证跑通](./04_rebuild_image_and_gazebo_verified.md)
  用更新后的 `Dockerfile` 重建镜像(补齐 `joint-state-broadcaster`、加共享目录挂载),证实第3篇里 Gazebo 控制器加载失败的根因是旧镜像缺包;通过 `list_controllers`、action client、`/joint_states` 对照三步确认机械臂在 Gazebo 里真实到位,并附上参数化的测试脚本(`scripts/my_move_group_client.py`)。

- [第5篇:从 client 到 Gazebo 关节——ROS2 / MoveIt / ros2_control 整条通信链路梳理](./05_ros2_moveit_ros2_control_communication_flow.md)
  概念梳理篇:ROS2、MoveIt、ros2_control、Gazebo 各自是谁、怎么分层;话题/服务/action 的区别;`ros2_controllers.yaml` 与 `moveit_controllers.yaml` 逐字段对照(两个文件靠同一个实例名对上);一次运动从 `/move_action` 到 Gazebo 关节的完整数据通路(controller → command interface → `GazeboSystem` → `/joint_states` 回路);真机 `action_move_server.py` 的逻辑与弱点;哪些组件现成、哪些要自己写,以及 WARA 任务层节点的位置。

- [第6篇:自己实现 FK,并与 MoveIt `/compute_fk` 对比验证](./06_fk_implementation_and_moveit_compute_fk_verification.md)
  M2 任务 1 的 FK 部分:沿 URDF 关节链连乘(`xyz`/`rpy`/`axis` 的含义、Rodrigues 旋转公式)实现 `nova5_model.py`;讲清 `/compute_fk` 是 `move_group` 自带的服务、请求字段和响应内容;用 `compare_fk_simple.py` 对同一组关节角比较位置差和姿态差(含"相对旋转"公式的推导);并说明这个对比能证明什么(实现与 MoveIt 一致)、不能证明什么(URDF 与真机一致)。

- [第7篇:Jacobian 矩阵与奇异性——原理、构造与用法](./07_jacobian_and_singularity.md)
  M2 任务 2:`Δp ≈ J·Δq` 的来源(一元导数 → 多元偏导 → 小步线性近似);J 每一行/每一列的含义;线速度列 `zᵢ × (pₑ − pᵢ)` 与角速度列 `zᵢ` 的推导(含叉乘计算、为什么姿态用角速度而不是欧拉角导数);代码 `nova5_kinematics.py` 的对应与有限差分校验;正向(`v = J q̇`)与反向(`q̇ = J⁻¹ v`,沿 +x 走 10 cm 的分辨率速度控制)两种用法;奇异性(`det J = 0` 与"6 维独立"、二连杆例子、为什么危险、可操作度、Nova5 在 q5=0 的腕奇异结构、阻尼最小二乘),以及尚未确认几何含义的几处数值奇异。

- [第8篇:逆运动学(IK)——原理、实现与验证](./08_ik_principles_and_verification.md)
  M2 任务 1 的 IK 部分:IK 为什么难(多解、无解);数值 IK 的牛顿迭代(二连杆演示);"起点"是算法的初始猜测而不是机械臂状态、答案是绝对关节角;灵敏度(奇异值)与阻尼最小二乘 `Jᵀ(JJᵀ+λ²I)⁻¹e` 的来由;步长限制、关节限位、多起点去重;`ik_numeric`/`ik_all`/`pose_error_vec` 代码逐段说明;验证 1:往返验证(单起点 9/10,多起点 10/10,误差 < 0.001 mm);验证 2:与 MoveIt `/compute_ik`(KDL)对比,同一起点下两边得到同一组解,代回 FK 误差 < 0.001 mm,另找到 2 组其他解(含"肘朝上/朝下"多解);假设与局限。

- [第9篇:模型对比(自己的模型 vs 仿真器)与 M2 总结](./09_model_comparison_and_m2_summary.md)
  M2 后两项:用 `verify_model_consistency.py --sim` 在同一组实际关节角下对比自己的 FK、MoveIt `/compute_fk`、Gazebo 侧 TF,发现 Dobot V3 中 Gazebo 用的 `cra_description` xacro 与 MoveIt 用的 `dobot_rviz` URDF 的 joint1、joint4 转轴方向相反(只转这两个关节时末端差 140~170 mm、约 57°);原因、修正(只改两行 + 同步 install)与修正后三者 0.00 mm 一致;能证明什么、不能证明什么;真机对比与 Dockerfile 修正待办;M2 的假设与局限汇总和完成情况。

（后续每次新的实践进展,继续在这里追加链接）

## 环境

- 开发主机:Ubuntu 24.04.4 LTS
- 目标机型:Dobot Nova 5(实验室硬件,概念上接近 WARA 挑战赛使用的 ABB GoFa)
- 运行环境:Docker 容器(Ubuntu 22.04 + ROS2 Humble),因为 Dobot SDK 官方仅支持这个组合,与主机的 ROS2 Jazzy/Ubuntu 24.04 不兼容
- 图形界面转发:[rocker](https://github.com/osrf/rocker)(处理 X11/XWayland 转发)
- SDK 来源:[Dobot-Arm/DOBOT_6Axis_ROS2_V3](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V3)

## 目录结构

```
WARA-Nova5-Pipeline/
├── README.md                                   本文件
├── 01_docker_environment_and_first_simulation.md   环境搭建 + 首次联动仿真跑通记录
├── 02_moveit_ros2_control_architecture_and_real_hardware_bridge.md   配置体系精读 + 真机执行链路侦破
├── 03_hand_written_action_client_and_demo_debugging.md   手写 Action Client 实战 + 环境排查记录
├── 04_rebuild_image_and_gazebo_verified.md   重建镜像 + Gazebo 链路验证
├── 05_ros2_moveit_ros2_control_communication_flow.md   通信链路与配置体系梳理
├── 06_fk_implementation_and_moveit_compute_fk_verification.md   自己实现 FK + 与 /compute_fk 对比
├── 06_compare_fk_result.png                    第6篇的运行结果截图
├── 07_jacobian_and_singularity.md             Jacobian 与奇异性:原理、构造与用法
├── 08_ik_principles_and_verification.md       IK 原理、实现与验证
├── 08_ik_roundtrip_result.png / 08_compare_ik_result.png   第8篇的运行结果截图
├── 09_model_comparison_and_m2_summary.md      模型对比(vs 仿真器)+ M2 假设与局限
├── nova5_model.py                              FK 实现(URDF 关节链连乘)
├── nova5_kinematics.py                         Jacobian、可操作度、IK(阻尼最小二乘)
├── demo_jacobian_usage.py                      Jacobian 用法演示(正向 / 沿直线 / 奇异附近)
├── ik_roundtrip_simple.py                      IK 往返验证(不需要 ROS)
├── compare_ik_simple.py                        自己的 IK 与 MoveIt /compute_ik 对比
├── compare_fk_simple.py                        自己的 FK 与 MoveIt /compute_fk 对比脚本
├── verify_model_consistency.py                 自己的 FK / MoveIt / Gazebo TF 三方对比(--sim),真机只读对比(--real)
├── my_move_group_client.py.py                  参数化 action client 测试脚本(文件名待修正)
├── Dockerfile                                  最终可用的容器构建文件
└── start_dobot.sh                              一键启动/进入容器脚本
```
