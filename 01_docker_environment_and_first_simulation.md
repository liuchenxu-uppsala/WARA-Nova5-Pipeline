# 01 - Docker 环境搭建 + RViz/MoveIt/Gazebo 联动仿真跑通记录

**日期**:2026-09-29
**目标**:按照导师徐志的要求,practice Dobot 官方的 `DOBOT_6Axis_ROS2_V3` ROS2 SDK(基于 Nova 5 机械臂),先在仿真环境里跑通"RViz 规划 → MoveIt 计算 → Gazebo 执行"这一整条闭环,为后续接入 WARA 项目的 planning-to-execution pipeline 打基础。

---

## 背景:系统版本冲突

- 本机系统:**Ubuntu 24.04.4 LTS**
- Dobot SDK 官方要求:**Ubuntu 22.04 + ROS2 Humble**
- 两者不兼容(Ubuntu 24.04 官方配套的是 ROS2 Jazzy,官方也没有为 24.04 发布 Humble 的 deb 包)

**解决方案**:不重装系统、不装虚拟机,用 **Docker** 起一个 Ubuntu 22.04 + ROS2 Humble 的容器,把 SDK 装在容器里跑,主机系统保持不动。

---

## 环境搭建过程(遇到的问题按顺序记录)

### 1. 基础工具安装

主机上装 Docker 和 [rocker](https://github.com/osrf/rocker)(OSRF 维护的小工具,专门用来给 `docker run` 自动补全图形界面转发、GPU、用户权限等参数,不需要手写一长串 `-e DISPLAY=... -v /tmp/.X11-unix:...`):

\`\`\`bash
sudo apt install -y docker.io
sudo usermod -aG docker $USER
pip3 install rocker --break-system-packages
\`\`\`

### 2. 第一版 Dockerfile

基于官方 `osrf/ros:humble-desktop` 镜像,clone SDK 源码,`rosdep install` 装依赖,`colcon build` 编译。

### 3. 问题①:`rosdep install` 报 "Unable to locate package"

一长串 `ros-humble-moveit-*`、`ros-humble-gazebo-*` 等包全部找不到。

**原因**:官方基础镜像为了减小体积,清空了 apt 的包列表缓存,而 Dockerfile 里直接执行 `rosdep install`(内部调用 `apt-get install`)之前没有先 `apt-get update` 重新拉取包列表。

**修复**:在 `rosdep install` 之前加一行 `apt-get update`。

### 4. 问题②:`ros-humble-warehouse-ros-mongo` 装不上

**原因**:这个包依赖 MongoDB,而 Ubuntu 22.04(Jammy)官方源因为 license 原因不再提供完整的 MongoDB 依赖包,这是一个业界已知的通用坑,不是本项目特有问题。

**判断**:这个包只是 MoveIt 里一个可选的"把规划场景存进数据库"插件,跟运动规划本身、仿真、真机控制完全无关,连 MoveIt 官方教程也经常直接跳过。

**修复**:`rosdep install` 加 `--skip-keys="warehouse_ros_mongo"` 跳过这一个依赖。

### 5. 问题③:`Permission denied`,普通用户读不了 `/root/dobot_ws`

**原因**:`rocker --user` 会用主机当前用户的身份(非 root)进入容器,而工作空间之前建在 `/root/dobot_ws` 下,`/root` 目录默认只有 root 自己能访问。

**修复**:把工作空间路径从 `/root/dobot_ws` 改成 `/dobot_ws`(不在任何用户的私有 home 目录下),并加一行 `chmod -R a+rwX /dobot_ws` 保证所有用户可读写。

### 6. 问题④:环境变量没有自动加载(`ros2 pkg list` 找不到任何 dobot 相关包)

**原因**:Dockerfile 里把 `source .../setup.bash` 写进了 **root 用户的 `~/.bashrc`**,但 `rocker --user` 进容器时用的是另一个用户(跟主机同名,例如 `desheng`),这个用户有自己独立的 home 目录和 `.bashrc`,不会读到 root 的那份配置。

**修复**:改成写进**全局**的 `/etc/bash.bashrc`(所有用户共享),不再写某个特定用户的 home 目录下。

### 7. 问题⑤:`DOBOT_TYPE` 环境变量在新终端里丢失

现象:第一个终端 `export DOBOT_TYPE=nova5` 之后能正常跑,但用 Terminator 分屏或者新开一个终端进同一个容器时,这个变量又消失了(报错里 URDF 文件名变成 `None_robot.xacro`)。

**原因**:`export` 只在当前这一个终端会话里生效,不会被新开的终端继承。

**修复**:同样把 `export DOBOT_TYPE=nova5` 写进 `/etc/bash.bashrc`,让它对任何新终端都自动生效,不用每次手动 `export`。

### 8. 问题⑥:`ros2 control` 命令不认识

\`\`\`
ros2: error: ... invalid choice: 'control' (choose from 'action', 'bag', ...)
\`\`\`

**原因**:`ros2 control` 这整个命令族(`list_controllers`、`load_controller` 等,不管具体哪个子命令)来自一个额外的扩展包 `ros2controlcli`,这个包没有被安装,所以 `ros2` 命令行工具完全不认识 `control` 这个词。

**修复**:

\`\`\`bash
sudo apt-get install -y ros-humble-ros2controlcli
\`\`\`

### 9. 问题⑦(最隐蔽的一个):控制器加载失败,MoveIt 点 Execute 后 Gazebo 毫无反应

装好 `ros2controlcli` 后,`ros2 control` 命令能跑了,但加载具体控制器时报错:

\`\`\`
[ros2-6] Error loading controller, check controller_manager logs
[ERROR] [ros2-6]: process has died [... cmd 'ros2 control load_controller --set-state active nova5_group_controller']
\`\`\`

`controller_manager` 日志显示它当前"认识"的控制器类型只有几个内部测试用的假控制器(`test_controller_failed_activate` 等),说明真正需要的轨迹控制器插件类型根本没有被注册。

**原因**:`nova5_group_controller` 依赖的插件包 `joint_trajectory_controller`,没有被 SDK 仓库的依赖声明列出来(rosdep 依赖清单里漏掉了),所以从来没被装过。

**修复**:

\`\`\`bash
sudo apt-get install -y ros-humble-joint-trajectory-controller ros-humble-ros2-controllers
\`\`\`

装完后重新跑 launch,查询确认两个控制器都变成 `active`:

\`\`\`bash
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers "{}"
\`\`\`

---

## 最终版 Dockerfile

（见同目录下 `Dockerfile` 文件）

## 一键启动脚本

（见同目录下 `start_dobot.sh` 文件)

---

## 功能验证:四种仿真模式逐一测试

| 模式 | 命令 | 观察到的现象 |
|---|---|---|
| 纯 RViz 展示 | `ros2 launch dobot_rviz dobot_rviz.launch.py` | 能看到 Nova5 模型,但没有滑块、不能交互(用的是不带 GUI 的 `joint_state_publisher`,只发布默认姿态) |
| MoveIt 假执行 | `ros2 launch dobot_moveit moveit_demo.launch.py` | RViz 内加载 MotionPlanning 插件,可以拖拽末端交互标记、Plan and Execute,只在 RViz 内播放动画,不经过物理仿真 |
| 纯 Gazebo | `ros2 launch dobot_gazebo dobot_gazebo.launch.py` | Gazebo 物理世界+机械臂模型,但没有加载控制器,无法交互(`controller_manager/list_controllers` 查询结果为空) |
| Gazebo + MoveIt 联动 | 终端1:`export DOBOT_TYPE=nova5 && ros2 launch dobot_gazebo gazebo_moveit.launch.py`<br>终端2:`ros2 launch dobot_moveit moveit_gazebo.launch.py` | **完整闭环跑通**:RViz 里拖拽目标位姿、Plan & Execute,Gazebo 窗口里机械臂真实地按物理仿真执行了对应动作 |

### 关于 Gazebo/MoveIt/RViz 三者关系的理解(过程中厘清的关键概念)

- **RViz 不是纯粹的展示工具**,它是承载 MoveIt 交互插件的界面本体——"MoveIt 的图形界面"这个说法不准确,MoveIt 没有独立窗口,看到的操作界面就是 RViz + MotionPlanning 插件
- **Gazebo 是真正的物理仿真引擎**(计算重力、碰撞、惯性),不只是画面展示,是没有真机时的"身体"替代品
- **`ros2_control`(`controller_manager` + 具体控制器)是 MoveIt 和 Gazebo/真机之间的标准翻译层**:MoveIt 算出抽象的关节角度轨迹,控制器负责把这份轨迹转换成对执行端(仿真或真机)的具体驱动指令
- 两个 launch 文件(`dobot_gazebo/gazebo_moveit.launch.py` 只负责准备"身体"+控制器接口;`dobot_moveit/moveit_gazebo.launch.py` 才是真正启动 `move_group`+RViz 的"大脑"部分)需要分两个终端配合跑,命名容易让人误以为单个文件就包含了全部内容,实际读代码验证后才确认清楚

---

## 排查过程中额外发现:CPU 占用高、风扇狂转

跑 Gazebo 联动时观察到 22 核 CPU 全部维持在 35%~50%。

**原因分析**:Gazebo 终端日志里有 `libGL error: failed to load driver: iris`,说明容器内没能正确调用主机 Intel 核显的硬件加速,Gazebo 被迫退化成纯 CPU 软件渲染(llvmpipe),叠加物理仿真计算本身的负载,导致多核占用偏高。这是一个已识别但**尚未解决**的优化项(需要正确配置容器访问 `/dev/dri` 显卡设备),不影响功能正确性,留作后续优化。

---

## 今天的最终成果

成功在仿真环境里完整跑通:**RViz 拖拽设定目标位姿 → MoveIt 完成逆运动学求解 + 无碰撞路径规划 → 通过 `ros2_control` 标准接口发送轨迹 → Gazebo 物理仿真真实执行动作**,并且在整个过程里，通过实际阅读源码、而非仅凭猜测，厘清了 SDK 里各个 launch 文件的真实职责边界。

## 下一步计划

1. 修复 Gazebo 的 GPU 硬件加速问题(降低 CPU 负载)
2. 把"RViz 手动拖拽目标位姿"这一步，改成用 Python(`MoveGroupInterface`)编写代码，自动向 MoveIt 提供目标位姿——这是从"手动演示"过渡到真正的 planning-to-execution pipeline 的第一步
3. 学习 MoveIt 的 `compute_cartesian_path` 接口(用于抓取任务里"沿直线靠近/撤离"这类精确路径需求)
4. 规划场景(Planning Scene)物体注册,让避障真正考虑实际环境中的障碍物
