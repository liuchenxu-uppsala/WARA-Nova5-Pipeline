# 第4篇:重建镜像 + Gazebo 链路验证跑通

第3篇结尾留下的问题是:Gazebo 模式下控制器加载失败(`Loader for controller 'nova5_group_controller' not found`),怀疑是容器用的是旧版 `Dockerfile` 构建的、之后从没重新 `build`。这一篇记录用当前 `Dockerfile` 彻底重建镜像之后,这个怀疑被证实,以及 Gazebo 链路从"控制器是否加载"到"机械臂是否真的到位"的完整验证。

## 一、重建镜像前的两处改动

**1. `Dockerfile`:显式补上 `ros-humble-joint-state-broadcaster`**

之前依赖 `ros2-controllers` 这个 metapackage 间接带入,结果在旧镜像里实际没装上。这次直接在 `apt-get install` 列表里写明,不再依赖隐式传递依赖。

**2. `start_dobot.sh`:加共享目录挂载**

```bash
rocker --x11 --user \
    --volume "$HOME/dobot_shared":"/dobot_ws/shared" \
    --name dobot_dev dobot_nova5:humble
```

主机的 `~/dobot_shared` 对应容器里的 `/dobot_ws/shared`,脚本放主机这个目录,容器里直接能看到,不用再 `docker cp`。

## 二、一个小插曲:`dobot_dev` 根本不存在

重建前按计划要先 `docker rm dobot_dev`,结果报 `No such container`。`docker ps -a` 显示只有两个随机名字的已退出容器(`friendly_hodgkin`、`agitated_elbakyan`),`COMMAND` 分别是 `apt-get…` 和 `rosdep …`,对应 `Dockerfile` 里的两条 `RUN`。

**结论:这是之前某次 `docker build` 在这两步失败时留下的调试残留容器**,和 `dobot_dev` 无关,可以 `docker rm` 清掉,不影响重建。既然没有同名容器,`start_dobot.sh` 直接走"首次创建"分支。

教训:`start_dobot.sh` 的逻辑是"同名容器存在就复用",**重建镜像之后,如果旧的 `dobot_dev` 还在,必须先 `docker rm` 掉,否则永远是旧镜像**。这正是第3篇里"Dockerfile 和容器状态对不上"的根源。

重建步骤:

```bash
docker build -t dobot_nova5:humble .
mkdir -p ~/dobot_shared
chmod +x start_dobot.sh
./start_dobot.sh
```

进容器后的两项验证:

```bash
ros2 pkg list | grep joint_state_broadcaster   # 能搜到 → 包装上了,环境变量也正常
ls /dobot_ws/shared                            # 在主机放个文件,容器里能看见 → 挂载生效
```

两项都通过。

## 三、Gazebo 链路验证

### 启动(Dobot 官方自带的两个 launch 文件)

```bash
# 终端1:Gazebo + 控制器
ros2 launch dobot_gazebo gazebo_moveit.launch.py

# 终端2:MoveIt + RViz(再开一个终端,./start_dobot.sh 会 docker exec 进同一个容器)
ros2 launch dobot_moveit moveit_gazebo.launch.py
```

`DOBOT_TYPE=nova5` 已写进容器全局 bashrc,不用手动 export。

### 验证 1:controller 是否加载

```
$ ros2 control list_controllers
joint_state_broadcaster JointStateBroadcaster                          active
nova5_group_controller  joint_trajectory_controller/JointTrajectoryController  active
```

两个都是 `active`。第3篇里 `controller=[]` / `Loader ... not found` 的问题消失,**证实根因是旧镜像缺包,不是 Gazebo 模式本身的路径问题**。

> 命令开头的 `waiting for service ... to become available` 只是 `controller_manager` 还在起,不是错误。

### 验证 2:手写 action client 执行

```
$ python3 my_move_group_client.py
目标已接受,等待结果...
error_code.val = 1  (1 表示 SUCCESS)
```

注意:`SUCCESS` 只说明 MoveIt 侧认为成功,不等于物理仿真里真的到位。

### 验证 3:对照 `/joint_states` 确认真的到位

注意 `/joint_states` 里关节顺序是 `joint2, joint3, joint1, ...`,**必须按名字对应,不能按位置**。

| 关节 | 目标 | 实际 | 误差 |
|---|---|---|---|
| joint1 | 0.3 | 0.3025 | 0.0025 |
| joint2 | 0.2 | 0.2069 | 0.0069 |
| joint3 | -0.8 | -0.7980 | 0.0020 |
| joint4 | 0.1 | 0.0907 | 0.0093 |
| joint5 | 1.0 | 1.0076 | 0.0076 |
| joint6 | -0.2 | -0.2082 | 0.0082 |

- 误差都在 0.01 rad(约 0.5°)以内,正好是代码里 `JointConstraint` 设的容差,正常
- `velocity` 量级 1e-13,说明已停稳
- `effort` 是 `nan`:Gazebo 没上报力矩,不是问题
- `A message was lost!!!` 是 `ros2 topic echo --once` 常见提示,不影响结果

**结论:demo / Gazebo / 真机三条链路里,Gazebo 这条验证完成。**

## 四、测试命令与脚本

```bash
# controller 状态(应有 2 个,都是 active)
ros2 control list_controllers

# 当前关节角
ros2 topic echo /joint_states --once

# 手写 action client(放进 ~/dobot_shared,容器里在 /dobot_ws/shared/ 下)
python3 /dobot_ws/shared/my_move_group_client.py --verify
python3 /dobot_ws/shared/my_move_group_client.py --home --verify
python3 /dobot_ws/shared/my_move_group_client.py --joints 0.3 0.2 -0.8 0.1 1.0 -0.2 --verify
```

`scripts/my_move_group_client.py` 是第3篇 action client 的参数化版本,支持 `--home` / `--joints` / `--plan-only` / `--verify`。`--verify` 执行后按关节名对比 `/joint_states`(即上面"验证 3"那张表),超差会标出,退出码 0/1 对应是否全部到位。

> 这个脚本是在没有 ROS 环境的机器上写的,只做了语法检查,还没在容器里实跑过。

## 五、下一步

- 在容器里实跑 `my_move_group_client.py --verify`,确认无误
- 用 Gazebo 看第3篇提到的、demo 模式看不出来的问题:关节有没有超调/震荡
- 用更多自定义关节角测试,排除"只对预设值有效"
- 开始"规划 vs 实际"轨迹跟踪误差分析的第一版原型(逐关节 RMSE)
