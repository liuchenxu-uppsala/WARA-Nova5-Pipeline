# 第5篇:从 client 到 Gazebo 关节——ROS2 / MoveIt / ros2_control 整条通信链路梳理

第4篇把 Gazebo 链路跑通之后,这一篇把"为什么能跑通"从头理了一遍:我的 action client 发出请求,到 Gazebo 里的关节真的转动,中间经过了哪些进程、哪些配置文件、哪些接口,各自是谁写的、谁负责。本篇是概念梳理,不涉及新的环境改动。

> 说明:文中的 `ros2 ...` 查看命令是根据概念推导整理的,建议在容器里实跑一遍,以实际输出为准。其余内容(两个 yaml、`action_move_server.py` 的逻辑)来自对 SDK 源码的直接阅读。

## 一、整体分层:谁是谁

```
你的脚本 / 任务层节点      move_group (MoveIt)       controller_manager (ros2_control)
        └───────────────────┴───────────────────────────┴──────  全都是 ROS2 节点
                     ROS2 (节点 / 话题 / 服务 / action / 参数 + DDS 通信)
Gazebo:本体不是 ROS2 程序,靠 gazebo_ros 插件(含 libgazebo_ros2_control.so)接入
```

| 层 | 是什么 | 谁写的 |
|---|---|---|
| ROS2 | 通信框架:节点、话题、服务、action、参数,以及让进程互相发现的机制(底层 DDS)。没有中心服务进程 | ROS2 官方 |
| MoveIt(`move_group`) | 运动规划应用,是用 ROS2 库(`rclcpp`)写成的一个节点 | MoveIt 官方 |
| ros2_control(`controller_manager` 及各 controller) | 机器人控制框架,同样是 ROS2 节点 | ros2_control 官方 |
| Gazebo | 物理仿真器,本体非 ROS2,靠插件接入 ROS2 网络 | Gazebo / gazebo_ros 官方 |
| 我的脚本、WARA 任务层 | 应用逻辑 | 我自己 |

关键认知:**`move_group` 不是 ROS2 创建的,是 MoveIt 开发者用 ROS2 的库写的程序**;`ros2 launch` 只负责把进程启动起来。用 `ros2 node list` 能看到所有节点,这就是"ROS2 正在运行"的直接证据。

## 二、ROS2 基础概念

### 话题 / 服务 / Action

| | 话题 Topic | 服务 Service | Action |
|---|---|---|---|
| 模式 | 发布/订阅,单向数据流 | 一问一答 | 目标 + 反馈 + 结果 |
| 过程信息 | 持续流 | 无,只有最终回复 | 有(feedback) |
| 能取消 | 不适用 | 不能 | 能 |
| 适合 | 传感器数据、状态 | 快速查询/设置 | 耗时任务 |
| 本项目例子 | `/joint_states` | `/controller_manager/list_controllers` | `/move_action` |

Action 是在话题和服务之上组合出来的。

### Action 的"名字 + 类型"

```python
ActionClient(self, MoveGroup, 'move_action')
#                  ^类型      ^名字
```

client 在整个 ROS2 网络里找"名字和类型都匹配的 server",**不关心对方是哪个进程**。`wait_for_server()` 等的就是这个发现过程。

### 参数与 yaml 类型

yaml 里的值会作为 ROS2 参数传给节点,**类型按写法自动推断**:`true`→bool,`100`→int,`0.5`→double,`"arm"`→string,列表→数组(**数组元素类型必须一致**);每个键是独立参数,嵌套键的参数名用点连接(`foo.bar`)。

## 三、启动阶段:谁在什么时候创建了什么

**① `ros2 launch dobot_gazebo gazebo_moveit.launch.py`**

1. `gzserver` 启动,加载 `libgazebo_ros2_control.so`,由它创建 `controller_manager`
2. `controller_manager` 读 `ros2_controllers.yaml`,创建两个 controller 实例:`joint_state_broadcaster`、`nova5_group_controller`
3. `nova5_group_controller` 被创建时,由它**类的代码**自己开一个 action server `nova5_group_controller/follow_joint_trajectory`
4. `list_controllers` 显示两个都是 `active`,即这一步完成

**② `ros2 launch dobot_moveit moveit_gazebo.launch.py`**

1. 启动 `move_group` 进程(MoveIt 官方的可执行文件)
2. 读 SRDF(规划组、`home` 姿态)、`kinematics.yaml`、`moveit_controllers.yaml`
3. 自己创建 `/move_action` 这个 action server,对外接请求

> `joint_state_broadcaster` 是一个 **controller**(把硬件状态发布成 `/joint_states` 话题),不是 action。在 ros2_control 里,yaml 里写了某个 controller **不等于它已经在运行**,还需要被"加载并激活",这一步通常由 launch 里的 `spawner` 完成。第3篇里出现过包没装好导致自动加载失败、需要手动补的情况。

## 四、两个 yaml:各给谁看

两个互相不认识的程序,各自要一份配置:

| 文件 | 谁读 | 作用 |
|---|---|---|
| `ros2_controllers.yaml` | `controller_manager`(ros2_control 一侧) | 创建哪些 controller 实例 |
| `moveit_controllers.yaml` | `move_group`(MoveIt 一侧) | 执行轨迹时去找哪个 controller |

**`ros2_controllers.yaml`**

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    nova5_group_controller:                                  # ← 实例名
      type: joint_trajectory_controller/JointTrajectoryController   # ← 类型(包名/类名)

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

nova5_group_controller:                                      # ← 同一个实例名,这块是它自己的参数
  ros__parameters:
    joints: [joint1, joint2, joint3, joint4, joint5, joint6]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
```

**`moveit_controllers.yaml`**

```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - nova5_group_controller          # ← 同一个实例名

  nova5_group_controller:
    type: FollowJointTrajectory       # ← action 的类型
    action_ns: follow_joint_trajectory   # ← action 的后缀
    default: true
    joints: [joint1, joint2, joint3, joint4, joint5, joint6]
```

**对照关系:**

| 内容 | `ros2_controllers.yaml` | `moveit_controllers.yaml` |
|---|---|---|
| 实例名 | `nova5_group_controller` | `nova5_group_controller`(**必须一致**) |
| 6 个关节 | `joints` | `joints`(**必须一致**) |
| `type` | 实例的**类**:`JointTrajectoryController` | **action 的类型**:`FollowJointTrajectory`(同名不同义) |
| action 后缀 | 文件里没写,由类的代码写死 | `action_ns`,要和代码写死的一致 |
| `joint_state_broadcaster` | 有 | 没有(它只发布状态,不执行轨迹) |

两个文件**不会互相读取,也没有程序检查它们一致**,靠"同一个名字"对上。`move_group` 把 `/` + 实例名 + `/` + `action_ns` 拼起来去连 server。

### 容易混淆的几点

- **斜杠的两种含义**:`joint_trajectory_controller/JointTrajectoryController`(类型)里,斜杠前是**包名**、后是**类名**;`nova5_group_controller/follow_joint_trajectory`(action 名)里,斜杠前是**实例名**、后是 **action 后缀**。
- **action server 是谁创建、名字怎么来的**:实例名由 yaml 决定,后缀 `follow_joint_trajectory` 由 `JointTrajectoryController` 类的代码写死,两者拼成最终名字。不是任何 yaml 里"创建"了 action。
- **`moveit_controllers.yaml` 里的字段不是通信数据**,是 `move_group` 的配置:`type` 决定创建哪种 action client,`action_ns` 参与拼名字,`joints` 用来判断这个 controller 能不能执行当前轨迹,`default` 决定有多个候选时选谁。真正通信的数据是 `FollowJointTrajectory` 的 Goal 消息(`joint_names` + 一串带 `time_from_start` 的路点)。
- **规划组名 vs controller 名**:`nova5_group`(SRDF 里的规划组)和 `nova5_group_controller`(controller 实例)前缀相同只是命名习惯,分别活在规划阶段和执行阶段。

## 五、`/move_action`:我的 client 与 `move_group` 交换什么

`MoveGroup.action` 三段:

- **Goal**(我发给它):`MotionPlanRequest`(去哪、哪个规划组、速度加速度比例、容差)+ `PlanningOptions`(只规划还是规划加执行、场景怎么处理)
- **Feedback**:一个状态字符串
- **Result**:`error_code`(脚本里读的 `error_code.val`)、规划出的轨迹、规划耗时等

**`move_group` 收到请求后做什么:**

1. 从 `/joint_states` 取机械臂当前状态作为起点(`start_state.is_diff = True` 就是这个意思;第3篇 `-4` 报错里的 `Didn't receive robot state` 就是这一步失败)
2. 把 `planning_scene_diff` 叠加到它维护的规划场景上
3. 调用规划管线算出一条无碰撞轨迹(默认规划器通常是 OMPL,具体以 `nova5_moveit/config` 为准)
4. `plan_only=False` 时,按 `moveit_controllers.yaml` 创建 `FollowJointTrajectory` 的 client,把轨迹发给 `nova5_group_controller/follow_joint_trajectory`
5. 把 `error_code` 放进 Result 回给我

**怎么知道该找哪个 server、传什么:** 没有配置文件告诉你,靠 MoveIt 官方文档加类型定义本身:

```bash
ros2 action list -t                                   # 有哪些 action、各是什么类型
ros2 interface show moveit_msgs/action/MoveGroup      # Goal/Result/Feedback 每个字段
ros2 node info /move_group                            # 这个节点提供哪些 action server
```

类型匹配只保证消息格式一致,语义要靠文档确认。规划组名看 SRDF,关节名看 URDF/SRDF。

## 六、轨迹怎么到达 Gazebo 里的关节

数据不是"发给 Gazebo",而是经过三层:

1. **`move_group` → controller**:`move_group` 内部的 `FollowJointTrajectory` client,把轨迹(6 个关节的位置 + `time_from_start`)发给 `nova5_group_controller/follow_joint_trajectory`。**这个 server 在 `controller_manager` 里**(Gazebo 模式下 `controller_manager` 又在 `gzserver` 进程里),`move_group` 里只有 client 端。
2. **controller 按时间插值**:`nova5_group_controller` 在 `controller_manager` 里以 `update_rate: 100` Hz 循环,每个周期按当前时间在轨迹上插值,把此刻各关节应在的位置写入 **command interface**(`ros2_control` 里的指令槽)。
3. **硬件接口交给仿真**:`gazebo_ros2_control/GazeboSystem`(由 URDF/xacro 里的 `<ros2_control>` 声明触发)每个周期读 command interface,施加到 Gazebo 里的关节;同时把关节的真实位置/速度写回 **state interface**。这一步发生在 `gzserver` 进程内部,不走 ROS 话题。

**反馈回路:** `joint_state_broadcaster` 读 state interface,发布 `/joint_states`;`move_group` 靠它知道当前状态,controller 靠它判断是否到位并返回 Result。

```
我的 client ──/move_action──> move_group (规划)
   └─ FollowJointTrajectory ─> JointTrajectoryController (在 controller_manager 里)
        └─ command interface ─> GazeboSystem ─> Gazebo 物理引擎
        <─ state interface ─── GazeboSystem
        └─ joint_state_broadcaster ──> /joint_states ──> move_group
```

## 七、真机链路:`action_move_server.py` 的逻辑

Dobot 真机**不启动 `controller_manager`**(`controller_manager` 不是 ROS2 或 MoveIt 强制要求的,只要有人在这个名字上提供类型匹配的 action server,`move_group` 就不关心是谁)。取而代之的是 `dobot_moveit/dobot_moveit/action_move_server.py`(约 85 行 Python):

1. **冒充 server**:创建 action `/{DOBOT_TYPE}_group_controller/follow_joint_trajectory`(`nova5` 时即 `/nova5_group_controller/follow_joint_trajectory`),类型也是 `FollowJointTrajectory`,与标准 controller 完全一致。
2. **转换**:遍历 `trajectory.points`,把每个路点的 6 个关节角从弧度转成角度(`180 * ii / 3.14159`)。
3. **逐点发给机械臂**:对每个路点调用 Dobot 自己的 `ServoJ` 服务(`/dobot_bringup_v3/srv/ServoJ`,`t = 0.2`),然后 `time.sleep(0.18)`,再发下一个。

**与标准 controller 的差别(弱点):**

- 节奏靠固定的 `sleep(0.18)`,**不使用**轨迹里每个点的 `time_from_start`,规划好的速度曲线被忽略
- `call_async` 之后**不等响应**(等待代码被注释),不知道有没有到位
- `execute_callback` 执行完直接 `succeed()`、`error_code = 0` **写死**,哪怕实际失败 `move_group` 也认为成功
- 不经过 `ros2_control`,没有 command/state interface,指令直接走 Dobot 自己的服务

这条链路**没有闭环**,正是第2篇提出"规划路径 vs 实际路径跟踪误差对比"这个分析方向的依据。`move_group` 仍然需要有人发布 `/joint_states`,真机上由 Dobot 驱动侧节点负责(具体节点用 `ros2 topic info /joint_states` 确认)。

三条链路汇总(承接第3篇):

| 模式 | `controller_manager` | 硬件接口 | 提供 `follow_joint_trajectory` server 的是 |
|---|---|---|---|
| demo | 启动 | `mock_components/GenericSystem`(纯数学) | controller(官方) |
| Gazebo | 启动(在 `gzserver` 内) | `gazebo_ros2_control/GazeboSystem` | controller(官方) |
| 真机 | **不启动** | 不适用 | `action_move_server.py`(Dobot 自写) |

## 八、哪些要自己写,哪些是现成的

| 东西 | 谁的代码 | 我需要做什么 |
|---|---|---|
| `move_group` | MoveIt 官方 | 提供机器人描述和配置,不写代码 |
| `controller_manager` | ros2_control 官方 | 写 `ros2_controllers.yaml` |
| `nova5_group_controller` | 官方类 `JointTrajectoryController` 的一个实例 | yaml 里起名字、指定关节 |
| 仿真硬件接口 | 官方(`GazeboSystem` / `GenericSystem`) | 不用写 |
| 真机硬件接口 | 要么写 ros2_control 硬件接口插件(官方标准做法),要么像 Dobot 一样自写 action server | 需要自己写 |
| 应用逻辑(WARA pipeline) | 自己 | 需要自己写 |

通用的规划、控制框架是现成的,自己写的是**"这台机器人是什么样"的描述、"这台机器人怎么跟硬件对话"、以及自己的任务逻辑**。

**新增一个 action 的标准做法**(不需要碰 `move_group` 和 controller 的 yaml;yaml 只是配置,不能"创建 action"):

1. 写 `.action` 文件定义 Goal/Result/Feedback,放在接口包里
2. 编译,生成类型
3. 写自己的节点实现 action server 的逻辑
4. 用 launch 启动
5. 其他节点用 action client 调用

## 九、WARA 任务层的位置

任务层节点是 `move_group` **上面**另外独立的一个节点,对 `move_group` 来说就是又一个 client:

```
任务层节点(自己写,对外提供 action,如"抓取物体")
      │  作为 client 调用
      ▼
move_group(/move_action)
      ▼
controller / 硬件
```

它做的事:接收物体位姿 → 转到 `base_link` 坐标系 → 计算预抓取/抓取/撤离姿态 → 逐个交给 `/move_action` → 检查每步 `error_code`,失败则重试或上报 → 通过 Feedback 汇报阶段。

需要的数据:物体位姿(视觉或用户输入)、坐标系关系(**TF**)、`/joint_states`、URDF/SRDF(末端连杆 `Link6`)、规划场景中的障碍物。

现在的脚本发的是**关节空间目标**(`JointConstraint`);任务层给出的通常是**笛卡尔位姿**,请求里要换成 `PositionConstraint` + `OrientationConstraint`(针对 `Link6`),或者先用 `/compute_ik` 服务求出关节角再发。这是从"手写 action client"走向 WARA pipeline 的下一步。

## 十、常用查看命令

```bash
ros2 node list                                  # 所有 ROS2 节点
ros2 node info /move_group                      # 某节点提供/订阅的 action、话题、服务
ros2 action list -t                             # 所有 action 及类型
ros2 action info /nova5_group_controller/follow_joint_trajectory   # 谁是 server、谁是 client
ros2 interface show moveit_msgs/action/MoveGroup
ros2 interface show control_msgs/action/FollowJointTrajectory
ros2 control list_controllers                   # controller 状态
ros2 control list_hardware_interfaces           # command/state interface
ros2 topic echo /joint_states --once            # 当前关节状态
ps aux | grep -E "move_group|gzserver|rviz"     # 对应的操作系统进程
```

## 十一、下一步

- 在容器里实跑第十节的查看命令,把实际输出和本篇对照,修正不准确之处
- 跑 `my_move_group_client.py` 的同时看 `ros2 node list`,确认 client 节点出现在同一个网络里
- 把第2篇提出的"规划 vs 实际轨迹跟踪误差分析"放到任务层之下做第一版原型(逐关节 RMSE)
- 从关节空间目标过渡到笛卡尔位姿目标(`PositionConstraint` / `OrientationConstraint` 或 `/compute_ik`)
