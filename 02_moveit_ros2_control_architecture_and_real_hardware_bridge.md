# 第2篇:MoveIt / ros2_control 配置体系精读 + 真机执行链路完整侦破

延续第1篇搭好的 Docker + ROS2 Humble 环境,这一篇记录的是对 `nova5_moveit/config/` 配置文件体系的逐个精读,以及围绕"MoveIt 到底是怎么控制到真实 Nova5 机械臂的"这个问题,展开的一次完整的源码侦查过程——包括中途一次被推翻又重新验证的错误结论,最终定位到真正的执行桥梁代码。

## 一、目标

在跑通仿真闭环(RViz → MoveIt → Gazebo)之后,想搞清楚两件事:

1. `nova5_moveit/config/` 里一堆配置文件,各自到底在管什么
2. MoveIt 规划出来的轨迹,最终是怎么让**真实**的 Nova5 机械臂动起来的——这条链路具体长什么样

## 二、MoveIt 配置文件体系(逐个过一遍)

| 文件 | 作用 |
|---|---|
| `nova5_robot.srdf` | 定义 planning group(哪些关节算一组,由 MoveIt 统一规划/控制)、末端执行器(挂在最后一节连杆上的可选部件声明)、自碰撞检测豁免矩阵(Setup Assistant 通过随机采样**离线算好、写死存下**,不是运行时实时算的;运行时只对未豁免的连杆对做碰撞检测) |
| `kinematics.yaml` | 指定用哪种 IK 求解器(通用数值解法如KDL,或者更快的解析解法),是可配置项,不是自动决定的 |
| `joint_limits.yaml` | MoveIt 自己用的每个关节速度/加速度上限(可能比 URDF 原始限制更保守),供默认的 OMPL 规划器做轨迹时间参数化 |
| `moveit_controllers.yaml` | MoveIt 的"地址簿"——告诉 `move_group`,某个 planning group 规划完了,要把轨迹发到哪个 controller 名字、走哪种 action 接口 |
| `ros2_controllers.yaml` | `controller_manager` 的配置——声明有哪些 controller、什么类型、各自管哪些关节、command/state interface 是什么 |
| `moveit.rviz` | 纯 UI 层面的 RViz 配置存档(打开哪些面板、什么参数),不影响任何计算逻辑 |
| `pilz_cartesian_limits.yaml` | Pilz 规划器专用,限制的是**末端在笛卡尔空间**的线速度/线加速度/角速度(区别于 `joint_limits.yaml` 限制的是逐关节角速度) |
| `nova5_robot.ros2_control.xacro` | 声明 ros2_control 的硬件接口插件——**关键发现见下文** |

### `ros2_controllers.yaml` 实际内容(核对过的原文)

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    nova5_group_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

nova5_group_controller:
  ros__parameters:
    joints: [joint1, joint2, joint3, joint4, joint5, joint6]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
```

`joints` 决定这个 controller 管哪些关节;`command_interfaces` 决定往硬件"发送"什么类型的指令(这里只发位置);`state_interfaces` 决定能"读回"什么类型的反馈。

### `moveit_controllers.yaml` 实际内容(核对过的原文)

```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - nova5_group_controller
  nova5_group_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints: [joint1, joint2, joint3, joint4, joint5, joint6]
```

`type` + `action_ns` 拼出最终地址:`<controller_names[0]>/<action_ns>` = **`nova5_group_controller/follow_joint_trajectory`**——`move_group` 发轨迹时,只认这个字符串地址,不关心背后到底是谁在接收。

**关键认知**:`moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager` 说明 MoveIt 自己有一套独立于 ros2_control 之外的"controller manager"概念,它只按"地址+消息类型"匹配,不会校验对方是不是真的符合 ros2_control 规范——这一点是后面"真机桥"能够成立的根本原因。

## 三、一次被推翻又重新验证的结论:真机到底支不支持 MoveIt 控制

### 第一次判断(过快,后来证明不完整)

在 `nova5_robot.ros2_control.xacro` 里看到:

```xml
<hardware>
    <!-- By default, set up controllers for simulation. This won't work on real hardware -->
    <plugin>mock_components/GenericSystem</plugin>
</hardware>
```

一开始据此判断"真机执行链路缺失"。后来意识到这句注释其实是 **MoveIt Setup Assistant 生成配置包时的标准模板文字**,不是 Dobot 专门写的声明,不能直接当作"厂商放弃真机支持"的证据,于是回去做了更完整的验证:

```bash
grep -rn "hardware_interface::SystemInterface" .   # 全仓库搜索,无结果
grep -rn "<plugin>" .                              # 全仓库搜索硬件插件声明
```

结果发现:全部 8 个机型(nova5/nova2/cr3/cr5/cr7/cr10/cr12/cr16)的 `xxx_moveit/config/xxx_robot.ros2_control.xacro`,**无一例外**全部是 `mock_components/GenericSystem`;而 `cra_description/urdf/xxx_robot.xacro` 里则**全部**声明了 `gazebo_ros2_control/GazeboSystem`——这说明"仿真硬件接口"是完整的,"真机硬件接口(标准 ros2_control 意义上的)"确实一个都没有。

### 真正的答案:真机执行链路是存在的,只是没有走标准 ros2_control 路线

后来在 Dobot 官方 README 里一段之前没细看的真机操作截图中,发现终端日志里有一个叫 `cr_robot_ros2_node` 的节点在持续发送/接收 `ServoJ` 指令——这跟已知的任何节点名字都对不上,顺着这条线,下载完整源码逐个排查后,找到了真正的桥梁:

**`dobot_moveit/dobot_moveit/action_move_server.py`**(之前完全没注意到这个包底下还藏着源码,一直误以为 `dobot_moveit` 只是个 launch 分发包)

```python
class FollowJointTrajectoryServer(Node):
    def __init__(self):
        super().__init__('dobot_group_controller')
        name = os.getenv("DOBOT_TYPE")
        self._action_server = ActionServer(
            self, FollowJointTrajectory,
            f'/{name}_group_controller/follow_joint_trajectory',   # 跟 moveit_controllers.yaml 拼出来的地址完全一致
            self.execute_callback)
        self.ServoJ_l = self.create_client(ServoJ, '/dobot_bringup_v3/srv/ServoJ')

    def execution_trajectory(self, trajectory):
        for point in trajectory.points:
            joint_deg = [180 * p / 3.14159 for p in point.positions]   # 弧度转角度
            self.ServoJ_C(*joint_deg)
            time.sleep(0.18)   # 固定间隔发送,忽略轨迹自带的时间戳/速度信息
```

**它做的事情**:在 `nova5_group_controller/follow_joint_trajectory` 这个地址上,**手写创建一个 ROS2 action server,冒充标准 ros2_control controller 该在的位置**。因为这条链路里 `controller_manager` 从未被启动(见下方真机 launch 文件),这个地址上没有任何"正规军"跟它竞争,`move_group` 发出的消息因此百分之百会被它收到。收到轨迹后,它把每个点的弧度转角度,依次调用 `dobot_bringup_v3` 的 `ServoJ` 服务,通过 TCP 私有协议发给真机。

### 真机启动链路完整图(已核实全部文件)

```
dobot_moveit/launch/dobot_moveit.launch.py  (读取 DOBOT_TYPE,同时启动下面两组)
│
├─ dobot_moveit/launch/dobot_joint.launch.py
│    ├─ action_move_server   (占住 follow_joint_trajectory 地址,收到轨迹→转ServoJ→发真机)
│    └─ joint_states.py      (订阅 /joint_states_robot → 转发成标准 /joint_states)
│
└─ {DOBOT_TYPE}_moveit/launch/dobot_moveit.launch.py
     ├─ robot_state_publisher
     ├─ move_group            (全程没有启动 controller_manager!)
     └─ rviz2

另需单独启动:
ros2 launch dobot_bringup_v3 dobot_bringup_ros2.launch.py
     ├─ dobot_bringup   (TCP 29999/30003端口,提供 EnableRobot/MovJ/ServoJ 等近70个服务)
     └─ feedback        (TCP 30004端口,100Hz轮询真机状态,发布 /joint_states_robot)
```

**没有一处启动了 `controller_manager`**,所以 `ros2_controllers.yaml` 和 `mock_components/GenericSystem` 这份配置,在真机链路里其实是"活在文件里、从未被加载运行"的死配置,不影响任何事——真正干活的,全是 `dobot_moveit` 包底下这几个手写节点。

## 四、标准 ros2_control 流程 vs Dobot 真机这套自定义流程,完整对照

| | 标准流程(Gazebo 走的这条) | Dobot 真机自定义流程 |
|---|---|---|
| 谁接收 `follow_joint_trajectory` | 真正的 `joint_trajectory_controller`(ros2_control官方C++实现) | 手写的 `action_move_server`(Python) |
| controller 和硬件的数据交换方式 | 进程内直接函数调用(`write()`),无 ROS2 消息开销,适配高频实时循环 | 又发起一次额外的 ROS2 服务调用(`ServoJ`),多一层通信开销 |
| 轨迹时间轴处理 | 按 `update_rate`(100Hz)逐周期做样条插值,充分利用每个路点的 velocity 信息重建平滑曲线 | 忽略路点自带的时间戳和速度,固定 `sleep(0.18)` 依次发送 |
| 执行中反馈 | 支持标准 action feedback,可实时查询执行进度/误差 | 从未调用 `publish_feedback`,只在**全部**执行完后一次性返回结果 |
| 执行确认 | 闭环,可基于实际状态做修正 | 开环,`call_async` 后不等待确认,发了就不管 |
| 关节状态发布 | `joint_state_broadcaster` 自动从硬件接口读取并发布标准 `/joint_states` | 手写 `feedback.py`(轮询TCP) + `joint_states.py`(转发)两个节点拼出来 |

**规划阶段为什么路点本来就是稀疏的**:OMPL 采样规划器的原始输出,是"能连成一条无碰撞路径的关键路点",不是逐瞬间的连续曲线;每个路点除了位置还带着速度信息,设计上是给执行阶段的 controller 用样条插值"撑开"成平滑曲线的。`action_move_server` 跳过了这一步插值,直接把稀疏点甩出去,是这条自定义链路精度打折扣的根本原因之一。

## 五、这套简化实现,是不是"失控"——不是,但确实有真实代价

机械臂控制柜自己内部还有一层完全独立、这份开源 SDK 完全碰不到的**高频实时伺服系统**(工业/协作机械臂普遍在1kHz以上),`ServoJ` 这类接口本来就是设计给外部低频("几Hz量级")喂目标点、由机械臂自己内部做精细平滑插值用的——所以物理上机械臂的运动大概率还是平滑安全的,不存在"电机真的按稀疏点生硬跳变"的情况。

真正打折扣的是:**实际走出来的运动节奏,不一定跟 MoveIt 计算出来的"理论最优时间/速度曲线"精确吻合**,加上开环、无执行中反馈这些弱点。UR(`Universal_Robots_ROS2_Driver`)、Franka(`franka_ros2`)等厂商官方提供的是**规范的 C++ `hardware_interface` 插件**,在真正的实时循环里做插值,这方面比 Dobot 这份 SDK 的简化实现更严谨——具体差异建议之后有空可以去对应仓库核实细节,再补充进文档。

## 六、对 WARA 项目提案的启发:一个可行的定量评估方向

结合这次挖到的细节,有一个具体、可执行、能"定量展示结果"的分析方向:

**规划路径 vs 实际执行路径的对比(轨迹跟踪误差分析)**

- **规划数据**:MoveIt 规划完成后、执行前,轨迹本身(离散路点+时间戳)就是现成可记录的数据
- **实际数据**:执行过程中订阅 `/joint_states`(100Hz),记录真实关节角度随时间变化
- **两种具体做法**:
  - 方式A(逐关节):规划角度 vs 实际角度,直接比较,算 RMSE,不需要正向运动学,最简单
  - 方式B(末端笛卡尔空间):两边都做正向运动学转换成末端xyz位置,比较空间偏移量,更贴近抓取任务的实际意义,但需要额外的FK计算
- **必须沿路径多点采样,不能只看终点**——终点误差通常很小(因为终点本来就是`ServoJ`发送的最后目标),真正有价值的信息在于中间过程的偏差,这正好能定量验证第四节表格里列出的那些真实存在的执行弱点(固定间隔、开环、丢失速度信息)

这个分析不需要额外开发一个真正的 C++ 硬件接口插件,基于现有的、已经跑通的链路就能做,产出的是可以直接放进论文/答辩的图表和数值指标。

## 七、下一步

- 继续探索 `dobot_demo`/`servo_action` 包里的其他文件,了解厂商自己提供的其他控制模式示例
- 开始写第一版 `MoveGroupInterface` 编程示例(替代手动 RViz 拖拽)
- 视时间安排,实现"规划 vs 实际"轨迹对比分析的第一版原型(先做方式A,逐关节RMSE)
- 有空的话,对照 UR/Franka 官方 ros2_control 驱动源码,补充"规范实现 vs 简化实现"的具体差异
