# 第3篇:手写 MoveGroup Action Client 实战 + demo 环境踩坑排查全记录

延续前两篇的环境和认知,这一篇记录的是跳出 RViz 鼠标拖拽、**自己写代码调用 MoveIt** 的第一次实战——包括为什么不能直接用 `MoveGroupInterface`、手写代码背后每一层消息结构的含义,以及围绕"明明代码逻辑是对的,为什么跑不通"展开的一次完整环境排查(最终定位到:容器镜像和当前 `Dockerfile` 已经不同步)。

## 一、目标

之前一直是通过 RViz 的 MotionPlanning 面板拖拽关节、点击 "Plan and Execute" 来验证 MoveIt。这次想做的是:**脱离 RViz 的图形界面,自己写一段 Python 代码,用代码的方式给 MoveIt 下达规划/执行请求**——这是未来真正做 WARA 项目"位姿输入 → 规划 → 执行"pipeline 绕不开的基本功。

## 二、第一个坑:`MoveGroupInterface` 在 ROS2 Humble 这套环境里不存在

`MoveGroupInterface` 是 ROS1 时代的说法(配套 Python 封装叫 `moveit_commander`)。ROS2 对应的官方 Python 高层封装叫 `moveit_py`(核心类是 `MoveItPy` + `PlanningComponent`),但这个包**不是** `ros-humble-moveit` 这个大包必然自带的,需要单独确认。

实测验证:

```bash
python3 -c "from moveit.planning import MoveItPy; print('可用')"
# ModuleNotFoundError: No module named 'moveit'

apt list --installed 2>/dev/null | grep moveit
# 只有 moveit-core / moveit-ros-* 等底层库,没有 ros-humble-moveit-py
```

**结论:这套环境没有高层 Python 封装可用,只能走"底层"路线——自己写一个 ROS2 action client,直接对接 `move_group` 暴露的标准 action 接口。** 好处是:能借此机会真正搞清楚 `MoveGroupInterface` 这类封装底下到底藏着什么,而不是把它当黑盒用。

## 三、先核实两个必须精确匹配的名字

写代码前,去 SRDF 里核实(不能靠猜):

```xml
<!-- nova5_moveit/config/nova5_robot.srdf -->
<group name="nova5_group">
    <chain base_link="base_link" tip_link="Link6"/>
</group>
<group_state name="home" group="nova5_group">
    <joint name="joint1" value="0"/>
    <joint name="joint2" value="0.5378"/>
    <joint name="joint3" value="-1.1869"/>
    <joint name="joint4" value="0"/>
    <joint name="joint5" value="1.7785"/>
    <joint name="joint6" value="0"/>
</group_state>
```

- 规划组名字:**`nova5_group`**(不是 `nova5_group_controller`——这两个名字长得像,但分别活在"规划阶段"的 SRDF 和"执行阶段"的 `moveit_controllers.yaml` 里,只是 Setup Assistant 生成配置时的命名习惯让它们前缀相同,不是技术上的强制关联)
- 6 个关节名:`joint1` ~ `joint6`
- 现成的 `home` 姿态,刚好拿来当第一次测试目标,方便肉眼核对结果

## 四、最终代码(可运行版本)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    JointConstraint,
    WorkspaceParameters,
)


class MyMoveGroupClient(Node):
    def __init__(self):
        super().__init__('my_move_group_client')
        self._client = ActionClient(self, MoveGroup, 'move_action')

    def send_joint_goal(self, joint_names, joint_values, plan_only=True):
        self.get_logger().info('等待 move_group 的 /move_action 上线...')
        self._client.wait_for_server()

        goal_msg = MoveGroup.Goal()

        # ---- 1. MotionPlanRequest:我想要什么 ----
        req = MotionPlanRequest()
        req.group_name = 'nova5_group'
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3

        ws = WorkspaceParameters()
        ws.header.frame_id = 'base_link'
        ws.min_corner.x, ws.min_corner.y, ws.min_corner.z = -1.0, -1.0, -1.0
        ws.max_corner.x, ws.max_corner.y, ws.max_corner.z = 1.0, 1.0, 1.0
        req.workspace_parameters = ws

        req.start_state.is_diff = True   # 起点 = 机械臂当前真实状态

        goal_constraints = Constraints()
        for name, value in zip(joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)
        req.goal_constraints.append(goal_constraints)

        goal_msg.request = req

        # ---- 2. PlanningOptions:算完之后怎么办 ----
        opts = PlanningOptions()
        opts.plan_only = plan_only
        opts.planning_scene_diff.is_diff = True
        opts.planning_scene_diff.robot_state.is_diff = True
        goal_msg.planning_options = opts

        self.get_logger().info(f'发送目标: {dict(zip(joint_names, joint_values))}')
        send_goal_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('目标被 move_group 拒绝了')
            return

        self.get_logger().info('目标已接受,等待结果...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        self.get_logger().info(f'error_code.val = {result.error_code.val}  (1 表示 SUCCESS)')


def main():
    rclpy.init()
    node = MyMoveGroupClient()

    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    home_values = [0.0, 0.5378, -1.1869, 0.0, 1.7785, 0.0]

    node.send_joint_goal(joint_names, home_values, plan_only=False)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 五、代码背后的核心概念

### 1. ROS2 action 机制:只认"名字+类型",不认进程

`ActionClient(self, MoveGroup, 'move_action')` 这一行,`MoveGroup` 是**类型**(Goal/Result/Feedback 三段消息格式的定义,在 `moveit_msgs/action/MoveGroup.action` 里),`'move_action'` 是**名字**。`ActionClient` 在整个 ROS2 网络范围内扫描"有没有东西同时满足这个名字+类型",**完全不关心对方具体是哪个进程**——这跟之前精读真机执行链路时得出的"ROS2 发现机制只认名字+类型"是同一条原则。

`wait_for_server()` 是客户端在阻塞等待"确认已经扫描到匹配的 server"——不是在和对方"协商格式"(格式双方在写代码时就已经各自硬编码死了),纯粹是等待发现完成。

### 2. `MotionPlanRequest` 关键字段

| 字段 | 作用 |
|---|---|
| `group_name` | 对应 SRDF 里的 `<group name="...">`,不是 controller 名字 |
| `goal_constraints` | 外层列表允许描述"多个互相等价的目标方案";内层 `joint_constraints` 才是每个关节的具体目标 |
| `start_state.is_diff = True` | 起点取"机械臂当前真实状态",而不是把(我们没填的)空 `start_state` 字面理解成"零状态" |
| `workspace_parameters` | 规划搜索的空间盒子,对纯关节空间目标影响不大,但是请求消息里的标准字段 |

### 3. `PlanningOptions` 里反复出现的 "is_diff" 模式

```python
opts.planning_scene_diff.is_diff = True
opts.planning_scene_diff.robot_state.is_diff = True
```

这两行和上面的 `start_state.is_diff` 是**同一种语义在消息结构里不同位置的重复出现**:我们的消息里这部分是空的,设 `is_diff=True` 就是告诉 `move_group`"别把这份空消息当字面意思(世界里没有任何障碍物/机器人状态是零),继续用你自己已经掌握的真实信息"。如果不小心设成 `False`,规划时可能会把已知的真实障碍物"当作不存在",是实打实的安全隐患,不是摆设代码。

`RobotState` 这个结构体,除了 `joint_state`(六个关节角度),还有一个 `attached_collision_objects` 字段——专门描述"有没有东西被刚性固定在机械臂上(比如夹爪抓住的物体),算碰撞时要不要当成机械臂的一部分"。这次练习没有夹爪,用不上,但这是以后真正做抓取任务时必须回来填的字段。

### 4. `move_group` 怎么决定把规划好的轨迹交给哪个 controller

`move_group` 规划完,会查 `moveit_controllers.yaml` 里 `controller_names` 列表,挑"`joints` 字段能完整覆盖这条轨迹所需关节"的那个 controller(Nova5 这边因为只有一个 `nova5_group_controller`,不存在选择歧义)。确定之后,`move_group` 内部会**另外创建一个**类型是 `FollowJointTrajectory`、名字是 `nova5_group_controller/follow_joint_trajectory` 的 `ActionClient`,把轨迹发出去——这整个过程完全不需要我们的代码参与。

## 六、新发现:demo 模式,是第三条真实会用到 `mock_components/GenericSystem` 的链路

第2篇里确认过:**真机**链路完全不启动 `controller_manager`,所以 `mock_components/GenericSystem` 这份配置在真机场景下"活在文件里、从未被加载"。这次通过 `moveit_demo.launch.py` 实测,补充验证了第三种场景——`demo` 模式:

```python
# nova5_moveit/launch/demo.launch.py(官方 moveit_configs_utils 标准调用,非 Dobot 定制)
from moveit_configs_utils.launches import generate_demo_launch
moveit_config = MoveItConfigsBuilder("nova5_robot", package_name="nova5_moveit").to_moveit_configs()
return generate_demo_launch(moveit_config)
```

`generate_demo_launch()` **真的会启动** `controller_manager`,并按 `nova5_robot.ros2_control.xacro` 里声明的插件加载硬件接口——也就是说,**`mock_components/GenericSystem` 终于在这里派上了用场**。三条链路现在完整对比如下:

| 模式 | controller_manager | 硬件接口插件 | 寄生在哪个进程 |
|---|---|---|---|
| **demo** | 启动(独立 `ros2_control_node` 进程) | `mock_components/GenericSystem`(纯数学插值,无物理) | 自己独立一个进程 |
| **Gazebo** | 启动 | `gazebo_ros2_control/GazeboSystem`(真实物理引擎) | 寄生在 `gzserver` 进程内部(靠 `cra_description` xacro 里 `<plugin filename="libgazebo_ros2_control.so">` 这条声明触发) |
| **真机** | **不启动** | 不适用 | 靠 `dobot_moveit/action_move_server.py` 手写代码,在同一个地址上冒充标准 controller |

三种模式下,`move_group` 发送轨迹的目标地址(`nova5_group_controller/follow_joint_trajectory`)**字面上完全一样**,`move_group` 本身不知道、也不关心背后换的是谁——这正是整套 `ros2_control` 插件架构设计的核心价值:控制逻辑和"硬件到底是什么"彻底解耦。

## 七、如何在 RViz 上跑通 Plan(demo 模式,已验证成功)

```bash
# 确认环境变量(正常应该已经写进 /etc/bash.bashrc,新终端会自动带上)
echo $DOBOT_TYPE   # 应输出 nova5

# 终端 1:启动 demo 模式(RViz + move_group + controller_manager[mock硬件])
ros2 launch dobot_moveit moveit_demo.launch.py

# 终端 2(确认用的是全新终端):确认 action 地址存在
ros2 action list                                        # 应看到 /move_action
ros2 interface show moveit_msgs/action/MoveGroup         # 核对消息结构

# 终端 2:跑我们自己写的脚本
python3 /dobot_ws/my_move_group_client.py
```

终端输出最后一行 `error_code.val = 1`(`SUCCESS`),同时 RViz 里机械臂应该平滑转动到目标姿态——这是本次练习最终跑通确认成功的结果。

## 八、调试记录:从 `CONTROL_FAILED(-4)` 到成功,完整排查链路

第一次执行(`plan_only=False`)报错 `error_code.val = -4`。按"先查含义、再查证据、不猜"的方法,排查过程记录如下,这套方法论本身比具体某一次的 bug 更值得留存:

**1. 先搞清楚错误码含义**:`-4` 对应 `moveit_msgs/MoveItErrorCodes` 里的 `CONTROL_FAILED`——说明**规划阶段是成功的**,问题出在更后面的**执行/控制阶段**(区别于 `-1` `PLANNING_FAILED`)。

**2. 去 `move_group` 的真实日志里找证据**,而不是对着一个分类错误码瞎猜:

```
Didn't receive robot state (joint angles) with recent timestamp...
latest received state has time 0.000000.
Failed to validate trajectory: couldn't receive full current joint state within 1s
Solution found but controller failed during execution
```

`latest received state has time 0.000000` 是关键线索——说明 `move_group` **从启动到现在,一条 `/joint_states` 都没收到过**,不是"稍微延迟"。

**3. 用基础 ROS2 命令逐层验证,缩小范围**(`ros2 control` 这个子命令在这个环境里是坏的,全程改用不依赖它的等价命令):

```bash
ros2 topic list | grep joint_states          # 话题存在(说明有订阅者),但...
ros2 topic hz /joint_states                  # ...卡住不动,说明没有发布者
ros2 service list | grep controller_manager  # controller_manager 进程本身是活的
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers
# → controller=[]   实锤:一个 controller 都没被加载
```

**4. 排查"为什么没加载",发现根因是容器环境和 `Dockerfile` 不同步**:

```bash
ros2 pkg list | grep joint_state_broadcaster      # 查不到
dpkg -l | grep ros-humble-joint                   # joint-state-broadcaster 显示 ii(已装),但...
# ...在当前终端 ros2 pkg list 还是看不到 → 旧终端没刷新,换新终端后恢复正常

ros2 pkg list | grep joint_trajectory_controller  # 查不到
dpkg -l | grep ros-humble-joint                   # 这个包连 dpkg 层面都没有 → 真的没装,尽管 Dockerfile 里写了
```

补装缺失的包之后,又遇到一层"**进程级别**的没刷新"——`controller_manager` 进程是在包装好**之前**就已经启动的,它内部的插件列表是启动那一刻就固定的,不会动态感知新装的包:

```
Loader for controller 'nova5_group_controller' not found.
Available classes: controller_manager/test_controller ...(只有自带的测试类)
```

解决办法:完整重启 `moveit_demo.launch.py`(让 `controller_manager` 用全新进程启动,这时两个包都已经在磁盘上了),重启后**自动加载成功**,不需要再手动 `spawner`。

**排查方法论小结**(以后遇到"配置看着没问题但就是跑不起来"可以直接套用):

1. 错误码只告诉你"分类",真正原因要去对应进程自己的日志里找
2. 怀疑缺包/缺组件时,`dpkg` 和 `ros2 pkg list` 两条渠道交叉验证,别只信一边
3. 终端环境和进程内部状态都可能"过时"——装了新东西之后,**终端要开新的,相关进程要重启**,这是两件独立的事,缺一不可

## 九、已知未解决问题:Gazebo 模式控制器加载失败

按同样的方法论排查 Gazebo 模式(`dobot_gazebo gazebo_moveit.launch.py` + `dobot_moveit moveit_gazebo.launch.py`)时,撞上了同样的 `Loader for controller 'nova5_group_controller' not found`,但这次**哪怕用全新终端、完整重启流程,问题依然存在**——说明这次不是"终端/进程没刷新"这个已知模式,而是更深层的东西。

已排除的可能:
- 不是终端没刷新(验证过全新终端)
- 不是包没装(`joint_trajectory_controller` 已确认在磁盘上)
- `reload_controller_libraries` service 尝试过,无效

怀疑方向:Gazebo 模式下,`controller_manager` 是寄生在 `gzserver` 进程内部的(通过 `libgazebo_ros2_control.so` 这个 Gazebo 插件触发创建),这条路径下动态库/插件的查找机制,可能跟标准 `ros2_control_node` 走的不是完全一样的环境传递路径。同时排查过程里还额外发现 `ros2controlcli`(提供 `ros2 control` 这个子命令的包)**也不在这个容器里**,尽管 `Dockerfile` 明确写了要装——这是本次排查过程中第三次撞见"`Dockerfile` 写的和容器实际状态对不上"的情况。

**结论:这次大概率不是单个包缺失能解释的,根因怀疑是当前这个容器(`dobot_dev`)是用一个较旧版本的 `Dockerfile` 构建出来的,后续 `Dockerfile` 虽然更新过,但容器从未重新 `build`**(`start_dobot.sh` 的逻辑是"容器存在就直接复用",不会自动检测/重建)。

**计划**:暂不继续在这个问题上排查,等找时间用当前这份 `Dockerfile` 彻底重新 `build` 一个新镜像后,再回来验证 Gazebo 模式是不是因此一并解决。

## 十、下一步

- 找时间重新 `build` Docker 镜像,确保容器状态和 `Dockerfile` 完全同步,预期能一次性解决 Gazebo 模式的问题
- 镜像重建后,回来验证 Gazebo 模式下的真实物理仿真表现(关节有没有超调/震荡等 demo 模式看不出来的问题)
- 继续用不同于 `home` 的自定义关节角度测试,排除"只对预设值有效"的可能性
- 开始实现"规划 vs 实际"轨迹跟踪误差分析的第一版原型(方式A:逐关节 RMSE)
