# 09 模型对比(自己的模型 vs 仿真器)与 M2 总结:假设与局限

> 对应 M2 的后两项:*compare the candidate's model with the simulator/robot model*;*document the main assumptions and limitations*。
> 本篇回答:自己的 FK 模型、MoveIt 加载的模型、Gazebo 加载的模型是否一致?发现了什么问题、为什么、怎么修、修完怎么验证?整个 M2 的结果建立在哪些假设上,有哪些局限?

## 1. 为什么要做这个对比

系统里同时存在三份"机械臂模型":

| 谁在用 | 模型文件 | 用来干什么 |
|---|---|---|
| 自己的代码 | `nova5_model.py`(从 URDF 抄出的常量) | FK / Jacobian / IK |
| MoveIt(`move_group`) | `dobot_rviz/urdf/nova5_robot.urdf` | 规划、`/compute_fk`、`/compute_ik` |
| Gazebo(`robot_state_publisher` + 物理仿真) | `cra_description/urdf/nova5_robot.xacro` | 仿真执行、发布 TF |

如果三份模型不一致,MoveIt 规划出"末端到 A 点"的关节角,Gazebo 里末端却会到 B 点,而且不会报错。所以必须检查。

## 2. 对比方法(`verify_model_consistency.py --sim`)

需要先启动 Gazebo + MoveIt:

```bash
# 终端 1
export DOBOT_TYPE=nova5
ros2 launch dobot_gazebo gazebo_moveit.launch.py
# 终端 2
ros2 launch dobot_moveit moveit_gazebo.launch.py
# 终端 3
python3 /dobot_ws/shared/verify_model_consistency.py --sim
```

(脚本依赖同目录的 `nova5_model.py` 和 `my_move_group_client.py`。)

关键流程(精简写法,完整代码见脚本 `run_sim()`)。对 4 组关节角(home、home 且 joint1=0.5、home 且 joint4=0.5、测试姿态)逐一:

```python
code = node.send_joint_goal(JOINT_NAMES, list(q_goal))   # ① 让 MoveIt 规划并在 Gazebo 里执行
node.spin_for(2.5)                                        # ② 等机械臂停稳
q = 从 /joint_states 读出的实际关节角                      # ③ Gazebo 上报的关节角
T_tf  = node.tf_pose()          # ④ TF: base_link→Link6(Gazebo 侧 robot_state_publisher 用 Gazebo 的 URDF 连乘)
T_mv  = node.moveit_fk(q)       # ⑤ MoveIt /compute_fk(用 MoveIt 的 URDF)
T_own = fk(q, "moveit")         # ⑥ 自己的 FK
dp, da = pose_error(T_tf, T_mv) # ⑦ 位置差(mm)、姿态差(°)
```

要点:

- **三个值用的是同一组关节角**(③ 读到的实际值),所以差异只能来自模型本身,而不是"关节没转到位"。
- **TF 不是独立的物理测量**:Gazebo 只上报关节角,`robot_state_publisher` 用它加载的 URDF 连乘发布 TF,`tf2` 再沿树连乘得到 `base_link→Link6`。所以 TF 本质上是"Gazebo 那份 URDF 的 FK"。
- 判定阈值:位置 < 1 mm 且姿态 < 0.1° 视为一致。

## 3. 修正前的结果:不一致

```
=== home,           实际关节角 = [-0.0064, 0.5403, -1.19,   -0.0054, 1.777,  -0.0043]
=== home, joint1=0.5 实际关节角 = [ 0.4997, 0.5398, -1.1881,  0.0071, 1.7876, -0.0089]
=== home, joint4=0.5 实际关节角 = [ 0.005,  0.532,  -1.1792,  0.5076, 1.7789, -0.0079]
=== test_pose_ch4    实际关节角 = [ 0.3043, 0.2082, -0.7941,  0.0941, 0.9909, -0.196 ]
```

| 关节角 | TF(Gazebo)x / y / z mm | MoveIt FK x / y / z mm | 位置差 | 姿态差 |
|---|---|---|---|---|
| home | 135.87 / −116.04 / 889.78 | 134.83 / −117.78 / 888.26 | 2.53 mm | 0.96° |
| home, joint1=0.5 | 63.09 / −166.60 / 888.73 | 173.71 / −37.33 / 890.73 | **170.15 mm** | **57.26°** |
| home, joint4=0.5 | 140.19 / −117.46 / 817.41 | 98.96 / −116.25 / 954.78 | **143.43 mm** | **58.17°** |
| test_pose_ch4 | 167.08 / −244.69 / 953.17 | 266.37 / −108.58 / 977.20 | **170.17 mm** | **36.44°** |

同时,自己的 FK 与 MoveIt FK 完全一致;若把自己 FK 里 joint1、joint4 的转轴取反(脚本里的 `FK[gazebo]`),则与 TF 完全一致(0.00 mm)。

**解读**:

- 只转 joint1 或 joint4 时差异最大(约 57°),说明问题出在这两个关节。
- home 也差 2.53 mm:home 的 joint1、joint4 名义上是 0,但实际停在 −0.0064、−0.0054 rad;转轴方向相反时,这点小角度在两边被算成相反方向,误差被放大成毫米级。
- 自己的 FK 只要把 joint1、joint4 的轴取反就能完全复现 TF,说明两份 URDF 的区别**只有这两个轴的方向**。

## 4. 原因:Dobot V3 仓库里两份 URDF 不一致

`cra_description/urdf/nova5_robot.xacro`(Gazebo 用):

```xml
<!-- joint1(第 110 行) -->   <axis xyz="0 0 -0.999999999999855" />
<!-- joint4(第 284 行) -->   <axis xyz="0 0 -1" />
```

`dobot_rviz/urdf/nova5_robot.urdf`(MoveIt 用):

```xml
<!-- joint1(第 107 行) -->   <axis xyz="0 0 0.999999999999855" />
<!-- joint4(第 281 行) -->   <axis xyz="0 0 1" />
```

其余四个关节两边都是 `0 0 1`。同一组关节角,joint1、joint4 在两份模型里转向相反。

Dobot 的 V4 仓库在 2026-01-23 的提交 `f31d2ed`("update nova5-xacro")里修改了 Gazebo 用的 nova5 xacro,改后与 MoveIt 版一致。这说明官方也认为 MoveIt 版的方向是对的。

## 5. 修正

只改 Gazebo 那份 xacro 里的两行,让它和 MoveIt 一致(两处字符串在文件里都是唯一的):

```bash
SRC=/dobot_ws/src/DOBOT_6Axis_ROS2_V3/cra_description/urdf/nova5_robot.xacro
INS=/dobot_ws/install/cra_description/share/cra_description/urdf/nova5_robot.xacro

sed -i 's/xyz="0 0 -0.999999999999855"/xyz="0 0 0.999999999999855"/; s/xyz="0 0 -1"/xyz="0 0 1"/' $SRC
cat $SRC > $INS        # 把修改同步到 install 目录
```

- **为什么要改 install 里的那份**:launch 文件通过 `get_package_share_directory` 读的是 `install/.../share/` 下的文件,不是 `src/` 下的。
- **为什么不用 `colcon build`**:install 下的文件属主是 root,普通用户 `colcon build` 时无法修改文件权限而报错。但我们只需改内容,有写权限就够,所以直接覆盖内容。
- **为什么不需要重新编译**:xacro 是在 launch 时才被展开成 URDF 的,改完重启 Gazebo 就生效。
- MoveIt 那份不用动。

## 6. 修正后的结果:一致

| 关节角 | TF(Gazebo)x / y / z mm | MoveIt FK x / y / z mm | 自己的 FK x / y / z mm | TF vs MoveIt |
|---|---|---|---|---|
| home | 128.40 / −117.44 / 893.04 | 128.40 / −117.44 / 893.04 | 128.40 / −117.44 / 893.04 | 0.00 mm, 0.00° |
| home, joint1=0.5 | 178.19 / −35.95 / 890.82 | 178.19 / −35.95 / 890.82 | 178.19 / −35.95 / 890.82 | 0.00 mm, 0.00° |
| home, joint4=0.5 | 91.65 / −117.05 / 952.96 | 91.65 / −117.05 / 952.96 | 91.65 / −117.05 / 952.96 | 0.00 mm, 0.00° |
| test_pose_ch4 | 266.47 / −110.71 / 976.78 | 266.47 / −110.71 / 976.78 | 266.47 / −110.71 / 976.78 | 0.00 mm, 0.00° |

(各行的坐标与修正前不同,是因为每次运行时 Gazebo 停下的实际关节角略有不同;比较总是在同一次运行的同一组实际关节角下进行。)

修正后,自己的 FK、MoveIt、Gazebo 三者在 4 组关节角下完全一致。

## 7. 这个对比证明了什么、没证明什么

**证明了**:

- 自己的 FK 实现与 MoveIt 一致(第 6 篇也用 `/compute_fk` 单独验证过)。
- 修正后,MoveIt 规划用的模型和 Gazebo 执行用的模型一致,仿真闭环可信。

**没证明**:

- **模型与真机一致**。三者都基于 URDF,URDF 错了会一起错。需要读真机控制器自己算的位姿来对比(见第 8 节)。
- **真机 joint1、joint4 的正方向是哪一种**。V4 的修改支持 MoveIt 版,但没有在实验室的这台 Nova 5 上确认过。

## 8. 待办:真机对比

`verify_model_consistency.py --real` 是**只读**模式,不会让机械臂动:

1. 用示教或拖动把机械臂放到某个姿态(最好包括 joint1、joint4 明显非零的姿态)。
2. 运行脚本,读取 Dobot 控制器的 `GetAngle`(关节角)和 `GetPose`(控制器自己算的末端位姿)。
3. 把关节角代入自己的 FK(两种轴向各算一次),和 `GetPose` 比较。

需要注意:`GetPose` 的位置可能以 mm 为单位,姿态是欧拉角,并且取决于当前选的用户坐标系和工具坐标系;对比时要确认这些约定。

**另外两件事**:

- 上面的修正只改在当前容器里,**重建镜像会丢失**。真机方向确认后,在 `Dockerfile` 的 `colcon build` 之前加入:

  ```dockerfile
  RUN sed -i 's/xyz="0 0 -0.999999999999855"/xyz="0 0 0.999999999999855"/; s/xyz="0 0 -1"/xyz="0 0 1"/' \
      /dobot_ws/src/DOBOT_6Axis_ROS2_V3/cra_description/urdf/nova5_robot.xacro
  ```

- 向导师确认实验室控制器对应 V3 还是 V4 SDK。

## 9. 假设(模型和结果成立的前提)

| 假设 | 说明 |
|---|---|
| 刚体、理想关节 | 连杆不变形;关节无间隙、无弹性 |
| URDF 参数等于真机 | 连杆长度、偏移、角度用 Dobot 提供的数值,未标定 |
| 真机关节零位与正方向与 MoveIt 版 URDF 一致 | joint1、joint4 尚未在真机确认 |
| 末端 = Link6 原点(法兰) | 未装工具,未考虑 TCP 偏移;装夹爪后需再乘固定变换 |
| 只做运动学 | 不考虑动力学、关节速度/加速度限制(M4)、碰撞 |
| 关节限位取自 URDF | joint1/4/5/6 ±6.28,joint2 ±3.14,joint3 ±2.79 rad |
| 基座固定 | base_link 即世界参考,未考虑安装位置和移动底盘 |

## 10. 局限

**验证方面**

- 仿真对比的参照(MoveIt、Gazebo)与自己的模型同源于 URDF,只能证明实现正确、模型彼此一致,不能证明模型准确。
- 样本量小:仿真对比 4 组关节角;`/compute_fk` 对比若干组;`/compute_ik` 对比 1 个目标;IK 往返验证 10 个随机目标。
- 真机对比尚未进行。

**IK 方面**

- 数值 IK 不保证找全所有解;多起点数量有限时可能漏解。
- 阻尼 λ = 0.05、步长上限 0.5 rad、迭代上限 200 次、容差 1e-6 都是经验值,未系统调参。
- 关节限位只是每步"夹住",不是优化约束,迭代可能停在边界附近。
- 对比 `/compute_ik` 时未考虑碰撞。

**奇异性方面**

- 只确认了 q5 = 0 / ±π 的结构(关节 3、4、6 的列线性相关,丢失的方向主要是沿 y 的平移)。
- q3 ≈ 0、q3 ≈ −1.01、q2 扫描中的几处只知道数值上奇异,几何含义未分类。
- 奇异性扫描只在 home 附近、每次改一个关节,未覆盖整个工作空间。

**环境方面**

- Gazebo 轴向修正只在容器内,重建镜像会丢失(见第 8 节)。
- 实验室控制器的 SDK 版本(V3 / V4)未知。
- 曾出现一次执行等待结果卡住、DDS "sequence size exceeds remaining buffer" 报错,未复现,原因未查明。

## 11. M2 完成情况

| M2 要求 | 状态 | 文档 |
|---|---|---|
| Implement and verify FK/IK | ✅ 完成(仿真) | 第 6 篇(FK)、第 8 篇(IK) |
| Demonstrate basic Jacobian and singularity understanding | ✅ 完成;部分奇异位置未分类 | 第 7 篇 |
| Compare the candidate's model with the simulator/robot model | ✅ 仿真器完成(发现并修正 URDF 轴向不一致);⏳ 真机待做 | 本篇第 2–8 节 |
| Document the main assumptions and limitations | ✅ | 本篇第 9–10 节 |
