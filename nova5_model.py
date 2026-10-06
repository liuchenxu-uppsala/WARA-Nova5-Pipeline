#!/usr/bin/env python3
"""Nova 5 的正运动学(FK)模型,直接由 URDF 关节链连乘得到。

两份 URDF 的几何数据都来自 DOBOT_6Axis_ROS2_V3 SDK,逐项抄自文件:

  "moveit": dobot_rviz/urdf/nova5_robot.urdf
            (nova5_moveit 的 nova5_robot.urdf.xacro 引用的就是它,即 MoveIt / RViz 用的模型)
  "gazebo": cra_description/urdf/nova5_robot.xacro
            (dobot_gazebo 的两个 launch 文件加载的就是它,即 Gazebo 里物理仿真用的模型)

两份文件的关节平移和旋转(origin xyz/rpy)完全一致,但 joint1 和 joint4 的转轴方向相反
(moveit: +z, gazebo: -z)。这正是需要拿两个模型分别对比的原因。

约定:
  - 单位 m / rad
  - FK 返回 base_link 到 Link6 的 4x4 齐次变换(SRDF 里规划组的 tip_link 是 Link6,没有额外的工具偏移)
  - 每个关节的变换 = Trans(xyz) · Rot_rpy(rpy) · Rot_axis(q)
    其中 URDF 的 rpy 是固定轴 roll-pitch-yaw,矩阵为 Rz(yaw)·Ry(pitch)·Rx(roll)
"""
import numpy as np

# (xyz, rpy, axis) —— 与 URDF 文件逐字一致
_COMMON_ORIGINS = [
    ((0.0, 0.0, 0.240000000000178),                      (0.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0),                                    (-1.57080287682252, 1.53586622836832, 3.14159265358979)),
    ((-0.399756009268664, -0.0139690033141953, 0.0),     (0.0, 0.0, 0.0)),
    ((-0.329798707647103, -0.0115244277211674, 0.134999532858734), (0.0, 0.0, -1.53586622840712)),
    ((0.0, -0.12, 0.0),                                  (1.5708, 0.0, 0.0)),
    ((0.0, 0.088328, 0.0),                               (-1.5708, 0.0, 0.0)),
]

_AXES = {
    "moveit": [(0.0, 0.0, 0.999999999999855), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1)],
    "gazebo": [(0.0, 0.0, -0.999999999999855), (0, 0, 1), (0, 0, 1), (0, 0, -1), (0, 0, 1), (0, 0, 1)],
}

# 关节范围(两份 URDF 一致)
JOINT_LIMITS = np.array([
    [-6.28, 6.28],
    [-3.14, 3.14],
    [-2.79, 2.79],
    [-6.28, 6.28],
    [-6.28, 6.28],
    [-6.28, 6.28],
])

HOME = np.array([0.0, 0.5378, -1.1869, 0.0, 1.7785, 0.0])   # SRDF 里的 home 姿态


def _rot_rpy(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _rot_axis(axis, angle):
    """Rodrigues 公式,绕任意轴(先归一化,与 URDF/KDL 的处理一致)转 angle。"""
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


def _T(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def joint_transforms(q, model="moveit"):
    """返回 [T_base_Link1, T_base_Link2, ..., T_base_Link6](每个都是 4x4,相对 base_link)。"""
    if model not in _AXES:
        raise ValueError(f"model 必须是 {list(_AXES)}")
    q = np.asarray(q, dtype=float)
    T = np.eye(4)
    out = []
    for i, ((xyz, rpy), axis) in enumerate(zip(_COMMON_ORIGINS, _AXES[model])):
        T = T @ _T(_rot_rpy(rpy), xyz) @ _T(_rot_axis(axis, q[i]), (0, 0, 0))
        out.append(T.copy())
    return out


def fk(q, model="moveit"):
    """base_link → Link6 的 4x4 齐次变换。"""
    return joint_transforms(q, model)[-1]


def rotation_to_quat(R):
    """旋转矩阵 → 四元数 (x, y, z, w),w ≥ 0。"""
    from scipy.spatial.transform import Rotation
    x, y, z, w = Rotation.from_matrix(R).as_quat()
    q = np.array([x, y, z, w])
    return q if q[3] >= 0 else -q


def pose_error(T_a, T_b):
    """返回 (位置误差 mm, 姿态误差 deg)。姿态误差 = 两个旋转之间的夹角。"""
    dp = np.linalg.norm(T_a[:3, 3] - T_b[:3, 3]) * 1000.0
    R = T_a[:3, :3].T @ T_b[:3, :3]
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return dp, np.degrees(np.arccos(c))


def test_configs(n_random=20, seed=0):
    """FK 对比用的一批关节角:零位、home、第4篇用的测试姿态、各关节单独 ±0.5、随机若干。"""
    cfgs = [("zeros", np.zeros(6)),
            ("home", HOME.copy()),
            ("test_pose_ch4", np.array([0.3, 0.2, -0.8, 0.1, 1.0, -0.2]))]
    for j in range(6):
        for s in (+0.5, -0.5):
            q = np.zeros(6)
            q[j] = s
            cfgs.append((f"joint{j+1}={s:+.1f}", q))
    rng = np.random.default_rng(seed)
    lo, hi = JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1]
    # 随机角限制在 ±π 内,避免只因 ±6.28 的大范围产生等价但难读的姿态
    lo, hi = np.maximum(lo, -np.pi), np.minimum(hi, np.pi)
    for k in range(n_random):
        cfgs.append((f"rand{k:02d}", rng.uniform(lo, hi)))
    return cfgs


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    for name, q in test_configs(0)[:3]:
        print(f"--- {name}  q={q}")
        for m in ("moveit", "gazebo"):
            T = fk(q, m)
            print(f"  {m:7s} pos(m)={T[:3,3]}  quat(xyzw)={rotation_to_quat(T[:3,:3])}")
