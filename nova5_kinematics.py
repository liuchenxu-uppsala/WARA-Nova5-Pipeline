#!/usr/bin/env python3
"""Nova 5 的 Jacobian、可操作度、数值 IK(在 nova5_model.py 的 FK 之上)。不依赖 ROS。

约定(与 nova5_model.py 一致):
  - 默认用 "moveit" 模型(joint1/joint4 转轴 +z,也是现在 Gazebo 修复后用的模型)
  - 末端 = Link6 坐标系原点,位姿用 4x4 齐次矩阵表示,在 base_link 下
  - 几何 Jacobian J(q) 是 6x6,前 3 行线速度、后 3 行角速度,都表示在 base_link 坐标系下:
        [v; w] = J(q) · qdot
    第 i 列(旋转关节):  [ z_i × (p_e − p_i) ;  z_i ]
    z_i = 第 i 个关节转轴在 base 下的方向,p_i = 该关节转轴上一点(关节原点)在 base 下的位置,
    p_e = 末端位置。这就是 M1_review.md 里 "第 i 列 = 只转第 i 个关节时末端的速度" 的公式。
"""
import numpy as np
from scipy.spatial.transform import Rotation

import nova5_model as m
from nova5_model import _COMMON_ORIGINS, _AXES, _T, _rot_rpy, _rot_axis, JOINT_LIMITS


# ------------------------------------------------------------------ Jacobian
def jacobian(q, model="moveit"):
    """解析(几何)Jacobian,6x6,表示在 base_link 下。"""
    q = np.asarray(q, dtype=float)
    T = np.eye(4)
    zs, ps = [], []
    for i, ((xyz, rpy), axis) in enumerate(zip(_COMMON_ORIGINS, _AXES[model])):
        T_pre = T @ _T(_rot_rpy(rpy), xyz)            # 关节 i 的原点坐标系(转动之前)
        a = np.asarray(axis, float)
        a = a / np.linalg.norm(a)
        zs.append(T_pre[:3, :3] @ a)                   # 转轴方向(base 下)
        ps.append(T_pre[:3, 3].copy())                 # 转轴上的点(base 下)
        T = T_pre @ _T(_rot_axis(axis, q[i]), (0, 0, 0))
    p_e = T[:3, 3]
    J = np.zeros((6, 6))
    for i in range(6):
        J[:3, i] = np.cross(zs[i], p_e - ps[i])
        J[3:, i] = zs[i]
    return J


def jacobian_numeric(q, model="moveit", eps=1e-6):
    """用有限差分对 FK 求导得到的 Jacobian,仅用来验证解析 Jacobian 有没有算对。
    线速度部分:位置的差分;角速度部分:R(q+δ)·R(q)^T 的旋转向量 / δ(表示在 base 下)。"""
    q = np.asarray(q, dtype=float)
    T0 = m.fk(q, model)
    J = np.zeros((6, 6))
    for i in range(6):
        dq = q.copy()
        dq[i] += eps
        T1 = m.fk(dq, model)
        J[:3, i] = (T1[:3, 3] - T0[:3, 3]) / eps
        dR = T1[:3, :3] @ T0[:3, :3].T
        J[3:, i] = Rotation.from_matrix(dR).as_rotvec() / eps
    return J


def manipulability(q, model="moveit"):
    """可操作度 w = sqrt(det(J J^T)),6x6 时等于 |det J|。越接近 0 越接近奇异。
    注意:J 同时含 m/rad 与 rad/rad 两种量纲,数值本身没有直观单位,看"是否趋近 0"和相对变化。"""
    J = jacobian(q, model)
    return float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0)))


def singular_values(q, model="moveit"):
    return np.linalg.svd(jacobian(q, model), compute_uv=False)


# ------------------------------------------------------------------ IK
def pose_error_vec(T_target, T_cur):
    """6 维误差:前 3 位置差(m),后 3 姿态差(旋转向量,rad),都在 base 下。"""
    e = np.zeros(6)
    e[:3] = T_target[:3, 3] - T_cur[:3, 3]
    e[3:] = Rotation.from_matrix(T_target[:3, :3] @ T_cur[:3, :3].T).as_rotvec()
    return e


def ik_numeric(T_target, q0, model="moveit", max_iter=200, tol_pos=1e-6, tol_rot=1e-6,
               damping=0.05, step_limit=0.5):
    """阻尼最小二乘(Levenberg–Marquardt)数值 IK,即 M1_review.md 里"方法二"的带阻尼版本:
        Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e
    阻尼 λ 让 J 接近奇异时步长不会爆炸(普通 J⁻¹ 会)。每步限幅,并把关节夹在限位内。
    返回 (q, 是否收敛, 迭代次数, 最终误差向量)。"""
    q = np.clip(np.asarray(q0, float), JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
    lam2 = damping ** 2
    for it in range(1, max_iter + 1):
        T = m.fk(q, model)
        e = pose_error_vec(T_target, T)
        if np.linalg.norm(e[:3]) < tol_pos and np.linalg.norm(e[3:]) < tol_rot:
            return q, True, it, e
        J = jacobian(q, model)
        dq = J.T @ np.linalg.solve(J @ J.T + lam2 * np.eye(6), e)
        n = np.max(np.abs(dq))
        if n > step_limit:
            dq *= step_limit / n
        q = np.clip(q + dq, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
    T = m.fk(q, model)
    return q, False, max_iter, pose_error_vec(T_target, T)


def wrap_to_pi(q):
    return (np.asarray(q) + np.pi) % (2 * np.pi) - np.pi


def ik_all(T_target, model="moveit", n_starts=60, seed=0, **kw):
    """多起点求 IK,返回去重后的所有解(6R 机械臂通常有多组解)。"""
    rng = np.random.default_rng(seed)
    lo = np.maximum(JOINT_LIMITS[:, 0], -np.pi)
    hi = np.minimum(JOINT_LIMITS[:, 1], np.pi)
    sols = []
    for _ in range(n_starts):
        q, ok, _, _ = ik_numeric(T_target, rng.uniform(lo, hi), model, **kw)
        if not ok:
            continue
        qw = wrap_to_pi(q)
        if not any(np.max(np.abs(wrap_to_pi(qw - s))) < 1e-3 for s in sols):
            sols.append(qw)
    return sols
