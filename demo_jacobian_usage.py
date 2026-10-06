#!/usr/bin/env python3
"""Jacobian 怎么用:三个小演示(纯 Python,不需要 ROS)
  A. 看 home 姿态下的 6x6 J
  B. 正向用法:给关节速度 q̇  →  J q̇ 得到末端速度 v,并用 FK 差分验证
  C. 反向用法:想让末端沿 +x 匀速走 10 cm,姿态不变 → 每一小步解 q̇ = J^-1 v,积分后看实际走到哪
  D. 同样的任务,但起点离腕奇异很近(q5≈0)→ 关节速度爆炸
用法: python3 demo_jacobian_usage.py
"""
import numpy as np
from nova5_model import fk, HOME
from nova5_kinematics import jacobian, manipulability, pose_error_vec

np.set_printoptions(precision=4, suppress=True, linewidth=120)
q0 = np.array(HOME, dtype=float)

# ---------- A ----------
print("=== A. home 姿态下的 Jacobian (行: vx vy vz wx wy wz;列: 关节1..6) ===")
J = jacobian(q0)
print(J)
print("上 3 行:线速度 (m/s 每 rad/s);下 3 行:角速度 (rad/s 每 rad/s)")
print(f"可操作度 w = {manipulability(q0):.4f}")

# ---------- B ----------
print("\n=== B. 正向:已知关节速度,求末端速度 ===")
qd = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])        # 只转关节1,0.1 rad/s
v = J @ qd
print("q̇ =", qd)
print("J q̇ =", v, "  (前3: 线速度 m/s,后3: 角速度 rad/s)")
dt = 1e-5
T0, T1 = fk(q0), fk(q0 + qd * dt)
e = pose_error_vec(T1, T0)       # 注意 pose_error_vec(目标, 当前) = 目标 - 当前
v_fd = np.concatenate([e[:3], e[3:]]) / dt
print("FK 差分 =", v_fd, "  (用 FK 真的走一小步再除以时间)")

# ---------- C / D ----------
def move_straight(q_start, v_des=None, T_total=2.0, dt=0.01):
    """按期望末端速度 v_des(6 维,默认沿 base +x 0.05 m/s、姿态不变)走 T_total 秒。每步 q̇ = J^-1 v;返回统计。"""
    q = q_start.copy()
    T_start = fk(q)
    if v_des is None:
        v_des = np.array([0.05, 0, 0, 0, 0, 0])
    n = int(round(T_total / dt))
    max_qd, min_w = 0.0, 1e9
    for _ in range(n):
        Jc = jacobian(q)
        qd = np.linalg.solve(Jc, v_des)
        max_qd = max(max_qd, np.abs(qd).max())
        min_w = min(min_w, manipulability(q))
        q = q + qd * dt
    T_end = fk(q)
    dp = (T_end[:3, 3] - T_start[:3, 3]) * 1000
    ang = np.degrees(np.linalg.norm(pose_error_vec(T_end, T_start)[3:]))
    return dp, ang, max_qd, min_w, q

print("\n=== C. 反向:让末端沿 +x 匀速走 10 cm(0.05 m/s,共 2 s),姿态不变 ===")
dp, ang, mq, mw, qf = move_straight(q0)
print(f"末端实际位移 (mm): x={dp[0]:.2f}  y={dp[1]:.2f}  z={dp[2]:.2f}   (目标 x=100, y=0, z=0)")
print(f"姿态偏离: {ang:.3f}°   最大关节速度: {mq:.3f} rad/s   最小可操作度: {mw:.4f}")
print("终点关节角:", qf)
print("说明:一步一步地算 q̇=J^-1 v 再积分,就是把末端的直线运动'翻译'成关节运动;小误差来自一阶近似。")

print("\n=== D. 靠近腕奇异(q5=0.02):要求末端往 J 最'弱'的那个方向运动 ===")
q_sing = q0.copy(); q_sing[4] = 0.02
U, S, Vt = np.linalg.svd(jacobian(q_sing))
v_weak = 0.05 * U[:, -1]          # 最弱方向(最小奇异值对应的末端速度方向)
print("要求的末端速度 (6维):", v_weak)
for name, q_start in [("home(远离奇异)", q0), ("q5=0.02(靠近腕奇异)", q_sing)]:
    Jx = jacobian(q_start)
    qd = np.linalg.solve(Jx, v_weak)
    print(f"{name:20s} 可操作度 w={manipulability(q_start):.5f}  需要的关节速度 max|q̇| = {np.abs(qd).max():8.3f} rad/s")
print("说明:同样的末端速度要求,在奇异附近需要的关节速度大几十倍(越接近奇异越大)(这就是 J^-1 爆炸)。")
print("      而在 C 里,沿 +x 走直线的方向并不是 J 最弱的方向,所以即使接近奇异也没问题——奇异'只丢一个方向'。")
