#!/usr/bin/env python3
"""【不需要 ROS】IK 往返验证:随机关节角 q_true → FK → 目标位姿 → 自己的 IK → FK → 和目标比较。

同一个目标会做两件事:
  1. 单起点:从 home 出发做一次 IK(ik_numeric),看能不能收敛、迭代几次、误差多少
  2. 多起点:从 n_starts 个随机起点各做一次(ik_all),数一数能找到几组不同的解

用法:
  python3 ik_roundtrip_simple.py            # 默认 10 个随机目标
  python3 ik_roundtrip_simple.py 20         # 20 个随机目标
需要同目录有 nova5_model.py 和 nova5_kinematics.py
"""
import sys
import numpy as np

from nova5_model import fk, HOME, JOINT_LIMITS
from nova5_kinematics import ik_numeric, ik_all, pose_error_vec

N_TARGETS = int(sys.argv[1]) if len(sys.argv) > 1 else 10
N_STARTS = 20

rng = np.random.default_rng(1)
lo, hi = JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1]


def err_mm_deg(T_target, q):
    """把关节角 q 代回 FK,和目标位姿比:返回 (位置误差 mm, 姿态误差 度)"""
    e = pose_error_vec(T_target, fk(q))
    return np.linalg.norm(e[:3]) * 1000, np.degrees(np.linalg.norm(e[3:]))


print(f"{'#':>2} | {'单起点(home)':^30} | {'多起点':^24}")
print(f"{'':>2} | {'收敛':^4} {'迭代':>4} {'位置mm':>9} {'姿态°':>9} | {'解的个数':>6} {'最大误差mm':>12}")
print("-" * 66)

n_single_ok = n_multi_ok = 0
for k in range(N_TARGETS):
    q_true = rng.uniform(lo, hi)          # ① 随机一组关节角
    T_target = fk(q_true)                 # ② FK 得到目标位姿(一定可达)

    # ③ 单起点:从 home 出发
    q, ok, it, _ = ik_numeric(T_target, HOME)
    dp, da = err_mm_deg(T_target, q)
    n_single_ok += ok

    # ④ 多起点:找所有能找到的解
    sols = ik_all(T_target, n_starts=N_STARTS, seed=k)
    n_multi_ok += bool(sols)
    worst = max((err_mm_deg(T_target, s)[0] for s in sols), default=float("nan"))

    print(f"{k:>2} | {'是' if ok else '否':^4} {it:>4} {dp:>9.4f} {da:>9.4f} | {len(sols):>6} {worst:>12.2e}")

print("-" * 66)
print(f"单起点(home)收敛: {n_single_ok}/{N_TARGETS}")
print(f"多起点({N_STARTS} 个)至少找到一组解: {n_multi_ok}/{N_TARGETS}")
print("解的'误差'是把解代回 FK 后与目标的差;收敛判据是位置 < 1e-6 m 且姿态 < 1e-6 rad。")
