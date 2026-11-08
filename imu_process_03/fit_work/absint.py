# -*- coding: utf-8 -*-
"""
能不能不用 13 mm 常数，直接靠二次积分测出"从最初静置算起"的绝对深度？
=========================================================================
思路：按压开始前有一段真正的静置（t < onset）。在静置段里把加速度均值去掉
      （这一步同时把加速度计零偏估掉），然后**不做任何高通/去趋势**地积分两次，
      位移就带着直流分量一路积下来。
问题：零偏残差和噪声会被积分成漂移。这个脚本就是量一下漂到多少。
"""
import io, os, sys, glob
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cpr_fit import (load_sensor, load_platform, find_platform, preprocess,
                     find_positive_peaks, G, BASE, OUT, SIMS)
import pipeline as P
import bench as B


def abs_integrate(sim, t_quiet_end=None):
    d0 = os.path.join(BASE, sim)
    imu = sorted(glob.glob(os.path.join(d0, "lsm6dsv_raw_*.txt")))[-1]
    t, acc = load_sensor(imu)
    pt, pv, peaks = load_platform(find_platform(sim))

    # 用 200 Hz 均匀网格
    fs = 200.0
    t_u = np.arange(t[0], t[-1], 1.0 / fs)
    ax = np.interp(t_u, t, acc[:, 0]); ay = np.interp(t_u, t, acc[:, 1])
    az = np.interp(t_u, t, acc[:, 2])
    A = np.column_stack([ax, ay, az])
    amag = np.linalg.norm(A, axis=1)
    quiet = (amag > 900) & (amag < 1100)

    # 运动起点
    h = B.harness(sim)
    tp = h["tp"]
    onset = tp[0] - 0.35                      # 估计按压开始时刻
    q = quiet & (t_u < onset)
    if q.sum() < 40:
        q = quiet.copy()
    g = A[q].mean(axis=0); u = g / np.linalg.norm(g)
    av = A.dot(u) - A[q].dot(u).mean()        # 静置段均值为 0（顺带扣零偏）
    # 静置段内残余的线性趋势也扣掉（更严格地估零偏）
    tt = t_u[q]
    p = np.polyfit(tt, av[q], 1)
    av_detr = av - np.polyval(p, t_u)

    dt = 1.0 / fs
    out = {}
    for name, sig in (("raw", av), ("detr", av_detr)):
        v = np.cumsum(sig) * dt
        d = np.cumsum(v) * dt
        d -= d[q][-1]                          # 静置段末位移记为 0
        out[name] = d
    return dict(t_u=t_u, d=out, av=av, h=h, pt=pt, pv=pv, peaks=peaks,
                onset=onset, fs=fs, u=u)


if __name__ == "__main__":
    for sim in SIMS:
        r = abs_integrate(sim)
        t_u = r["t_u"]; d = r["d"]["detr"]
        h = r["h"]; tp = h["tp"]; fs = r["fs"]
        print("=" * 74)
        print(sim, " 静置结束 %.2f s, 记录末 %.1f s" % (r["onset"], t_u[-1]))
        # 终点漂移：记录最后 0.5 s 的位移（应该回到 0 附近）
        tail = d[t_u > t_u[-1] - 0.5].mean()
        print("   记录末尾位移 %.1f mm  (若零偏残差为 0 应接近 0)  -> 漂移速率 %.3f mm/s"
              % (tail, tail / (t_u[-1] - r["onset"])))
        # 逐次按压：谷底相对静置的绝对深度
        ab, tr, rep = [], [], []
        for i in range(1, len(r["peaks"]) - 1):
            tpk = r["peaks"][i, 0]
            off = h["off"]
            lo = (tp[0] - off)                 # 粗略对齐
            m = (t_u >= tpk - off - 0.55) & (t_u <= tpk - off + 0.15)
            if m.sum() < 10:
                continue
            seg = d[m]
            ab.append(-seg.min())
            tr.append(seg.max() - seg.min())
            rep.append(r["peaks"][i, 1])
        ab = np.array(ab); tr = np.array(tr); rep = np.array(rep)
        sl = slice(6, len(ab) - 3)
        print("   绝对深度(积分) 均值 %.1f mm | 平台自报 %.1f mm | 差 %+.1f"
              % (ab[sl].mean(), rep[sl].mean(), ab[sl].mean() - rep[sl].mean()))
        print("   行程(积分)     均值 %.1f mm | 平台行程 %.1f mm"
              % (tr[sl].mean(), np.mean(tr[sl])))
        # 漂移的单调性：把绝对深度按时间打印几段
        n = len(ab)
        for a in range(0, n, max(1, n // 8)):
            print("      idx %2d  t=%5.1f  绝对深度 %6.1f  平台自报 %6.1f"
                  % (a, r["peaks"][a + 1, 0], ab[a], rep[a]))
