# -*- coding: utf-8 -*-
"""板级验证汇总：三次实验的板子 EVL 对比图 + 旧算法丢按压的原因。"""
import io, os, sys, glob
import numpy as np
from scipy.signal import lfilter
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cpr_fit import find_platform
import board_check as B

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(HERE, "fit_work", "out")

LOGS = [("sim01", "board_logs/board_sim01_20261105_224043.txt"),
        ("sim02", "board_logs/board_sim02_20261105_224121.txt"),
        ("sim03", "board_logs/board_sim03_20261105_224156.txt")]


def biquad(fs, fc, hp):
    w0 = 2 * np.pi * fc / fs
    cw = np.cos(w0); sw = np.sin(w0)
    al = sw / 1.41421356
    a0 = 1 + al
    if hp:
        b = np.array([(1 + cw) * .5, -(1 + cw), (1 + cw) * .5]) / a0
    else:
        b = np.array([(1 - cw) * .5, (1 - cw), (1 - cw) * .5]) / a0
    return b, np.array([1.0, -2 * cw / a0, (1 - al) / a0])


fig, axes = plt.subplots(3, 1, figsize=(13, 9.4))
rows = []
for ax, (sim, log) in zip(axes, LOGS):
    d = os.path.join(HERE, sim)
    raw = sorted(glob.glob(os.path.join(d, "lsm6dsv_raw_*.txt")))[-1]
    t, a = B.read_raw(raw)
    wt, wv, pk = B.read_platform(find_platform(d))
    ref, rest = B.platform_travel(wt, wv, pk)
    ev, evl, dv = B.parse_log(os.path.join(HERE, log))

    m = B.match(evl[:, 0], evl[:, 1], pk[:, 0], ref)
    gt, est, off, s = m
    sc = B.score(gt, est)
    K = sc["K"]
    rows.append((sim, len(ev), len(evl), sc))

    idx = np.arange(len(gt))
    ax.plot(idx, gt, "k.-", ms=5, lw=1.2, label="平台 travel（真值 = 一个周期峰峰值）")
    rep = pk[:len(gt), 1]
    ax.plot(idx, rep, color="0.55", ls="--", lw=1.1,
            label="平台自报 mannequin_depth（含手压偏置，高 ~12 mm）")
    ax.plot(np.arange(len(est)), K * est, color="tab:cyan", lw=1.6,
            label="板子 EVL（漏积分）  MAE %.2f mm / %.1f%%" % (sc["mae"], sc["mape"]))
    if len(ev):
        m2 = B.match(ev[:, 0], ev[:, 1], pk[:, 0], ref)
        if m2:
            gt2, est2, _, _ = m2
            if len(est2) >= 8:
                K2 = gt2.mean() / est2.mean()
                ax.plot(np.arange(len(est2)), K2 * est2, color="tab:red", lw=1.2,
                        alpha=0.8, label="板子 EV（旧 dft1） 只有 %d 次" % len(ev))
    ax.set_ylabel("深度 (mm)")
    ax.set_title("%s" % sim, fontsize=10)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="lower right")

axes[-1].set_xlabel("第几次按压")
plt.tight_layout()
png = os.path.join(OUT, "board_summary.png")
plt.savefig(png, dpi=110)
plt.close(fig)
print("图:", png)

print("\n=== 板级验证汇总 ===")
print("%-7s %8s %8s %10s %8s" % ("sim", "EV(旧)", "EVL(新)", "EVL MAE", "MAPE"))
for sim, n1, n2, sc in rows:
    print("%-7s %8d %8d %8.2f mm %7.1f%%" % (sim, n1, n2, sc["mae"], sc["mape"]))

# ---- 旧算法丢按压的原因：half 与 0.75*period_avg 的关系 ----
print("\n=== 旧算法门限诊断 ===")
print("门限: (2*half) >= 0.75*period_avg 且 half in [0.12, 0.50]")
for sim, log in LOGS:
    d = os.path.join(HERE, sim)
    raw = sorted(glob.glob(os.path.join(d, "lsm6dsv_raw_*.txt")))[-1]
    t_ms, a = B.read_raw(raw)
    fs = 1000.0 / np.mean(np.diff(t_ms))
    amag = np.linalg.norm(a, axis=1)
    quiet = (amag > 900) & (amag < 1100)
    g = a[quiet].mean(axis=0); u = g / np.linalg.norm(g)
    av = a.dot(u) - 1000.0
    b, aa = biquad(fs, 1.0, True); bp = lfilter(b, aa, av); bp = lfilter(b, aa, bp)
    b, aa = biquad(fs, 8.0, False); bp = lfilter(b, aa, bp)
    T = 0.5
    halves = []
    state = 0
    t_cross = None
    for i in range(1, len(bp)):
        hyst = 80.0
        if state == 0 and bp[i] < -hyst:
            state = 1; t_cross = t_ms[i]
        elif state == 1 and bp[i] > hyst:
            halves.append((t_ms[i] - t_cross) / 1000.0)
            state = 0
    halves = np.array(halves)
    halves = halves[(halves > 0.05) & (halves < 1.0)]
    frac = np.mean(2 * halves < 0.75 * T) * 100
    print("  %s  half 中位 %.3f s (2*half=%.3f), 门限 0.75T=%.3f -> 会被判为"
          "\"回弹小起伏\"而丢弃的占 %.0f%%"
          % (sim, np.median(halves), 2 * np.median(halves), 0.75 * T, frac))
