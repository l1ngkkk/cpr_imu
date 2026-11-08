# -*- coding: utf-8 -*-
"""
两种深度算法的直观对比图
========================
上：竖直加速度 av 与两次积分得到的位移 d —— 说明"积分两次就是位移"
中：旧方法（单频正交 dft1）—— 一个周期内拟合一条正弦，只取基波幅值
下：新方法（漏积分）—— 位移的峰谷差，速度过零就是峰谷所在

输出 fit_work/out/method_compare.png
"""
import os
import sys
import numpy as np
from scipy.signal import lfilter, find_peaks
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import board_check as B

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(HERE, "fit_work", "out")


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


sim = "sim03"
raw, plat = B.load_sim(os.path.join(HERE, sim))
t_ms, a = B.read_raw(raw)

# 跑一遍固件的链，拿到 av / bp / disp / vel
ev, disp, vel, av = B.replica_leaky(t_ms, a)
fs = 1000.0 / np.mean(np.diff(t_ms))
bb, aa = biquad(fs, 1.0, True)
bp = lfilter(bb, aa, av); bp = lfilter(bb, aa, bp)
bb, aa = biquad(fs, 8.0, False); bp = lfilter(bb, aa, bp)

t = t_ms / 1000.0
t0 = 12.0                      # 取中间一段
sel = (t >= t0) & (t <= t0 + 2.2)
idx, _ = find_peaks(bp, distance=int(0.3 * fs), prominence=0.15 * np.std(bp))

fig, axes = plt.subplots(3, 1, figsize=(12.5, 10.2))

# ---------------- 上：av 与 disp ----------------
ax = axes[0]
ax.plot(t[sel], av[sel], color="0.6", lw=0.9, label="竖直加速度 $a_v$ (mg)")
ax.set_ylabel("加速度 $a_v$ (mg)", color="0.4")
ax.tick_params(axis="y", labelcolor="0.4")
ax2 = ax.twinx()
ax2.plot(t[sel], disp[sel], color="tab:blue", lw=1.8, label="两次积分后的位移 d (mm)")
ax2.set_ylabel("位移 d (mm)", color="tab:blue")
ax2.tick_params(axis="y", labelcolor="tab:blue")
ax.set_title("① 加速度积分两次就是位移：  $v=\\int a\\,dt$ ,  $d=\\int v\\,dt$")
ax.grid(alpha=0.25)
ax.set_xlim(t0, t0 + 2.2)

# ---------------- 中：旧方法 ----------------
ax = axes[1]
m = (t >= t0) & (t <= t0 + 2.2)
ax.plot(t[sel], bp[sel], color="tab:red", lw=1.1,
        label="带通后的加速度 $a_{bp}$ (mg)")

# 取中间那一个完整周期做演示
k = [i for i in idx if t[i] > t0 + 0.6][0]
pos = int(np.where(idx == k)[0][0])
ta, tb = t[idx[pos]], t[idx[pos + 1]]
T = tb - ta
w = 2 * np.pi / T
i0 = int(np.ceil(ta * fs)); i1 = int(np.floor(tb * fs))
seg = bp[i0:i1]
tt = t[i0:i1] - tb
A = 2.0 / len(seg) * np.hypot(np.dot(seg, np.sin(w * tt)), np.dot(seg, np.cos(w * tt)))
S = np.dot(seg, np.sin(w * tt)); C = np.dot(seg, np.cos(w * tt))
ph = np.arctan2(C, S)
fit = A * np.sin(w * (t[i0:i1] - ta) + ph)

ax.axvspan(ta, tb, color="tab:red", alpha=0.10)
ax.plot(t[i0:i1], fit, "k--", lw=1.4, label="拟合出来的基波（只有这一条正弦）")
ax.set_ylabel("加速度 (mg)")
ax.set_xlabel("时间 (s)")
ax.set_title("② 旧方法 dft1：在一个周期内拟合正弦，取基波幅值 $A$，"
             "再算 $d=2A/\\omega^2$（$\\omega=2\\pi/T$）")
ax.grid(alpha=0.25)
ax.legend(fontsize=8.5, loc="upper right")
ax.set_xlim(t0, t0 + 2.2)
ax.annotate("$T=%.0f$ ms" % (T * 1000), xy=((ta + tb) / 2, ax.get_ylim()[0] * 0.8),
            ha="center", fontsize=9, color="tab:red")

# ---------------- 下：新方法 ----------------
ax = axes[2]
ax.plot(t[sel], disp[sel], color="tab:blue", lw=1.8, label="位移 d (mm)")
dsel = disp[sel]
ylo, yhi = dsel.min() - 6, dsel.max() + 6
ax.set_ylim(ylo, yhi)

# 在选中的周期里标出峰谷
i0 = int(np.ceil(ta * fs)); i1 = int(np.floor(tb * fs))
seg_d = disp[i0:i1]
imax = i0 + int(np.argmax(seg_d)); imin = i0 + int(np.argmin(seg_d))
ax.axhspan(disp[imin], disp[imax], color="tab:blue", alpha=0.10)
ax.annotate("", xy=(t[imax], disp[imax]), xytext=(t[imax], disp[imin]),
            arrowprops=dict(arrowstyle="<->", color="k", lw=1.3))
ax.annotate("深度 = K × (峰 − 谷)\n= %.1f mm" % (0.914 * (disp[imax] - disp[imin])),
            xy=(t[imax] + 0.06, 0.5 * (disp[imax] + disp[imin])), fontsize=9.5)
ax.plot(t[imax], disp[imax], "v", color="k", ms=7)
ax.plot(t[imin], disp[imin], "^", color="k", ms=7)
vzero = np.where(np.diff(np.sign(vel[sel])) != 0)[0]
ax.plot(t[sel][vzero], disp[sel][vzero], "r.", ms=6, label="速度过零点（= 位移极值）")
ax.set_ylabel("位移 (mm)")
ax.set_xlabel("时间 (s)")
ax.set_title("③ 新方法 漏积分：不假设波形，直接在速度过零处取位移的峰谷差")
ax.grid(alpha=0.25)
ax.legend(fontsize=8.5, loc="upper right")
ax.set_xlim(t0, t0 + 2.2)

plt.tight_layout()
png = os.path.join(OUT, "method_compare.png")
plt.savefig(png, dpi=115)
plt.close(fig)
print("图:", png)
