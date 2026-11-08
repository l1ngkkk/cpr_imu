# -*- coding: utf-8 -*-
"""分析 LSM6DSV 原始采集文件：趋势、量程、采样率、按压次数"""
import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import os
path = os.path.join(os.environ["TEMP"], "lsm_raw.txt")

df = pd.read_csv(path, comment="#", header=None,
                 names=["t", "ax", "ay", "az", "gx", "gy", "gz", "temp"])

print("样本数:", len(df))
print("时间范围: %.3f ~ %.3f s" % (df.t.iloc[0] / 1000.0, df.t.iloc[-1] / 1000.0))

dt = np.diff(df.t.values) / 1000.0
print("采样间隔: 中位数 %.4f s, 最小 %.4f, 最大 %.4f" %
      (np.median(dt), dt.min(), dt.max()))
print("平均采样率: %.1f Hz" % (1.0 / np.mean(dt)))

# 原始 LSB -> 物理量（当前配置 ±4 g / ±500 dps）
acc = df[["ax", "ay", "az"]].values * 0.244        # mg (±8 g)
gyr = df[["gx", "gy", "gz"]].values * 0.0175       # dps

amag = np.sqrt((acc ** 2).sum(axis=1))
print("\n合加速度 |a|: 中位数 %.0f mg, 5%%分位 %.0f, 95%%分位 %.0f, 最大 %.0f" %
      (np.median(amag), np.percentile(amag, 5), np.percentile(amag, 95), amag.max()))

print("各轴范围 (mg):")
for i, name in enumerate(("ax", "ay", "az")):
    print("  %s: %7.0f ~ %7.0f  (均值 %6.0f)" %
          (name, acc[:, i].min(), acc[:, i].max(), acc[:, i].mean()))

print("陀螺范围 (dps):")
for i, name in enumerate(("gx", "gy", "gz")):
    print("  %s: %7.1f ~ %7.1f" % (name, gyr[:, i].min(), gyr[:, i].max()))

# 用静置段估计重力方向
quiet = amag < 1050
if quiet.sum() > 100:
    gvec = acc[quiet].mean(axis=0)
    print("\n静置段 %d 个样本，重力向量 (mg): [%.0f, %.0f, %.0f], |g|=%.0f" %
          (quiet.sum(), gvec[0], gvec[1], gvec[2], np.linalg.norm(gvec)))
else:
    gvec = np.array([0.0, 0.0, 1000.0])
    print("\n没有明显静置段，按 az 竖直处理")

u = gvec / np.linalg.norm(gvec)
av = acc.dot(u) - np.linalg.norm(gvec)      # 竖直运动加速度 mg

print("竖直加速度 av: 峰峰值 %.0f mg, 最大 %.0f, 最小 %.0f" %
      (av.max() - av.min(), av.max(), av.min()))

# 找按压段：|av| 超过 150 mg
active = np.abs(av) > 150
edges = np.diff(active.astype(int))
starts = np.where(edges == 1)[0] + 1
ends = np.where(edges == -1)[0] + 1
if active[0]:
    starts = np.r_[0, starts]
if active[-1]:
    ends = np.r_[ends, len(active)]
print("\n|av|>150mg 的活动段: %d 段" % len(starts))
for s, e in zip(starts[:15], ends[:15]):
    print("  %.2f ~ %.2f s, 持续 %.2f s, 峰值 %+.0f mg" %
          (df.t.iloc[s] / 1000.0, df.t.iloc[e - 1] / 1000.0,
           (df.t.iloc[e - 1] - df.t.iloc[s]) / 1000.0, av[s:e][np.argmax(np.abs(av[s:e]))]))

# 画图
t = df.t.values / 1000.0
fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

ax0 = axes[0]
ax0.plot(t, acc[:, 0], lw=0.8, label="ax")
ax0.plot(t, acc[:, 1], lw=0.8, label="ay")
ax0.plot(t, acc[:, 2], lw=0.8, label="az")
ax0.set_ylabel("acc (mg)")
ax0.legend(loc="upper right", fontsize=8)
ax0.grid(alpha=0.3)
ax0.set_title("LSM6DSV raw capture  (%s)" % path.split("\\")[-1])

ax1 = axes[1]
ax1.plot(t, amag, lw=0.8, color="k", label="|a|")
ax1.axhline(1000, color="r", ls="--", lw=0.8, label="1 g")
ax1.set_ylabel("|a| (mg)")
ax1.legend(loc="upper right", fontsize=8)
ax1.grid(alpha=0.3)

ax2 = axes[2]
ax2.plot(t, av, lw=0.8, color="tab:blue")
ax2.axhline(0, color="gray", lw=0.6)
ax2.set_ylabel("vertical acc (mg)")
ax2.set_xlabel("time (s)")
ax2.grid(alpha=0.3)

plt.tight_layout()
out = os.path.join(os.environ["TEMP"], "analysis.png")
plt.savefig(out, dpi=110)
print("\n图已保存:", out)
