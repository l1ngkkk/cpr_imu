# -*- coding: utf-8 -*-
"""
对比三种深度解算方法（以“逐周期积分”为参考）

    1. RMS 法      : d = 2√2·a_rms/ω²      （固件现在用的）
    2. 基波拟合法  : 最小二乘拟合 a(t)=A·sinωt+B·cosωt，d = 2√(A²+B²)/ω²
    3. 逐周期积分  : 两次积分 + 首尾对齐（参考真值）

用法：
    python fit_compare.py
"""

import io
import os
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BASE = os.path.dirname(os.path.abspath(__file__))

FILES = [
    (os.path.join(BASE, "原始数据采集", "lsm6dsv_raw_20261105_112615.txt"), "5~6 cm, 89 cpm"),
    (os.path.join(BASE, "lsm6dsv_raw_20261105_141646.txt"), "1 cm, 151 cpm"),
]


def load(path):
    lsb = 0.122
    t, acc = [], []
    for line in io.open(path, encoding="utf-8", errors="replace"):
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        p = s.split(",")
        if len(p) != 8:
            continue
        try:
            v = [int(x) for x in p]
        except ValueError:
            continue
        t.append(v[0] / 1000.0)
        acc.append(v[1:4])
    t = np.array(t)
    acc = np.array(acc, float) * lsb
    fs = 1.0 / np.mean(np.diff(t))
    dt = 1.0 / fs

    amag = np.sqrt((acc ** 2).sum(axis=1))
    q = (amag > 920) & (amag < 1080)
    g = acc[q].mean(axis=0)
    g /= np.linalg.norm(g)
    av = acc.dot(g) - np.linalg.norm(acc[q].mean(axis=0))

    b, a = signal.butter(4, [1.0 / (fs / 2), 8.0 / (fs / 2)], btype="band")
    return t, signal.filtfilt(b, a, av), fs, dt


def fit_amp(seg, w, dt):
    """最小二乘拟合基波，返回深度 mm"""
    n = len(seg)
    tt = np.arange(n) * dt
    S = np.sin(w * tt)
    C = np.cos(w * tt)
    m11 = S @ S
    m22 = C @ C
    m12 = S @ C
    det = m11 * m22 - m12 * m12
    if abs(det) < 1e-9:
        return None
    r1 = S @ seg
    r2 = C @ seg
    A = (m22 * r1 - m12 * r2) / det
    B = (-m12 * r1 + m11 * r2) / det
    return 2.0 * np.hypot(A, B) * 9.80665 / (w * w)


def cycles(t, bp, dt):
    rms = np.sqrt(np.mean(bp ** 2))
    hyst = max(40.0, min(100.0, 0.30 * rms))
    fall = []
    pos = True
    for i in range(len(bp)):
        if pos:
            if bp[i] < -hyst:
                pos = False
                fall.append(i)
        else:
            if bp[i] > hyst:
                pos = True

    out = []
    for k in range(len(fall) - 1):
        i0, i1 = fall[k], fall[k + 1]
        if i1 - i0 < 10:
            continue
        T = t[i1] - t[i0]
        if not (0.3 <= T <= 1.2):
            continue
        seg = bp[i0:i1]
        n = len(seg)
        w = 2 * np.pi / T

        d_rms = 2.8284271 * np.sqrt(np.mean(seg ** 2)) * 9.80665 / (w * w)
        d_fit = fit_amp(seg, w, dt)

        a2 = seg * 9.80665
        v = np.cumsum(a2) * dt
        v -= np.linspace(v[0], v[-1], n)
        d = np.cumsum(v) * dt
        d -= np.linspace(d[0], d[-1], n)
        d_int = d.max() - d.min()

        if d_fit is None:
            continue
        out.append((t[i0], T, d_rms, d_fit, d_int))
    return np.array(out)


def main():
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    all_ref, all_rms, all_fit = [], [], []

    for col, (name, label) in enumerate(FILES):
        t, bp, fs, dt = load(name)
        c = cycles(t, bp, dt)
        ref = c[:, 4]
        rms = c[:, 2]
        fit = c[:, 3]

        all_ref.append(ref)
        all_rms.append(rms)
        all_fit.append(fit)

        err_rms = (rms - ref) / ref * 100
        err_fit = (fit - ref) / ref * 100

        print("=" * 66)
        print("%s  (%s)   周期 %d" % (name.split("\\")[-1], label, len(ref)))
        print("  参考(积分)  中位 %6.1f mm" % np.median(ref))
        print("  RMS 法      中位 %6.1f mm   平均偏差 %+6.1f%%   平均|误差| %.1f%%" %
              (np.median(rms), err_rms.mean(), np.abs(err_rms).mean()))
        print("  基波拟合    中位 %6.1f mm   平均偏差 %+6.1f%%   平均|误差| %.1f%%" %
              (np.median(fit), err_fit.mean(), np.abs(err_fit).mean()))

        ax = axes[0, col]
        idx = np.arange(1, len(ref) + 1)
        ax.plot(idx, ref, "k.-", ms=5, lw=1.0, label="integration (ref)")
        ax.plot(idx, fit, "b.-", ms=5, lw=0.9, label="fundamental fit")
        ax.plot(idx, rms, "r.-", ms=4, lw=0.9, alpha=0.8, label="RMS (current)")
        ax.set_title(label, fontsize=10)
        ax.set_xlabel("cycle #")
        ax.set_ylabel("depth (mm)")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)

    ref = np.concatenate(all_ref)
    rms = np.concatenate(all_rms)
    fit = np.concatenate(all_fit)

    ax = axes[1, 0]
    ax.scatter(ref, fit, s=18, c="tab:blue", label="fundamental fit", alpha=0.8)
    ax.scatter(ref, rms, s=18, c="tab:red", label="RMS", alpha=0.6)
    lim = [0, max(ref.max(), rms.max()) * 1.1]
    ax.plot(lim, lim, "k--", lw=1.0, label="perfect")
    ax.set_xlabel("integration depth (mm)")
    ax.set_ylabel("estimated depth (mm)")
    ax.set_title("all cycles: estimate vs reference", fontsize=10)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes[1, 1]
    bins = np.linspace(-60, 60, 25)
    ax.hist((rms - ref) / ref * 100, bins=bins, color="tab:red",
            alpha=0.55, label="RMS")
    ax.hist((fit - ref) / ref * 100, bins=bins, color="tab:blue",
            alpha=0.55, label="fundamental fit")
    ax.axvline(0, color="k", lw=0.8)
    ax.set_xlabel("relative error (%)")
    ax.set_ylabel("cycles")
    ax.set_title("error distribution (all cycles)", fontsize=10)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    plt.tight_layout()
    out = r"C:\Users\Han_y\Desktop\本科毕业设计\imu_process_02\fit_vs_rms.png"
    plt.savefig(out, dpi=110)
    plt.close(fig)

    e_rms = np.abs((rms - ref) / ref * 100)
    e_fit = np.abs((fit - ref) / ref * 100)
    print("\n" + "=" * 66)
    print("全部 %d 个周期合计：" % len(ref))
    print("  RMS 法     平均|误差| %.1f%%   最大 %.1f%%" % (e_rms.mean(), e_rms.max()))
    print("  基波拟合   平均|误差| %.1f%%   最大 %.1f%%" % (e_fit.mean(), e_fit.max()))
    print("图已保存:", out)


if __name__ == "__main__":
    main()
