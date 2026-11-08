# -*- coding: utf-8 -*-
"""Waveform-level comparison: reconstructed IMU displacement vs pad wave, plus
the sampling bias of the 20 Hz / 1 mm quantised pad as a reference."""
import io, os, csv, glob, sys
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cpr_fit import (load_sensor, load_platform, find_platform, preprocess,
                     find_positive_peaks, harmonic_travel, G)

SIMS = ["sim01", "sim02", "sim03"]


def disp_from_acc(seg, dt, w, K=1):
    """fit harmonics then evaluate twice-integrated displacement, zero-meaned"""
    n = len(seg)
    tt = np.arange(n) * dt
    cols = []
    for k in range(1, K + 1):
        cols.append(np.sin(k * w * tt))
        cols.append(np.cos(k * w * tt))
    M = np.column_stack(cols)
    coef, *_ = np.linalg.lstsq(M, seg, rcond=None)
    d = np.zeros(n)
    for k in range(1, K + 1):
        A, B = coef[2 * (k - 1)], coef[2 * (k - 1) + 1]
        wk = k * w
        d += -(A * np.sin(wk * tt) + B * np.cos(wk * tt)) / (wk * wk)
    return d * G


def pad_sub_travel(pt, pv, lo, hi, K=2):
    """用 2 次谐波模型拟合平台 20Hz 采样点，返回 (采样峰峰值, 连续模型峰峰值)"""
    m = (pt >= lo) & (pt <= hi)
    if m.sum() < 6:
        return np.nan, np.nan
    ts = pt[m]; vs = pv[m]
    T = hi - lo
    w = 2 * np.pi / T
    tau = ts - lo
    cols = [np.ones(len(tau))]
    for k in range(1, K + 1):
        cols.append(np.sin(k * w * tau)); cols.append(np.cos(k * w * tau))
    M = np.column_stack(cols)
    c, *_ = np.linalg.lstsq(M, vs, rcond=None)
    tg = np.linspace(0, T, 400)
    mod = np.full(len(tg), c[0])
    for k in range(1, K + 1):
        mod += c[2 * k - 1] * np.sin(k * w * tg) + c[2 * k] * np.cos(k * w * tg)
    return vs.max() - vs.min(), mod.max() - mod.min()


for sim in SIMS:
    d = os.path.join(BASE, sim)
    imu = sorted(glob.glob(os.path.join(d, "lsm6dsv_raw_*.txt")))[-1]
    t, acc = load_sensor(imu)
    pt, pv, peaks = load_platform(find_platform(sim))
    t_u, bp, av, fs_u = preprocess(t, acc)
    pk_idx, tp = find_positive_peaks(bp, fs_u)
    dt = 1.0 / fs_u

    print("=" * 70)
    print(sim)
    # ---- 平台参考自身的采样误差 ----
    diff, samp, cont = [], [], []
    for i in range(1, len(peaks) - 1):
        lo = peaks[i, 0] - (peaks[i, 0] - peaks[i - 1, 0])
        hi = peaks[i, 0]
        a, b = pad_sub_travel(pt, pv, lo, hi, K=2)
        if np.isfinite(a):
            samp.append(a); cont.append(b)
    samp = np.array(samp); cont = np.array(cont)
    print("  平台 20Hz 采样峰峰值 %.1f mm, 谐波重建连续峰峰值 %.1f mm, 低估 %.1f mm (%.1f%%), "
          "残差 SD %.1f mm"
          % (samp.mean(), cont.mean(), cont.mean() - samp.mean(),
             (cont.mean() - samp.mean()) / cont.mean() * 100,
             (cont - samp).std()))

    # ---- 波形图：取中间一段 ----
    k0 = len(tp) // 2 - 8
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=False)
    ax = axes[0]
    tt = np.arange(len(bp)) * dt
    for k in range(k0, k0 + 8):
        T = tp[k + 1] - tp[k]
        i0 = int(round(tp[k] * fs_u)); i1 = int(round(tp[k + 1] * fs_u))
        w = 2 * np.pi / T
        dd = disp_from_acc(bp[i0:i1], dt, w, 1)
        seg_t = (np.arange(i1 - i0)) * dt + (tt[i0] - tt[i0])
        ax.plot(seg_t, dd - dd.mean(), lw=1.2, label="cycle %d" % k if k < k0 + 3 else None)
    ax.set_title("IMU reconstructed displacement, 8 consecutive cycles (mm, zero-mean)")
    ax.set_ylabel("disp (mm)"); ax.grid(alpha=0.3); ax.legend(fontsize=7)

    # 叠图：一个周期内 IMU 位移 vs 平台
    ax = axes[1]
    for k in [k0 + 1, k0 + 3, k0 + 5]:
        T = tp[k + 1] - tp[k]
        i0 = int(round(tp[k] * fs_u)); i1 = int(round(tp[k + 1] * fs_u))
        w = 2 * np.pi / T
        dd = disp_from_acc(bp[i0:i1], dt, w, 1)
        ax.plot(np.arange(i1 - i0) * dt / T, dd, lw=1.6, label="IMU cyc %d" % k)
    ax.set_ylabel("disp (mm)"); ax.grid(alpha=0.3); ax.legend(fontsize=8)
    ax.set_title("single-cycle shapes (normalised time)")

    ax = axes[2]
    m = (pt > pt[pt < 15].max() - 6) & (pt < pt[pt < 15].max())
    ax.plot(pt[m], pv[m], "k.-", ms=3, lw=1)
    ax.set_ylabel("pad (mm)"); ax.set_xlabel("platform elapsed (s)"); ax.grid(alpha=0.3)
    ax.set_title("pad raw, 6 s")
    plt.tight_layout()
    plt.savefig(os.path.join(OUT, "wave_%s.png" % sim), dpi=105)
    plt.close(fig)
print("saved waveforms")
