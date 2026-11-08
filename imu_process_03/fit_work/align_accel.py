# -*- coding: utf-8 -*-
"""
用"加速度本身"做时间对齐，彻底消除整周期歧义
=============================================
平台 mannequin_raw 是位移（mm），对它求二阶差分 ≈ 竖直加速度；
IMU 的带通信号也是竖直加速度。两者做互相关：
  正确对齐 -> 正相关峰；错半个周期 -> 负峰。所以不可能错一个周期。
"""
import io, os, sys, glob
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib as mpl
mpl.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
mpl.rcParams["axes.unicode_minus"] = False

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cpr_fit import (load_sensor, load_platform, find_platform, preprocess,
                     BASE, OUT, SIMS)


def align_accel(sim, plot=False):
    d = os.path.join(BASE, sim)
    imu = sorted(glob.glob(os.path.join(d, "lsm6dsv_raw_*.txt")))[-1]
    t, acc = load_sensor(imu)
    t_u, bp, av, fs_u = preprocess(t, acc)
    pt, pv, peaks = load_platform(find_platform(sim))

    # 平台：插值到 200 Hz，低通 8 Hz，再求两次差分
    fs = 200.0
    tp = np.arange(pt[0], pt[-1], 1.0 / fs)
    p = np.interp(tp, pt, pv)
    b, a = signal.butter(3, 8.0 / (fs / 2), btype="low")
    p = signal.filtfilt(b, a, p)
    ap = np.gradient(np.gradient(p, 1.0 / fs), 1.0 / fs)     # mm/s^2

    # 统一到同一时间轴，且都要覆盖重叠区
    t0 = max(t_u[0], tp[0]); t1 = min(t_u[-1], tp[-1])
    m = int((t1 - t0) * fs)
    g = np.arange(m) / fs + max(t0, 0)
    imu_i = np.interp(g, t_u, bp)
    pad_i = np.interp(g, tp, ap)
    imu_i = imu_i / (np.std(imu_i) + 1e-9)
    pad_i = pad_i / (np.std(pad_i) + 1e-9)

    lags = np.arange(-1.5, 1.5, 1.0 / fs)
    cc = np.empty(len(lags))
    for i, L in enumerate(lags):
        s = int(round(L * fs))
        if s >= 0:
            cc[i] = np.mean(imu_i[s:] * pad_i[:len(pad_i) - s])
        else:
            cc[i] = np.mean(imu_i[:s] * pad_i[-s:])
    k = int(np.argmax(cc))
    lag = lags[k]
    # 半个周期后的相关值（做对照）
    j = int(np.argmin(np.abs(lags - (lag + 0.25))))
    if plot:
        fig, ax = plt.subplots(2, 1, figsize=(11, 6))
        ax[0].plot(lags, cc, lw=1.2)
        ax[0].axvline(lag, color="r", ls="--")
        ax[0].set_xlabel("lag (s)"); ax[0].set_ylabel("归一化互相关")
        ax[0].set_title("%s 加速度互相关：正确对齐应为正峰，错半周期为负峰" % sim)
        ax[0].grid(alpha=0.3)
        # 对齐后的波形
        s = int(round(lag * fs))
        if s >= 0:
            a1 = imu_i[s:]; a2 = pad_i[:len(pad_i) - s]
        else:
            a1 = imu_i[:s]; a2 = pad_i[-s:]
        ax[1].plot(a2[600:1600], lw=0.9, alpha=0.8, label="平台二阶差分")
        ax[1].plot(a1[600:1600] / (np.std(a1) + 1e-9) * np.std(a2), lw=0.9, alpha=0.8,
                   label="IMU 带通（已按 lag 平移）")
        ax[1].set_xlabel("样本(200 Hz)"); ax[1].legend(fontsize=8); ax[1].grid(alpha=0.3)
        plt.tight_layout()
        plt.savefig(os.path.join(OUT, "align_%s.png" % sim), dpi=100)
        plt.close(fig)
    return lag, cc[k], cc[j]


if __name__ == "__main__":
    for i, s in enumerate(SIMS):
        lag, c1, c2 = align_accel(s, plot=True)
        print("%s  最佳 lag = %+.3f s  (corr %.3f)   半周期后相关 %.3f  -> %s"
              % (s, lag, c1, c2, "清晰" if c1 > -c2 else "不可靠"))
