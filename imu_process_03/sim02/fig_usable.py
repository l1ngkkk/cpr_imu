# -*- coding: utf-8 -*-
"""
可用方法的对比图（平台真值 vs 基波拟合 vs 逐周期积分）

生成 fig5_usable_compare.png：
    (a) 每次按压深度对比（RMS 用灰虚线标出，提示不可用）
    (b) 每次按压的误差（拟合 / 积分），带 ±5 mm 参考带
    (c) 8 秒窗口内的位移波形叠加：平台测距 vs IMU 解算
"""

import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import compare_platform as cp

BASE = os.path.dirname(os.path.abspath(__file__))


def main():
    t, acc = cp.load_sensor(cp.SENSOR)
    wave_t, wave_v, peak_t, peak_d = cp.load_platform(cp.PLATFORM)

    bp, fs, dt = cp.prepare(t, acc)

    # 带位移的周期检测（复制 detect_cycles，同时保存每段位移）
    rms = np.sqrt(np.mean(bp ** 2))
    hyst = max(40.0, min(100.0, 0.30 * rms))
    fall, pos = [], True
    for i in range(len(bp)):
        if pos:
            if bp[i] < -hyst:
                pos = False
                fall.append(i)
        else:
            if bp[i] > hyst:
                pos = True

    disp = np.zeros(len(bp))
    cyc = []
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
        d_fit = cp.fit_amp(seg, w, dt)

        a2 = seg * 9.80665
        v = np.cumsum(a2) * dt
        v -= np.linspace(v[0], v[-1], n)
        d = np.cumsum(v) * dt
        d -= np.linspace(d[0], d[-1], n)
        disp[i0:i1] = d

        if d_fit is None:
            continue
        cyc.append((t[i0], t[i0] + T / 2, T, d_rms, d_fit, d.max() - d.min()))

    cyc = np.array(cyc)

    # 平台真值
    gt_travel, gt_abs = cp.platform_depths(wave_t, wave_v, peak_t)
    offset, _ = cp.align(cyc[:, 1], peak_t)

    rows = []
    for tp, g, ga in zip(peak_t, gt_travel, gt_abs):
        if not np.isfinite(g):
            continue
        idx = int(np.argmin(np.abs(cyc[:, 1] - (tp + offset))))
        if abs(cyc[idx, 1] - (tp + offset)) > 0.25:
            continue
        rows.append((tp, g, cyc[idx, 3], cyc[idx, 4], cyc[idx, 5]))
    rows = np.array(rows)

    # 去掉前 3 个过渡周期
    r = rows[6:-2]
    gt = r[:, 1]
    d_rms = r[:, 2]
    d_fit = r[:, 3]
    d_int = r[:, 4]

    rest = np.median(wave_v[wave_t < 1.0])

    fig = plt.figure(figsize=(13, 11))
    gs = fig.add_gridspec(3, 1, height_ratios=[1.15, 1.0, 1.1], hspace=0.35)

    # ---------- (a) 深度对比 ----------
    ax = fig.add_subplot(gs[0])
    ax.plot(r[:, 0], gt, "k.-", ms=7, lw=1.4, label="platform (truth)")
    ax.plot(r[:, 0], d_fit, "b.-", ms=5, lw=1.1, label="fundamental fit")
    ax.plot(r[:, 0], d_int, "g.-", ms=5, lw=1.0, alpha=0.85, label="per-cycle integral")
    ax.plot(r[:, 0], d_rms, "--", color="0.65", lw=1.0, label="RMS (current, unusable)")
    ax.set_ylabel("depth (mm)")
    ax.set_xlabel("time (s)")
    ax.set_title("(a) Compression depth per beat", fontsize=11, loc="left")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9, ncol=4, loc="lower left")
    ax.set_ylim(min(gt.min(), d_int.min()) - 8, max(d_rms.max(), d_fit.max()) + 6)

    # ---------- (b) 误差 ----------
    ax = fig.add_subplot(gs[1])
    e_fit = d_fit - gt
    e_int = d_int - gt
    ax.axhspan(-5, 5, color="tab:green", alpha=0.10)
    ax.axhline(0, color="k", lw=0.8)
    ax.bar(r[:, 0] - 0.10, e_fit, width=0.20, color="tab:blue",
           alpha=0.85, label="fit - platform")
    ax.bar(r[:, 0] + 0.10, e_int, width=0.20, color="tab:green",
           alpha=0.65, label="integral - platform")
    ax.set_ylabel("error (mm)")
    ax.set_xlabel("time (s)")
    ax.set_title("(b) Per-beat error   (shaded band = ±5 mm)", fontsize=11, loc="left")
    ax.grid(alpha=0.3, axis="y")
    ax.legend(fontsize=9)
    ax.set_ylim(-15, 15)

    # ---------- (c) 波形叠加 ----------
    ax = fig.add_subplot(gs[2])
    t0, t1 = 14.0, 22.0
    m = (wave_t >= t0) & (wave_t <= t1)
    plat_disp = wave_v[m] - rest
    ax.plot(wave_t[m], plat_disp, "k-", lw=1.6, label="platform displacement")

    ms = (t >= t0 + offset) & (t <= t1 + offset)
    imu_disp = disp[ms]
    imu_disp = imu_disp - imu_disp.mean() + plat_disp.mean()
    ax.plot(t[ms] - offset, imu_disp, "b-", lw=1.2, alpha=0.9,
            label="IMU solved (per-cycle integral)")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("displacement (mm)")
    ax.set_title("(c) Waveform overlay, 14-22 s", fontsize=11, loc="left")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9, loc="upper right")

    plt.savefig(os.path.join(BASE, "fig5_usable_compare.png"), dpi=110,
                bbox_inches="tight")
    plt.close(fig)

    print("fit  : bias %+.1f mm, MAE %.1f mm, MAPE %.1f%%" %
          (e_fit.mean(), np.abs(e_fit).mean(),
           np.abs(e_fit / gt * 100).mean()))
    print("integ: bias %+.1f mm, MAE %.1f mm, MAPE %.1f%%" %
          (e_int.mean(), np.abs(e_int).mean(),
           np.abs(e_int / gt * 100).mean()))
    print("saved:", os.path.join(BASE, "fig5_usable_compare.png"))


if __name__ == "__main__":
    main()
