# -*- coding: utf-8 -*-
"""
实验平台对比：IMU 解算深度 vs 平台测距（真值）

输入：
    ../imu_process_02/lsm6dsv_raw_20261105_154711.txt   IMU 原始数据
    platform_raw.csv                                    平台测量数据（已复制到本目录）

输出（都在本目录）：
    report.txt              文字报告
    cycles_compare.csv      每个周期的对比表
    fig1_depth_timeline.png 深度-时间曲线（平台 vs 三种方法）
    fig2_scatter.png        散点图（估计 vs 真值）
    fig3_error.png          误差分布
    fig4_waveform.png       同一段时间的波形对照

方法：
    RMS 法      d = 2√2·a_rms/ω²            （固件当前使用）
    基波拟合    最小二乘拟合 a(t)=A sinωt+B cosωt，d = 2√(A²+B²)/ω²
    逐周期积分  两次积分 + 首尾对齐         （离线参考）
"""

import os
import io
import csv
import glob
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BASE = os.path.dirname(os.path.abspath(__file__))


def _find_inputs():
    """自动识别本目录下的 IMU 原始数据与平台测量数据"""
    imu = sorted(glob.glob(os.path.join(BASE, "lsm6dsv_raw_*.txt")))
    if not imu:
        raise SystemExit("本目录找不到 lsm6dsv_raw_*.txt")
    plat = os.path.join(BASE, "platform_raw.csv")
    if not os.path.exists(plat):
        cands = [c for c in glob.glob(os.path.join(BASE, "*", "*.csv"))
                 if "校验" not in os.path.basename(c)]
        if not cands:
            raise SystemExit("找不到平台测量数据 csv")
        plat = cands[0]
    return imu[-1], plat


SENSOR, PLATFORM = _find_inputs()

ACC_LSB = 0.122


# ----------------------------------------------------------------------
#  读取
# ----------------------------------------------------------------------
def load_sensor(path):
    t, acc = [], []
    with io.open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
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
    acc = np.array(acc, float) * ACC_LSB
    return t, acc


def load_platform(path):
    """返回 (波形 t, 波形 mm, 峰值事件 t, 平台报告的深度 cm)"""
    wave_t, wave_v = [], []
    peak_t, peak_d = [], []
    with io.open(path, encoding="utf-8-sig", errors="replace", newline="") as f:
        for row in csv.DictReader(f):
            if row["source"] == "mannequin" and row["mannequin_raw"]:
                try:
                    wave_t.append(float(row["elapsed_s"]))
                    wave_v.append(float(row["mannequin_raw"]))
                except ValueError:
                    pass
            elif row["source"] == "mannequin_peak" and row["mannequin_depth"]:
                try:
                    peak_t.append(float(row["elapsed_s"]))
                    peak_d.append(float(row["mannequin_depth"]))
                except ValueError:
                    pass

    order = np.argsort(wave_t)
    wave_t = np.array(wave_t)[order]
    wave_v = np.array(wave_v)[order]

    order = np.argsort(peak_t)
    return wave_t, wave_v, np.array(peak_t)[order], np.array(peak_d)[order]


def platform_depths(wave_t, wave_v, peak_t):
    """返回 (行程, 绝对深度)
    行程   = 本周期最低点 - 之前 0.5 s 内的最高点（真正的上下行程）
    绝对深度 = 最低点 - 静置基线（平台自报的口径）
    """
    rest = np.median(wave_v[wave_t < 1.0]) if (wave_t < 1.0).sum() > 3 else wave_v.min()
    travel, absolute = [], []
    for tp in peak_t:
        m = (wave_t >= tp - 0.5) & (wave_t <= tp + 0.12)
        if m.sum() < 5:
            travel.append(np.nan); absolute.append(np.nan); continue
        seg = wave_v[m]
        vmax = seg.max()
        travel.append(vmax - seg.min())
        absolute.append(vmax - rest)
    return np.array(travel), np.array(absolute)


# ----------------------------------------------------------------------
#  IMU 处理
# ----------------------------------------------------------------------
def prepare(t, acc):
    fs = 1.0 / np.mean(np.diff(t))
    dt = 1.0 / fs
    amag = np.sqrt((acc ** 2).sum(axis=1))
    q = (amag > 920) & (amag < 1080)
    g = acc[q].mean(axis=0)
    g /= np.linalg.norm(g)
    av = acc.dot(g) - np.linalg.norm(acc[q].mean(axis=0))
    b, a = signal.butter(4, [1.0 / (fs / 2), 8.0 / (fs / 2)], btype="band")
    bp = signal.filtfilt(b, a, av)
    return bp, fs, dt


def fit_amp(seg, w, dt):
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


def detect_cycles(t, bp, dt):
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
        out.append((t[i0], t[i0] + T / 2, T, d_rms, d_fit, d_int))
    return np.array(out)


# ----------------------------------------------------------------------
#  对齐
# ----------------------------------------------------------------------
def align(sensor_times, plat_times):
    """找时间偏移，使平台峰值与传感器周期最近邻距离最小"""
    best = (1e9, 0.0)
    for off in np.arange(-6.0, 12.0, 0.01):
        d = np.abs(sensor_times[None, :] - (plat_times[:, None] + off))
        dist = d.min(axis=1).mean()
        if dist < best[0]:
            best = (dist, off)
    return best[1], best[0]


def main():
    t, acc = load_sensor(SENSOR)
    wave_t, wave_v, peak_t, peak_d = load_platform(PLATFORM)

    bp, fs, dt = prepare(t, acc)
    cyc = detect_cycles(t, bp, dt)
    cyc_t = cyc[:, 1]

    offset, dist = align(cyc_t, peak_t)

    gt_travel, gt_abs = platform_depths(wave_t, wave_v, peak_t)
    gt = gt_travel
    gt_cm = peak_d * 10.0

    # 平台波形本身的分析：静置基线、回弹顶点、谷底
    rest = np.median(wave_v[wave_t < 1.0]) if (wave_t < 1.0).sum() > 3 else wave_v.min()
    tops, bots = [], []
    for tp in peak_t:
        m = (wave_t >= tp - 0.5) & (wave_t <= tp + 0.12)
        if m.sum() >= 5:
            tops.append(wave_v[m].min())
            bots.append(wave_v[m].max())
    tops = np.array(tops)
    bots = np.array(bots)        # 平台报告的深度（cm -> mm）

    # 每个平台峰值配一个最近的传感器周期
    rows = []
    for tp, g, ga in zip(peak_t, gt_travel, gt_abs):
        if not np.isfinite(g):
            continue
        idx = int(np.argmin(np.abs(cyc_t - (tp + offset))))
        if abs(cyc_t[idx] - (tp + offset)) > 0.25:
            continue
        _, tc, T, d_rms, d_fit, d_int = cyc[idx]
        rows.append((tp, T, g, ga, d_rms, d_fit, d_int))

    rows = np.array(rows)
    if len(rows) > 10:
        rows_stats = rows[6:-2]      # 去掉开头 6 个（基线还在下降）和末尾 2 个
    else:
        rows_stats = rows

    # ---------------- 统计 ----------------
    lines = []
    def say(s=""):
        print(s)
        lines.append(s)

    say("=" * 72)
    say("实验平台对比报告")
    say("IMU 数据: %s" % os.path.basename(SENSOR))
    say("平台数据: %s" % os.path.basename(PLATFORM))
    say("IMU 采样率 %.1f Hz, 平台 20 Hz" % fs)
    say("对齐偏移: 传感器时间 = 平台时间 + %.2f s (平均最近邻 %.3f s)" % (offset, dist))
    say("匹配上的周期: %d 个（平台峰值 %d 个）" % (len(rows), len(peak_t)))
    say("")

    say("（统计时去掉开头 6 个和末尾 2 个过渡周期）")
    say("")
    gt = rows_stats[:, 2]
    gt_report = rows_stats[:, 3]
    methods = [("RMS (current)", rows_stats[:, 4]), ("fundamental fit", rows_stats[:, 5]), ("per-cycle integral", rows_stats[:, 6])]

    say("%-12s %8s %8s %8s %8s %8s" % ("方法", "均值", "中位", "偏差", "MAE", "MAPE"))
    for name, v in methods:
        bias = (v - gt).mean()
        mae = np.abs(v - gt).mean()
        mape = np.abs((v - gt) / gt * 100).mean()
        say("%-12s %8.1f %8.1f %+8.1f %8.1f %7.1f%%" %
            (name, v.mean(), np.median(v), bias, mae, mape))

    say("")
    say("平台波形: 静置基线 %.1f mm, 回弹顶点 %.1f mm, 压缩谷底 %.1f mm" %
        (rest, tops.mean(), bots.mean()))
    say("  行程(谷底-顶点)   %.1f mm" % (bots - tops).mean())
    say("  绝对深度(谷底-静置) %.1f mm" % (bots - rest).mean())
    say("  回弹残差(顶点-静置) %.1f mm, 占绝对深度 %.0f%%" %
        ((tops - rest).mean(), (tops - rest).mean() / (bots - rest).mean() * 100))
    say("")
    say("两种真值口径下的平均误差：")
    say("  %-20s %12s %12s" % ("方法", "vs 行程", "vs 绝对深度"))
    for name, v in methods:
        e1 = (v - gt).mean() / gt.mean() * 100
        e2 = (v - rows_stats[:, 3]).mean() / rows_stats[:, 3].mean() * 100
        say("  %-20s %+11.1f%% %+11.1f%%" % (name, e1, e2))
    say("")
    say("说明：IMU 的高通滤波去掉了慢变基线，因此测的是行程；")
    say("      若回弹不充分（顶点高于静置位置），行程会小于从静置算起的绝对深度。")
    say("")

    for name, v in methods:
        k = np.polyfit(gt, v, 1)
        r = np.corrcoef(gt, v)[0, 1]
        say("%-12s 线性拟合: y = %.3f x + %.1f   相关系数 %.3f" %
            (name, k[0], k[1], r))

    # ---------------- 文件 ----------------
    with io.open(os.path.join(BASE, "cycles_compare.csv"), "w",
                 encoding="utf-8", newline="") as f:
        f.write("platform_t_s,period_s,gt_mm,platform_reported_mm,"
                "rms_mm,fit_mm,integral_mm\n")
        for r in rows:
            f.write("%.2f,%.3f,%.1f,%.1f,%.1f,%.1f,%.1f\n" %
                    (r[0], r[1], r[2], r[3], r[4], r[5], r[6]))

    # ---------------- 图 1：深度-时间 ----------------
    fig, ax = plt.subplots(figsize=(13, 6))
    ax.plot(rows[:, 0], rows[:, 2], "k.-", ms=6, lw=1.2, label="platform travel (truth)")
    ax.plot(rows[:, 0], rows[:, 4], "r.-", ms=5, lw=0.9, alpha=0.8, label="RMS (current)")
    ax.plot(rows[:, 0], rows[:, 5], "b.-", ms=5, lw=0.9, alpha=0.9, label="fundamental fit")
    ax.plot(rows[:, 0], rows[:, 6], "g.-", ms=4, lw=0.9, alpha=0.7, label="per-cycle integral")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("compression depth (mm)")
    ax.set_title("Depth: platform ground truth vs IMU estimates")
    ax.grid(alpha=0.3)
    ax.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(BASE, "fig1_depth_timeline.png"), dpi=110)
    plt.close(fig)

    # ---------------- 图 2：散点 ----------------
    fig, ax = plt.subplots(figsize=(7, 6.5))
    for (name, v), c, m in zip(methods, ["tab:red", "tab:blue", "tab:green"], ["o", "s", "^"]):
        ax.scatter(gt, v, s=26, c=c, marker=m, alpha=0.75, label=name)
    lo = min(gt.min(), rows_stats[:, 4:7].min()) - 4
    hi = max(gt.max(), rows_stats[:, 4:7].max()) + 4
    lim = [lo, hi]
    ax.plot(lim, lim, "k--", lw=1.0, label="perfect")
    ax.set_xlim(lim); ax.set_ylim(lim)
    ax.set_xlabel("platform depth (mm)")
    ax.set_ylabel("IMU estimated depth (mm)")
    ax.set_title("Estimate vs ground truth")
    ax.grid(alpha=0.3)
    ax.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(BASE, "fig2_scatter.png"), dpi=110)
    plt.close(fig)

    # ---------------- 图 3：误差分布 ----------------
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    bins = np.linspace(-30, 30, 25)
    for (name, v), c in zip(methods, ["tab:red", "tab:blue", "tab:green"]):
        axes[0].hist((v - gt) / gt * 100, bins=bins, alpha=0.5, color=c, label=name)
        err = v - gt
        axes[1].plot(gt, err, ".", color=c, alpha=0.7, label=name)
    axes[0].axvline(0, color="k", lw=0.8)
    axes[0].set_xlabel("relative error (%)"); axes[0].set_ylabel("count")
    axes[0].set_title("relative error"); axes[0].grid(alpha=0.3); axes[0].legend(fontsize=8)
    axes[1].axhline(0, color="k", lw=0.8)
    axes[1].set_xlabel("platform depth (mm)"); axes[1].set_ylabel("error (mm)")
    axes[1].set_title("absolute error vs depth"); axes[1].grid(alpha=0.3); axes[1].legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(os.path.join(BASE, "fig3_error.png"), dpi=110)
    plt.close(fig)

    # ---------------- 图 4：波形对照 ----------------
    fig, axes = plt.subplots(2, 1, figsize=(13, 7), sharex=False)
    m = (t >= 10) & (t <= 20)
    axes[0].plot(t[m], bp[m], lw=0.8, color="tab:blue")
    axes[0].set_ylabel("vertical acc (mg)")
    axes[0].set_title("IMU (band-passed, 10-20 s)")
    axes[0].grid(alpha=0.3)

    m2 = (wave_t >= 10 - offset) & (wave_t <= 20 - offset)
    axes[1].plot(wave_t[m2], wave_v[m2], lw=1.0, color="k")
    axes[1].set_ylabel("platform (mm)")
    axes[1].set_xlabel("time (s)")
    axes[1].set_title("platform distance (10-20 s equivalent)")
    axes[1].grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(BASE, "fig4_waveform.png"), dpi=110)
    plt.close(fig)

    say("")
    say("图已保存: fig1_depth_timeline.png, fig2_scatter.png, fig3_error.png, fig4_waveform.png")
    say("逐周期表: cycles_compare.csv")

    with io.open(os.path.join(BASE, "report.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
