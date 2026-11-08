# -*- coding: utf-8 -*-
"""
LSM6DSV 原始数据离线分析

用法：
    python analyze_raw.py [数据文件.txt]

不传文件名时，自动分析本目录下最新的 lsm6dsv_raw_*.txt。

输出：
    - 控制台统计：采样率、时长、静置 |a|、按压次数、频率、深度
    - 图片：<文件名>_overview.png、<文件名>_zoom.png

依赖：numpy、matplotlib（conda base 环境自带）
"""

import os
import sys
import glob
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# 不同量程对应的灵敏度（mg/LSB）
SCALE_MAP = {
    "2g": 0.061,
    "4g": 0.122,
    "8g": 0.244,
    "16g": 0.488,
}


def find_file(argv):
    """确定要分析的文件"""
    if len(argv) > 1:
        return argv[1]

    here = os.path.dirname(os.path.abspath(__file__))
    files = sorted(glob.glob(os.path.join(here, "lsm6dsv_raw_*.txt")))
    if not files:
        raise SystemExit("没有找到 lsm6dsv_raw_*.txt，请把文件名作为参数传入")
    return files[-1]


def read_capture(path):
    """读取采集文件，返回时间(s)、加速度(mg)、陀螺(dps)、量程"""
    acc_lsb = None
    t = []
    acc = []
    gyr = []

    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            s = line.strip()
            if not s:
                continue

            if s.startswith("#"):
                # 从表头里识别量程
                if "FS_XL=+-4g" in s:
                    acc_lsb = SCALE_MAP["4g"]
                elif "FS_XL=+-8g" in s:
                    acc_lsb = SCALE_MAP["8g"]
                elif "FS_XL=+-2g" in s:
                    acc_lsb = SCALE_MAP["2g"]
                elif "FS_XL=+-16g" in s:
                    acc_lsb = SCALE_MAP["16g"]
                continue

            parts = s.split(",")
            if len(parts) != 8:
                continue
            try:
                v = [int(x) for x in parts]
            except ValueError:
                continue

            t.append(v[0] / 1000.0)
            acc.append(v[1:4])
            gyr.append(v[4:7])

    if not t:
        raise SystemExit("文件里没有有效数据行")

    if acc_lsb is None:
        acc_lsb = SCALE_MAP["4g"]
        print("注意：表头里没找到量程，按 ±4 g 处理")

    return (np.array(t), np.array(acc, dtype=float) * acc_lsb,
            np.array(gyr, dtype=float) * 0.0175, acc_lsb)


def bandpass_fft(x, fs, f_lo, f_hi):
    """频域带通，简单可靠"""
    n = len(x)
    X = np.fft.rfft(x)
    f = np.fft.rfftfreq(n, 1.0 / fs)
    X[(f < f_lo) | (f > f_hi)] = 0.0
    return np.fft.irfft(X, n=n)


def detect_cycles(bp, t, fs):
    """
    在带通信号上做“上升过零 + 迟滞”检测，返回每次按压的 (起点, 终点) 索引。
    和在板子上跑的逻辑一致。
    """
    rms = np.sqrt(np.mean(bp ** 2))
    hyst = max(40.0, min(100.0, 0.30 * rms))

    state_pos = False
    t_fall = None
    segments = []

    for i in range(len(bp)):
        if not state_pos:
            if bp[i] > hyst:
                if t_fall is not None:
                    half = t[i] - t_fall
                    if 0.12 <= half <= 0.50:
                        segments.append((t_fall, t[i]))
                state_pos = True
        else:
            if bp[i] < -hyst:
                state_pos = False
                t_fall = t[i]

    return segments, hyst


def analyze(path):
    t, acc, gyr, acc_lsb = read_capture(path)

    dt = np.diff(t)
    fs = 1.0 / np.mean(dt)

    print("=" * 62)
    print("文件:", os.path.basename(path))
    print("量程: 灵敏度 %.3f mg/LSB" % acc_lsb)
    print("样本: %d, 时长 %.1f s, 平均采样率 %.1f Hz" %
          (len(t), t[-1] - t[0], fs))
    print("采样间隔: 中位 %.1f ms, 最大 %.1f ms" %
          (np.median(dt) * 1000, dt.max() * 1000))

    # 合加速度与静置判断
    amag = np.sqrt((acc ** 2).sum(axis=1))
    print("合加速度 |a|: 中位 %.0f mg, 5%%~95%% 分位 %.0f~%.0f" %
          (np.median(amag), np.percentile(amag, 5), np.percentile(amag, 95)))

    quiet = (amag > 920) & (amag < 1080)
    if quiet.sum() > 50:
        gvec = acc[quiet].mean(axis=0)
    else:
        gvec = np.array([0.0, 0.0, 1000.0])
    gn = np.linalg.norm(gvec)
    u = gvec / gn
    print("重力方向 (mg): [%.0f, %.0f, %.0f], |g| = %.0f" %
          (gvec[0], gvec[1], gvec[2], gn))

    av = acc.dot(u) - gn
    bp = bandpass_fft(av, fs, 1.0, 8.0)

    segments, hyst = detect_cycles(bp, t, fs)
    print("迟滞阈值: %.0f mg" % hyst)
    print("检测到按压: %d 次" % len(segments))

    if len(segments) > 1:
        periods = np.array([b - a for a, b in segments]) * 2.0   # 半周 -> 周期
        keep = (periods > 0.3) & (periods < 1.2)
        periods = periods[keep]
        seg_keep = [s for s, k in zip(segments, keep) if k]

        if len(periods):
            print("平均周期: %.3f s  ->  %.0f 次/分" %
                  (periods.mean(), 60.0 / periods.mean()))
            print("周期范围: %.2f ~ %.2f s (%.0f~%.0f 次/分)" %
                  (periods.min(), periods.max(), 60.0 / periods.max(), 60.0 / periods.min()))

            depths = []
            for (a, b), per in zip(seg_keep, periods):
                mask = (t >= a) & (t <= b)
                if mask.sum() < 5:
                    continue
                rms = np.sqrt(np.mean(bp[mask] ** 2))
                w = 2 * np.pi / per
                depths.append(2.8284271 * rms * 9.80665 / (w * w))
            depths = np.array(depths)
            if len(depths):
                print("深度: 平均 %.0f mm, 中位 %.0f mm, 范围 %.0f~%.0f mm" %
                      (depths.mean(), np.median(depths), depths.min(), depths.max()))

    # ---------------- 画图 ----------------
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

    axes[0].plot(t, acc[:, 0], lw=0.7, label="ax")
    axes[0].plot(t, acc[:, 1], lw=0.7, label="ay")
    axes[0].plot(t, acc[:, 2], lw=0.7, label="az")
    axes[0].set_ylabel("acc (mg)")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[0].grid(alpha=0.3)
    axes[0].set_title(os.path.basename(path))

    axes[1].plot(t, av, lw=0.7, color="tab:blue", label="av")
    axes[1].plot(t, bp, lw=0.7, color="tab:red", alpha=0.7, label="1-8 Hz")
    for a, b in segments:
        axes[1].axvspan(a, b, color="orange", alpha=0.15)
    axes[1].set_ylabel("vertical (mg)")
    axes[1].legend(loc="upper right", fontsize=8)
    axes[1].grid(alpha=0.3)

    axes[2].plot(t, gyr[:, 0], lw=0.6, label="gx")
    axes[2].plot(t, gyr[:, 1], lw=0.6, label="gy")
    axes[2].plot(t, gyr[:, 2], lw=0.6, label="gz")
    axes[2].set_ylabel("gyro (dps)")
    axes[2].set_xlabel("time (s)")
    axes[2].legend(loc="upper right", fontsize=8)
    axes[2].grid(alpha=0.3)

    plt.tight_layout()
    base = os.path.splitext(path)[0]
    out1 = base + "_overview.png"
    plt.savefig(out1, dpi=110)
    plt.close(fig)
    print("图已保存:", out1)

    # 放大 10 秒
    if len(t) > 1200:
        t0 = t[len(t) // 2]
        m = (t > t0) & (t < t0 + 10)
        fig2, ax = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
        ax[0].plot(t[m], av[m], lw=0.8)
        ax[0].plot(t[m], bp[m], lw=0.8, alpha=0.7)
        ax[0].set_ylabel("vertical (mg)")
        ax[0].grid(alpha=0.3)
        ax[0].set_title("zoom %.0f~%.0f s" % (t0, t0 + 10))
        for a, b in segments:
            if t0 < a < t0 + 10:
                ax[1].axvspan(a, b, color="orange", alpha=0.2)
        ax[1].plot(t[m], bp[m], lw=0.9, color="tab:red")
        ax[1].set_ylabel("band-pass (mg)")
        ax[1].set_xlabel("time (s)")
        ax[1].grid(alpha=0.3)
        plt.tight_layout()
        out2 = base + "_zoom.png"
        plt.savefig(out2, dpi=110)
        plt.close(fig2)
        print("图已保存:", out2)


if __name__ == "__main__":
    analyze(find_file(sys.argv))
