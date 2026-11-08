# -*- coding: utf-8 -*-
"""
从原始采集文件解算“位移波形”（每个按压周期内做二次积分）

思路：
    1. 用重力方向把三轴加速度投影成竖直加速度 av；
    2. 带通 0.8~8 Hz，去掉零偏、手的慢移和高频噪声；
    3. 用上升过零把数据切成一个个按压周期；
    4. 每个周期内积分两次，并把速度/位移的线性趋势去掉
       （周期性运动：每个周期结束时状态应回到起点，这样漂移不会累积）；
    5. 拼接起来就是连续的位移波形，峰峰值就是按压深度。

用法：
    python solve_displacement.py [数据文件.txt]
"""

import os
import sys
import glob
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SCALE = {"2g": 0.061, "4g": 0.122, "8g": 0.244, "16g": 0.488}


def find_file(argv):
    if len(argv) > 1:
        return argv[1]
    here = os.path.dirname(os.path.abspath(__file__))
    files = sorted(glob.glob(os.path.join(here, "lsm6dsv_raw_*.txt")))
    if not files:
        raise SystemExit("找不到 lsm6dsv_raw_*.txt")
    return files[-1]


def read_capture(path):
    lsb = None
    t, acc = [], []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            s = line.strip()
            if not s:
                continue
            if s.startswith("#"):
                for k, v in SCALE.items():
                    if "FS_XL=+-" + k in s:
                        lsb = v
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
    if not t:
        raise SystemExit("没有有效数据")
    if lsb is None:
        lsb = SCALE["4g"]
    return np.array(t), np.array(acc, float) * lsb, lsb


def solve_cycles(bp, t, fs):
    """按上升过零切周期，逐周期二次积分并去趋势"""
    dt = 1.0 / fs
    rms = np.sqrt(np.mean(bp ** 2))
    hyst = max(30.0, min(120.0, 0.30 * rms))

    # 找上升过零
    bounds = []
    pos = False
    for i in range(len(bp)):
        if not pos:
            if bp[i] > hyst:
                pos = True
                bounds.append(i)
        else:
            if bp[i] < -hyst:
                pos = False

    disp = np.zeros(len(bp))
    depths = []
    periods = []

    for k in range(len(bounds) - 1):
        i0, i1 = bounds[k], bounds[k + 1]
        n = i1 - i0
        if n < 8:
            continue
        # mg -> mm/s^2（1 mg = 9.80665e-3 m/s^2 = 9.80665 mm/s^2）
        seg = bp[i0:i1] * 9.80665
        per = t[i1] - t[i0]
        if not (0.3 <= per <= 1.2):
            continue

        # 第一次积分：速度
        v = np.cumsum(seg) * dt
        v -= np.linspace(v[0], v[-1], n)

        # 第二次积分：位移
        d = np.cumsum(v) * dt
        d -= np.linspace(d[0], d[-1], n)

        disp[i0:i1] = d
        depths.append(d.max() - d.min())
        periods.append(per)

    return disp, np.array(depths), np.array(periods), bounds


def main():
    path = find_file(sys.argv)
    t, acc, lsb = read_capture(path)

    fs = 1.0 / np.mean(np.diff(t))

    # 重力方向
    amag = np.sqrt((acc ** 2).sum(axis=1))
    quiet = (amag > 920) & (amag < 1080)
    gvec = acc[quiet].mean(axis=0) if quiet.sum() > 50 else np.array([0, 0, 1000.0])
    gn = np.linalg.norm(gvec)
    av = acc.dot(gvec / gn) - gn

    # 带通
    b, a = signal.butter(4, [0.8 / (fs / 2), 8.0 / (fs / 2)], btype="band")
    bp = signal.filtfilt(b, a, av)

    disp, depths, periods, bounds = solve_cycles(bp, t, fs)

    print("=" * 60)
    print("文件:", os.path.basename(path))
    print("时长 %.1f s, 采样率 %.1f Hz, 量程系数 %.3f mg/LSB" % (t[-1] - t[0], fs, lsb))
    print("周期数: %d" % len(depths))
    if len(depths):
        print("周期: 平均 %.3f s -> %.0f 次/分" % (periods.mean(), 60 / periods.mean()))
        print("深度: 平均 %.0f mm, 中位 %.0f mm, 范围 %.0f ~ %.0f mm" %
              (depths.mean(), np.median(depths), depths.min(), depths.max()))
        print("位移波形峰峰值: %.0f mm" % (disp.max() - disp.min()))

    # ---------- 作图 ----------
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    axes[0].plot(t, bp, lw=0.7, color="tab:red")
    axes[0].set_ylabel("vertical acc (mg)")
    axes[0].grid(alpha=0.3)
    axes[0].set_title("displacement solved from " + os.path.basename(path))

    axes[1].plot(t, disp, lw=0.9, color="tab:blue")
    axes[1].axhline(0, color="gray", lw=0.6)
    axes[1].set_ylabel("displacement (mm)")
    axes[1].set_xlabel("time (s)")
    axes[1].grid(alpha=0.3)

    plt.tight_layout()
    out = os.path.splitext(path)[0] + "_displacement.png"
    plt.savefig(out, dpi=110)
    plt.close(fig)
    print("图已保存:", out)

    # 放大 10 秒
    if len(t) > 2000:
        t0 = t[len(t) // 3]
        m = (t >= t0) & (t <= t0 + 8)
        fig2, axes2 = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
        axes2[0].plot(t[m], bp[m], lw=0.9, color="tab:red")
        axes2[0].set_ylabel("acc (mg)")
        axes2[0].grid(alpha=0.3)
        axes2[0].set_title("zoom %.1f~%.1f s" % (t0, t0 + 8))
        axes2[1].plot(t[m], disp[m], lw=1.1, color="tab:blue")
        axes2[1].axhline(0, color="gray", lw=0.6)
        axes2[1].set_ylabel("disp (mm)")
        axes2[1].set_xlabel("time (s)")
        axes2[1].grid(alpha=0.3)
        plt.tight_layout()
        out2 = os.path.splitext(path)[0] + "_displacement_zoom.png"
        plt.savefig(out2, dpi=110)
        plt.close(fig2)
        print("图已保存:", out2)


if __name__ == "__main__":
    main()
