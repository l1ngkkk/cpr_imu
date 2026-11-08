# -*- coding: utf-8 -*-
"""
残差消除实验：从静止开始，尝试几种积分/修正算法，看能否还原“绝对深度”

用法：
    python residual_test.py <sim目录>

算法：
    A. raw      : 连续二次积分 + 0.5 Hz 高通（无修正）
    B. zupt     : 在加速度极值点做零速修正（v=0），分段积分
    C. zupt_b   : B + 用开始时静止段估计零偏并扣除
    D. zupt_bz  : C + 每段结束时把速度精确归零（双端约束）

评价：
    绝对深度 = |位移最低点|（从静止零点算起）
    与平台“绝对深度（谷底-静置）”对比
"""

import os
import sys
import io
import csv
import glob
import numpy as np
from scipy import signal

ACC_LSB = 0.122


# ----------------------------------------------------------------------
def find_inputs(base):
    imu = sorted(glob.glob(os.path.join(base, "lsm6dsv_raw_*.txt")))
    if not imu:
        raise SystemExit("找不到 IMU 数据")
    plat = os.path.join(base, "platform_raw.csv")
    if not os.path.exists(plat):
        cands = [c for c in glob.glob(os.path.join(base, "*", "*.csv"))
                 if "校验" not in os.path.basename(c)]
        if not cands:
            raise SystemExit("找不到平台数据")
        plat = cands[0]
    return imu[-1], plat


def load_sensor(path):
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
    return np.array(t), np.array(acc, float) * ACC_LSB


def load_platform(path):
    wave_t, wave_v, peak_t = [], [], []
    with io.open(path, encoding="utf-8-sig", errors="replace", newline="") as f:
        for row in csv.DictReader(f):
            if row["source"] == "mannequin" and row["mannequin_raw"]:
                try:
                    wave_t.append(float(row["elapsed_s"]))
                    wave_v.append(float(row["mannequin_raw"]))
                except ValueError:
                    pass
            elif row["source"] == "mannequin_peak":
                try:
                    peak_t.append(float(row["elapsed_s"]))
                except ValueError:
                    pass
    o = np.argsort(wave_t)
    return np.array(wave_t)[o], np.array(wave_v)[o], np.array(sorted(peak_t))


def platform_truth(wave_t, wave_v, peak_t):
    rest = np.median(wave_v[wave_t < 1.0])
    absd, tops, bots = [], [], []
    for tp in peak_t:
        m = (wave_t >= tp - 0.5) & (wave_t <= tp + 0.12)
        if m.sum() < 5:
            absd.append(np.nan); tops.append(np.nan); bots.append(np.nan)
            continue
        seg = wave_v[m]
        bots.append(seg.max())
        tops.append(seg.min())
        absd.append(seg.max() - rest)
    return np.array(absd), np.array(tops), np.array(bots), rest


# ----------------------------------------------------------------------
def integrate_zupt(av, t, fs, two_sided=True):
    """在加速度极值点之间分段积分，每个极值点强制 v=0"""
    dt = 1.0 / fs
    # 极值点：加速度的局部最大/最小
    pk, _ = signal.find_peaks(av, distance=int(0.10 * fs), prominence=40)
    tr, _ = signal.find_peaks(-av, distance=int(0.10 * fs), prominence=40)
    idx = np.unique(np.concatenate([pk, tr]))
    idx = idx[(idx > 2) & (idx < len(av) - 3)]

    d = np.zeros(len(av))
    v_prev = 0.0
    d_prev = 0.0
    i_prev = 0

    for i in idx:
        seg = av[i_prev:i + 1]
        n = len(seg)
        if n < 3:
            i_prev = i
            continue
        v = np.cumsum(seg) * dt * 9.80665
        if two_sided:
            v -= np.linspace(v[0], v[-1], n)   # 两端速度归零
            dseg = np.cumsum(v) * dt
            d[i_prev:i + 1] = d_prev + dseg
            d_prev = d[i]
            v_prev = 0.0
        else:
            v -= v[0]                          # 只把起点归零
            dseg = np.cumsum(v) * dt
            d[i_prev:i + 1] = d_prev + dseg
            d_prev = d[i]
            v_prev = v[-1]
        i_prev = i

    return d, idx


def integrate_raw(av, fs):
    dt = 1.0 / fs
    b, a = signal.butter(2, 0.5 / (fs / 2), btype="high")
    avf = signal.filtfilt(b, a, av)
    v = np.cumsum(avf) * dt * 9.80665
    v = signal.filtfilt(b, a, v)
    d = np.cumsum(v) * dt
    d = signal.filtfilt(b, a, d)
    return d


# ----------------------------------------------------------------------
def main():
    base = sys.argv[1] if len(sys.argv) > 1 else "."
    base = os.path.abspath(base)
    imu, plat = find_inputs(base)

    t, acc = load_sensor(imu)
    wave_t, wave_v, peak_t = load_platform(plat)
    abs_true, tops, bots, rest = platform_truth(wave_t, wave_v, peak_t)

    fs = 1.0 / np.mean(np.diff(t))
    dt = 1.0 / fs

    amag = np.sqrt((acc ** 2).sum(axis=1))
    q = (amag > 920) & (amag < 1080)
    g = acc[q].mean(axis=0)
    g /= np.linalg.norm(g)
    av = acc.dot(g) - np.linalg.norm(acc[q].mean(axis=0))

    # 静止段零偏（前 1 秒）
    rest_mask = t < 1.0
    bias = av[rest_mask].mean() if rest_mask.sum() > 10 else 0.0
    av_b = av - bias

    # 对齐
    bp, a2 = signal.butter(4, [1.0 / (fs / 2), 8.0 / (fs / 2)], btype="band")
    bpf = signal.filtfilt(bp, a2, av)
    rms = np.sqrt(np.mean(bpf ** 2))
    hyst = max(40.0, min(100.0, 0.30 * rms))
    fall, pos = [], True
    for i in range(len(bpf)):
        if pos:
            if bpf[i] < -hyst:
                pos = False
                fall.append(i)
        else:
            if bpf[i] > hyst:
                pos = True

    fall = np.array(fall)
    best = (1e9, 0.0)
    for off in np.arange(-6.0, 12.0, 0.01):
        d = np.abs((t[fall] + off)[None, :] - peak_t[:, None])
        dist = d.min(axis=1).mean()
        if dist < best[0]:
            best = (dist, off)
    offset = best[1]

    print("=" * 70)
    print("目录:", os.path.basename(base))
    print("静止零偏估计: %+.2f mg" % bias)
    print("对齐偏移: %.2f s" % offset)
    print("")

    results = {}

    # A. 纯积分
    d_raw = integrate_raw(av_b, fs)
    results["A raw"] = d_raw

    # B. ZUPT 单端
    d_b, _ = integrate_zupt(av, t, fs, two_sided=False)
    results["B zupt"] = d_b

    # C. ZUPT + 零偏
    d_c, _ = integrate_zupt(av_b, t, fs, two_sided=False)
    results["C zupt+bias"] = d_c

    # D. ZUPT + 零偏 + 双端约束
    d_d, _ = integrate_zupt(av_b, t, fs, two_sided=True)
    results["D zupt+bias+2s"] = d_d

    print("%-20s %10s %10s %10s" % ("算法", "中位深度", "偏差", "MAPE"))
    for name, d in results.items():
        vals = []
        for tp, gt in zip(peak_t, abs_true):
            if not np.isfinite(gt):
                continue
            idx = int(np.argmin(np.abs(t - (tp + offset))))
            lo = max(0, idx - int(0.30 * fs))
            hi = min(len(d), idx + int(0.30 * fs))
            vals.append(-d[lo:hi].min())
        vals = np.array(vals)
        ok = np.isfinite(gt) & (abs_true > 25)
        vv = vals[ok]
        gg = abs_true[ok]
        mape = np.abs((vv - gg) / gg * 100).mean()
        print("%-20s %10.1f %+10.1f %9.1f%%" %
              (name, np.median(vv), (vv - gg).mean(), mape))

    # 保存位移曲线
    np.savetxt(os.path.join(base, "residual_displacement.csv"),
               np.column_stack([t, d_raw, d_b, d_c, d_d]),
               delimiter=",", header="t,raw,zupt,zupt_bias,zupt_bias_2s",
               comments="")

    with io.open(os.path.join(base, "residual_report.txt"), "w",
                 encoding="utf-8") as f:
        f.write("目录 %s\n" % os.path.basename(base))
        f.write("静止零偏 %+.2f mg, 对齐 %.2f s\n" % (bias, offset))
        f.write("平台绝对深度均值 %.1f mm\n" % np.nanmean(abs_true))


if __name__ == "__main__":
    main()
