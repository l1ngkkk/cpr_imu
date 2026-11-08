# -*- coding: utf-8 -*-
"""固件滤波器链（因果 IIR：2×2 阶高通 1 Hz + 2 阶低通 8 Hz）与离线零相位滤波的增益差异。

固件 cpr_biquad_init() 的系数就是 RBJ cookbook、Q=0.7071，两级高通串联 = 4 阶巴特沃斯。
它是有相位的因果滤波，在 2 Hz 基波上会有固定衰减，标定常数必须按它来。
"""
import io, os, sys, glob
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pipeline as P
from cpr_fit import (load_sensor, load_platform, find_platform, preprocess,
                     find_positive_peaks, align, onset_guess, BASE, SIMS, G)


def biquad(fs, fc, highpass):
    w0 = 2 * np.pi * fc / fs
    cw = np.cos(w0); sw = np.sin(w0)
    alpha = sw / 1.41421356
    a0 = 1.0 + alpha
    if highpass:
        b = np.array([(1 + cw) * 0.5, -(1 + cw), (1 + cw) * 0.5]) / a0
    else:
        b = np.array([(1 - cw) * 0.5, (1 - cw), (1 - cw) * 0.5]) / a0
    a = np.array([1.0, -2.0 * cw / a0, (1.0 - alpha) / a0])
    return b, a


def firmware_bandpass(x, fs, hp=1.0, lp=8.0):
    from scipy.signal import lfilter
    b, a = biquad(fs, hp, True)
    y = lfilter(b, a, x)
    y = lfilter(b, a, y)
    b, a = biquad(fs, lp, False)
    y = lfilter(b, a, y)
    return y


def gain_at(fs, f, hp=1.0, lp=8.0):
    """因果链在频率 f 处的幅频响应（用冲激响应 FFT 求）"""
    n = 1 << 15
    imp = np.zeros(n); imp[0] = 1.0
    h = firmware_bandpass(imp, fs, hp, lp)
    H = np.abs(np.fft.rfft(h))
    fr = np.fft.rfftfreq(n, 1.0 / fs)
    return np.interp(f, fr, H)


for f in (1.5, 2.0, 2.5, 3.0, 4.0):
    print("固件滤波器链在 %.1f Hz 的幅频增益: %.4f" % (f, gain_at(117.0, f)))
print()


def run(sim, use_fw, hp=1.0, lp=8.0):
    d = os.path.join(BASE, sim)
    imu = sorted(glob.glob(os.path.join(d, "lsm6dsv_raw_*.txt")))[-1]
    t, acc = load_sensor(imu)
    pt, pv, peaks = load_platform(find_platform(sim))
    t_u, bp_off, av_off, fs_u = preprocess(t, acc)
    pcyc, rest = P.platform_cycles2(pt, pv, peaks)

    if use_fw:
        # 直接在原始时间序列上跑固件滤波链（模拟板子上 117 Hz 实时处理）
        amag = np.sqrt((acc ** 2).sum(axis=1))
        q = (amag > 900) & (amag < 1100)
        g = acc[q].mean(axis=0); gn = np.linalg.norm(g); u = g / gn
        av = acc.dot(u) - gn
        fs = 1.0 / np.mean(np.diff(t))
        bp = firmware_bandpass(av, fs, hp, lp)
        tt = t
    else:
        bp = bp_off; fs = fs_u; tt = t_u

    idx, tp = find_positive_peaks(bp, fs)
    off = align(tp, pcyc[:, 0], onset_guess(t_u, bp_off, pt, pv))
    dt = 1.0 / fs
    cyc = []
    for k in range(len(tp) - 1):
        i0 = int(round(tp[k] * fs)); i1 = int(round(tp[k + 1] * fs))
        if i1 - i0 < int(0.30 * fs) or i1 - i0 > int(1.2 * fs):
            continue
        T = tp[k + 1] - tp[k]
        cyc.append((0.5 * (tp[k] + tp[k + 1]), T, P.dft_travel(bp[i0:i1], dt, T)))
    cyc = np.array(cyc)
    rows = []
    for j in range(len(pcyc)):
        k = int(np.argmin(np.abs(cyc[:, 0] - (pcyc[j, 0] + off))))
        if abs(cyc[:, 0][k] - (pcyc[j, 0] + off)) > 0.28:
            continue
        rows.append((pcyc[j, 0], pcyc[j, 2], 0, 0, 0, cyc[k, 2]))
    rows = np.array(rows)
    m = P.valid_mask(np.column_stack([rows[:, 0], rows[:, 1], rows[:, 2], rows[:, 3],
                                      rows[:, 4], rows[:, 5]]))
    rows = rows[m]
    K = rows[:, 1].mean() / rows[:, 5].mean()
    e = K * rows[:, 5] - rows[:, 1]
    x = K * rows[:, 5]
    e5 = P.movavg(x) - P.movavg(rows[:, 1])
    r1 = np.corrcoef(x, rows[:, 1])[0, 1]
    r5 = np.corrcoef(P.movavg(x), P.movavg(rows[:, 1]))[0, 1]
    return (K, np.abs(e).mean(), (np.abs(e) / rows[:, 1]).mean() * 100,
            rows[:, 5].mean(), np.abs(e5).mean(),
            (np.abs(e5) / P.movavg(rows[:, 1])).mean() * 100, r1, r5)


print("%-7s %14s %14s %12s" % ("sim", "K(离线零相位)", "K(固件因果链)", "比值"))
for s in SIMS:
    k1 = run(s, False)
    k2 = run(s, True)
    print("%-7s %14.4f %14.4f %12.4f" % (s, k1[0], k2[0], k2[0] / k1[0]))

for hp in (1.0, 0.8, 0.6):
    print("\n=== 固件 hp_hz = %.1f Hz ===" % hp)
    ks = []
    for s in SIMS:
        r = run(s, True, hp=hp)
        ks.append(r[0])
        print("  %s  K=%.4f  逐次 MAE %.2f mm (%.1f%%)  5次均 MAE %.2f mm (%.1f%%)  r=%.2f/%.2f"
              % (s, r[0], r[1], r[2], r[4], r[5], r[6], r[7]))
    print("  统一 K=%.4f, 离散 %.2f%%" % (np.mean(ks),
                                         (max(ks) - min(ks)) / np.mean(ks) * 100))
