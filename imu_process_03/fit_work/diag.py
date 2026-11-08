# -*- coding: utf-8 -*-
"""Diagnostics: period tracking, gyro rotation, harmonic content (why RMS fails)."""
import io, os, sys, glob
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cpr_fit import (load_sensor, load_platform, find_platform, preprocess,
                     find_positive_peaks, platform_cycles, align, onset_guess,
                     BASE, OUT, SIMS, G)

SKIP_HEAD, SKIP_TAIL = 8, 2

for sim in SIMS:
    d = os.path.join(BASE, sim)
    imu = sorted(glob.glob(os.path.join(d, "lsm6dsv_raw_*.txt")))[-1]
    t, acc = load_sensor(imu)
    # gyro
    gyr = []
    with io.open(imu, encoding="utf-8") as f:
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
            gyr.append(v[4:7])
    gyr = np.array(gyr, float) * 0.0175        # mdps/LSB
    tg = np.array([])
    # 重新对齐 gyro 时间
    tl = []
    with io.open(imu, encoding="utf-8") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            p = s.split(",")
            if len(p) != 8:
                continue
            try:
                tl.append(int(p[0]) / 1000.0)
            except ValueError:
                pass
    tl = np.array(tl)

    pt, pv, peaks = load_platform(find_platform(sim))
    t_u, bp, av, fs_u = preprocess(t, acc)
    pk_idx, tp = find_positive_peaks(bp, fs_u)
    pcyc, rest = platform_cycles(pt, pv, peaks)
    off = align(tp, pcyc[:, 0], onset_guess(t_u, bp, pt, pv))

    print("=" * 70)
    print(sim)
    print("  gyro during compressions: |g| p95 = %.1f dps, max %.1f dps (stillness: %.1f)"
          % (np.percentile(np.abs(gyr).sum(1), 95),
             np.abs(gyr).sum(1).max(),
             np.median(np.abs(gyr).sum(1)[:200])))

    # 周期对比
    T_imu, T_pl = [], []
    for j in range(1, len(pcyc)):
        k = int(np.argmin(np.abs(tp - (pcyc[j, 0] + off))))
        if abs(tp[k] - (pcyc[j, 0] + off)) > 0.28:
            continue
        if k + 1 >= len(tp):
            continue
        T_imu.append(tp[k + 1] - tp[k])
        T_pl.append(pcyc[j, 1])
    T_imu = np.array(T_imu[SKIP_HEAD:len(T_imu) - SKIP_TAIL])
    T_pl = np.array(T_pl[SKIP_HEAD:len(T_pl) - SKIP_TAIL])
    print("  T_imu mean %.4f sd %.4f (%.1f%%) | T_plat mean %.4f sd %.4f (%.1f%%) | r=%.3f"
          % (T_imu.mean(), T_imu.std(), T_imu.std() / T_imu.mean() * 100,
             T_pl.mean(), T_pl.std(), T_pl.std() / T_pl.mean() * 100,
             np.corrcoef(T_imu, T_pl)[0, 1]))

    # 谐波含量：对一个周期做 FFT
    amps = []
    for k in range(SKIP_HEAD + 2, len(tp) - 3):
        i0 = int(round(tp[k] * fs_u)); i1 = int(round(tp[k + 1] * fs_u))
        seg = bp[i0:i1] * np.hanning(i1 - i0)
        n = len(seg)
        X = np.abs(np.fft.rfft(seg)) / (n / 4.0)
        if len(X) < 6:
            continue
        amps.append([X[1], X[2], X[3], X[4]])
    amps = np.array(amps)
    a1 = amps[:, 0].mean()
    print("  基波/2/3/4 次谐波幅度 (mg): %.1f / %.1f / %.1f / %.1f  -> h2=%.2f h3=%.2f h4=%.2f"
          % (a1, amps[:, 1].mean(), amps[:, 2].mean(), amps[:, 3].mean(),
             amps[:, 1].mean() / a1, amps[:, 2].mean() / a1, amps[:, 3].mean() / a1))
    crest = np.sqrt(1 + (amps[:, 1] / amps[:, 0]) ** 2 + (amps[:, 2] / amps[:, 0]) ** 2
                    + (amps[:, 3] / amps[:, 0]) ** 2).mean()
    print("  由谐波算出的 rms 法高估系数 (应为 1.00): %.2f  实测 %.2f"
          % (crest, 1.0))
    np.save(os.path.join(OUT, "harm_%s.npy" % sim), amps)
