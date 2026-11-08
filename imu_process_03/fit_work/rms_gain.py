# -*- coding: utf-8 -*-
"""为什么 RMS 法加个增益也救不回来（增益三次实验不一致）。"""
import io, os, sys, glob
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cpr_fit import (load_sensor, load_platform, find_platform, preprocess,
                     find_positive_peaks, platform_cycles, align, onset_guess,
                     BASE, SIMS, G)
import pipeline as P

rows_out = []
print("%-7s %8s %8s %8s %8s %8s %8s" % ("sim", "travel", "rms", "K_rms", "fit1", "K_fit", "h_rms"))
for s in SIMS:
    d = os.path.join(BASE, s)
    imu = sorted(glob.glob(os.path.join(d, "lsm6dsv_raw_*.txt")))[-1]
    t, acc = load_sensor(imu)
    pt, pv, peaks = load_platform(find_platform(s))
    t_u, bp, av, fs_u = preprocess(t, acc)
    _, tp = find_positive_peaks(bp, fs_u)
    pcyc, rest = P.platform_cycles2(pt, pv, peaks)
    off = align(tp, pcyc[:, 0], onset_guess(t_u, bp, pt, pv))
    dt = 1.0 / fs_u
    cyc = []
    for k in range(len(tp) - 1):
        i0 = int(round(tp[k] * fs_u)); i1 = int(round(tp[k + 1] * fs_u))
        if i1 - i0 < int(0.30 * fs_u) or i1 - i0 > int(1.2 * fs_u):
            continue
        T = tp[k + 1] - tp[k]; w = 2 * np.pi / T; seg = bp[i0:i1]
        rms = 2.8284271 * np.sqrt(np.mean(seg ** 2)) * G / (w * w)
        cyc.append((0.5 * (tp[k] + tp[k + 1]), rms, P.dft_travel(seg, dt, T)))
    cyc = np.array(cyc)
    rows = []
    for j in range(len(pcyc)):
        k = int(np.argmin(np.abs(cyc[:, 0] - (pcyc[j, 0] + off))))
        if abs(cyc[:, 0][k] - (pcyc[j, 0] + off)) > 0.28:
            continue
        rows.append((pcyc[j, 0], pcyc[j, 2], cyc[k, 1], cyc[k, 2]))
    rows = np.array(rows)
    m = P.valid_mask(np.column_stack([rows[:, 0], rows[:, 1], rows[:, 2],
                                      np.zeros(len(rows)), np.zeros(len(rows)),
                                      rows[:, 3]]))
    rows = rows[m]
    tr = rows[:, 1].mean()
    k_rms = tr / rows[:, 2].mean()
    k_fit = tr / rows[:, 3].mean()
    print("%-7s %8.1f %8.1f %8.4f %8.1f %8.4f %8.2f"
          % (s, tr, rows[:, 2].mean(), k_rms, rows[:, 3].mean(), k_fit,
             rows[:, 2].mean() / rows[:, 3].mean()))
    rows_out.append((s, k_rms, k_fit))
kr = [r[1] for r in rows_out]; kf = [r[2] for r in rows_out]
print("\nK_rms 离散 %.1f%%    K_fit 离散 %.1f%%"
      % ((max(kr) - min(kr)) / np.mean(kr) * 100, (max(kf) - min(kf)) / np.mean(kf) * 100))
