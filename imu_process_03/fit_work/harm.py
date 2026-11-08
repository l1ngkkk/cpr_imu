# -*- coding: utf-8 -*-
"""谐波含量 + RMS 高估系数（与 fig4 柱状图同一算法）。"""
import io, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pipeline as P
from cpr_fit import SIMS, G

for s in SIMS:
    r = P.process(s)
    tp, bp, fs = r["tp"], r["bp"], r["fs_u"]
    amps = []
    for k in range(6, len(tp) - 6):
        i0 = int(round(tp[k] * fs)); i1 = int(round(tp[k + 1] * fs))
        T = tp[k + 1] - tp[k]; n = i1 - i0
        t = np.arange(n) / fs; w = 2 * np.pi / T
        A = []
        for h in range(1, 6):
            S = np.dot(bp[i0:i1], np.sin(h * w * t))
            C = np.dot(bp[i0:i1], np.cos(h * w * t))
            A.append(2.0 / n * np.hypot(S, C))
        amps.append(A)
    amps = np.array(amps); m = amps.mean(axis=0)
    ratio = m / m[0]
    crest = np.sqrt((ratio ** 2).sum())
    print("%s  基波 %.0f mg   h2=%.2f h3=%.2f h4=%.2f h5=%.2f   RMS 高估系数 %.3f"
          % (s, m[0], ratio[1], ratio[2], ratio[3], ratio[4], crest))
