# -*- coding: utf-8 -*-
"""漏积分的时间常数敏感性：f_leak 越大，记忆越短、是否还够准？"""
import io, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cpr_fit import SIMS, OUT, G
import realtime_eval as R
import pipeline as P

print("%-8s %-6s %8s %8s %7s %10s" % ("f_leak", "sim", "MAE/mm", "MAPE", "r", "tau/ms"))
lines = []
for fl in (0.2, 0.3, 0.5, 0.8, 1.2, 2.0):
    maes = []
    for sim in SIMS:
        ch = R.Chain(sim, f_leak=fl)
        SH = {"sim01": 0, "sim02": -1, "sim03": 0}[sim]
        pairs, pcyc, tp, off = R.frame(sim, ch)
        ref, est = [], []
        for j, k in pairs:
            ta, tb = tp[k], tp[k + 1]
            i0 = int(np.ceil(ta * ch.fs)); i1 = int(np.floor(tb * ch.fs))
            seg = ch.disp[i0:i1]
            if len(seg) < 6:
                continue
            ref.append(pcyc[j, 2]); est.append(seg.max() - seg.min())
        ref = np.array(ref); est = np.array(est)
        K = ref.mean() / est.mean()
        e = K * est - ref
        maes.append(np.abs(e).mean())
        print("%-8.1f %-6s %8.2f %7.1f%% %7.3f %10.0f"
              % (fl, sim, np.abs(e).mean(), (np.abs(e) / ref).mean() * 100,
                 np.corrcoef(est, ref)[0, 1], 1000.0 / (2 * np.pi * fl)))
        lines.append("f=%.1f %s MAE %.2f MAPE %.1f%% r %.3f" %
                     (fl, sim, np.abs(e).mean(), (np.abs(e) / ref).mean() * 100,
                      np.corrcoef(est, ref)[0, 1]))
    print("         平均 MAE %.2f mm\n" % np.mean(maes))

with io.open(os.path.join(OUT, "leaky_sweep.txt"), "w", encoding="utf-8") as f:
    f.write("\n".join(lines) + "\n")
