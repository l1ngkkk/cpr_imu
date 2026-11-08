# -*- coding: utf-8 -*-
"""Per-cycle trough/peak of the platform pad wave: is the 'rebound residual' constant or drifting?"""
import io, os, csv, glob
import numpy as np

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIMS = ["sim01", "sim02", "sim03"]


def find_platform(sim):
    d = os.path.join(BASE, sim)
    p = os.path.join(d, "platform_raw.csv")
    if os.path.exists(p):
        return p
    return [c for c in glob.glob(os.path.join(d, "*", "*.csv"))
            if "校验" not in os.path.basename(c)][0]


for sim in SIMS:
    rows = list(csv.DictReader(io.open(find_platform(sim), encoding="utf-8-sig",
                                        errors="replace")))
    pt, pv = [], []
    for r in rows:
        if r["source"] == "mannequin" and r["mannequin_raw"]:
            pt.append(float(r["elapsed_s"])); pv.append(float(r["mannequin_raw"]))
    o = np.argsort(pt); pt = np.array(pt)[o]; pv = np.array(pv)[o]
    pk = [(float(r["elapsed_s"]), float(r["mannequin_depth"]))
          for r in rows if r["source"] == "mannequin_peak" and r["mannequin_depth"]]
    pk.sort()

    print("=" * 74)
    print(sim, " pad samples %d" % len(pv))
    print(" t      report  sam_max  sam_min  travel  trough-rest")
    rest = np.median(pv[pt < 1.5]) if (pt < 1.5).sum() > 3 else pv.min()
    print(" rest baseline (t<1.5s) = %.1f mm" % rest)
    tr, mn, rep = [], [], []
    for i, (tp, d) in enumerate(pk):
        lo = tp - 0.55
        if i > 0:
            lo = max(lo, pk[i - 1][1] * 0 + pk[i - 1][0])
        m = (pt >= lo) & (pt <= tp + 0.15)
        if m.sum() < 4:
            continue
        mx = pv[m].max(); mi = pv[m].min()
        tr.append(mx - mi); mn.append(mi); rep.append(d * 10)
        if i % 8 == 0 or i == len(pk) - 1:
            print(" %6.2f  %5.1f  %6.1f  %6.1f  %6.1f  %+7.1f" %
                  (tp, d * 10, mx, mi, mx - mi, mi - rest))
    tr = np.array(tr); mn = np.array(mn); rep = np.array(rep)
    half = len(tr) // 2
    print(" travel  mean %.1f | first-half %.1f second-half %.1f" %
          (tr.mean(), tr[:half].mean(), tr[half:].mean()))
    print(" trough  mean %.1f | first-half %.1f second-half %.1f" %
          (mn.mean(), mn[:half].mean(), mn[half:].mean()))
    print(" report  mean %.1f | first-half %.1f second-half %.1f" %
          (rep.mean(), rep[:half].mean(), rep[half:].mean()))
    print(" report == sample max ?  max|diff| = %.1f mm" % np.abs(rep - (mn + tr)).max())
