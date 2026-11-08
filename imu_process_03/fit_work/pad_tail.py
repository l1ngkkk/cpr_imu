# -*- coding: utf-8 -*-
"""Is the pad's ~9-12 mm 'trough above rest' a load offset (hands on chest) or permanent set?
Look at the pad signal before / during / after the compression bout."""
import io, os, csv, glob, sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cpr_fit import load_platform, find_platform, BASE, SIMS

for sim in SIMS:
    pt, pv, peaks = load_platform(find_platform(sim))
    print("=" * 72)
    print(sim, "pad span %.2f .. %.2f s, %d compressions %.2f..%.2f s"
          % (pt[0], pt[-1], len(peaks), peaks[0, 0], peaks[-1, 0]))
    # 每 1 s 的中位/最小/最大值
    print("  %6s %7s %7s %7s" % ("t(s)", "min", "med", "max"))
    tt = np.arange(pt[0], pt[-1], 1.0)
    for t0 in tt:
        m = (pt >= t0) & (pt < t0 + 1.0)
        if m.sum() < 3:
            continue
        print("  %6.1f %7.1f %7.1f %7.1f" % (t0, pv[m].min(), np.median(pv[m]), pv[m].max()))
