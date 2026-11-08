# -*- coding: utf-8 -*-
"""校验评估帧是否错了一个周期：把周期-峰值对应关系整体平移 s，看误差曲线的最小值在哪。"""
import io, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cpr_fit import SIMS, OUT
import realtime_eval as R
from cpr_fit import G


for sim in SIMS:
    ch = R.Chain(sim)
    pairs, pcyc, tp, off = R.frame(sim, ch)
    print("=" * 70)
    print("%s  对齐 off=%.3f s, 配对 %d" % (sim, off, len(pairs)))
    # 直接用整段位移序列和整段平台序列比，扫描整数错位
    for s in (-2, -1, 0, 1, 2):
        ref, est = [], []
        for j, k in pairs:
            kk = k + s
            if kk < 0 or kk + 1 >= len(tp):
                continue
            ta, tb = tp[kk], tp[kk + 1]
            T = tb - ta
            d = R.dft_depth(ch, ta, tb, T, 1)
            if np.isfinite(d):
                ref.append(pcyc[j, 2]); est.append(d)
        ref = np.array(ref); est = np.array(est)
        K = ref.mean() / est.mean()
        e = K * est - ref
        print("   平移 %+d 周期: MAE %.2f mm, r=%.3f, K=%.3f"
              % (s, np.abs(e).mean(), np.corrcoef(est, ref)[0, 1], K))
