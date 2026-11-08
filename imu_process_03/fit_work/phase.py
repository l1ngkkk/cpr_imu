# -*- coding: utf-8 -*-
"""相位关系与报数延迟：把 bp、漏积分位移、平台波形画在一起看清楚。"""
import io, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cpr_fit import SIMS, OUT, G
import realtime_eval as R

SH = {"sim01": 0, "sim02": -1, "sim03": 0}

for sim in SIMS:
    ch = R.Chain(sim)
    pairs, pcyc, tp, off = R.frame(sim, ch)
    sh = SH[sim]
    # 取中间一段
    j0 = len(pairs) // 2
    ks = [pairs[i][1] - sh for i in range(j0 - 2, j0 + 3)]
    ks = [k for k in ks if 0 <= k < len(tp) - 1]
    t_a = tp[ks[0]]; t_b = tp[ks[-1] + 1]
    m = (ch.t >= t_a) & (ch.t <= t_b)
    tt = ch.t[m]

    fig, axes = plt.subplots(3, 1, figsize=(12, 8.4), sharex=True)
    axes[0].plot(tt, ch.bp[m], lw=1.0, color="tab:red")
    for k in ks:
        axes[0].axvline(tp[k], color="0.6", ls=":")
    axes[0].set_ylabel("带通加速度 (mg)")
    axes[0].set_title("%s  相位关系（灰虚线=带通正峰，也就是周期分界）" % sim)
    axes[0].grid(alpha=0.3)

    axes[1].plot(tt, ch.disp[m], lw=1.4, color="tab:blue")
    for k in ks:
        i0 = int(np.ceil(tp[k] * ch.fs)); i1 = int(np.floor(tp[k + 1] * ch.fs))
        imin = i0 + int(np.argmin(ch.disp[i0:i1]))
        axes[1].axvline(ch.t[imin], color="tab:green", ls="--", lw=1.0)
    axes[1].set_ylabel("漏积分位移 (mm)")
    axes[1].grid(alpha=0.3)
    axes[1].set_title("蓝=位移，绿虚线=谷底（=最大压缩时刻）")

    # 平台波形，按帧的偏移平移
    pt = np.concatenate([[p[0] for p in []]]) if False else None
    from cpr_fit import load_platform, find_platform
    pt2, pv2, pk2 = load_platform(find_platform(sim))
    mm = (pt2 >= t_a - off + 0.3) & (pt2 <= t_b - off + 0.3)
    axes[2].plot(pt2[mm] + off, pv2[mm], "k.-", ms=4)
    axes[2].set_ylabel("平台读数 (mm)")
    axes[2].set_xlabel("时间 (s)")
    axes[2].grid(alpha=0.3)
    axes[2].set_title("平台 mannequin_raw（已按帧的偏移对齐到 IMU 时间轴）")
    plt.tight_layout()
    plt.savefig(os.path.join(OUT, "phase_%s.png" % sim), dpi=105)
    plt.close(fig)

    # 统计：谷底到周期结束的延迟
    lags = []
    for j, k in pairs:
        i0 = int(np.ceil(tp[k] * ch.fs)); i1 = int(np.floor(tp[k + 1] * ch.fs))
        if i1 - i0 < 10:
            continue
        imin = i0 + int(np.argmin(ch.disp[i0:i1]))
        lags.append(tp[k + 1] - ch.t[imin])
    lags = np.array(lags)
    print("%s  谷底 -> 周期结束(tp[k+1]) 的中位延迟 %.0f ms   (=%.2f 个周期)"
          % (sim, np.median(lags) * 1000,
             np.median(lags) / np.median([tp[k + 1] - tp[k] for k in range(len(tp) - 1)])))
