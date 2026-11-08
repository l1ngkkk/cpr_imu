# -*- coding: utf-8 -*-
"""
延迟预算实验（budget）
======================
允许"晚 1~2 个周期"之后，可以用的手段有：
  * 跨 N 次按压的滑动窗（N = 1,2,3,4,5）
  * 跨 N 次的滑动平均
  * 在一个 N 周期窗口里**联合估计频率和幅值**（把 d ∝ T² 的敏感度去掉）
  * 块内零相位滤波（延迟 2 个周期即可实现）

对每个 N 报：逐次 MAE / MAPE / r，以及"相对压到底的延迟"= (N-1)/2 个周期。

所有方法都在同一评估帧上（沿用 realtime_eval 的周期↔峰值对应）。
"""
import io, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]

matplotlib.rcParams["axes.unicode_minus"] = False

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cpr_fit import SIMS, OUT, G
import pipeline as P
import realtime_eval as R

SH = {"sim01": 0, "sim02": -1, "sim03": 0}


def load(sim):
    ch = R.Chain(sim, f_leak=0.5)
    n = len(ch.bp)
    X = np.fft.rfft(ch.av)
    fr = np.fft.rfftfreq(n, 1.0 / ch.fs)
    H = np.zeros_like(fr)
    band = (fr >= 0.5) & (fr <= 12.0)
    H[band] = -1.0 / ((2 * np.pi * fr[band]) ** 2)
    ch.disp_zp = np.fft.irfft(X * H, n=n) * G
    pairs, pcyc, tp, off = R.frame(sim, ch)
    # 注意：R.frame() 内部已经应用了整数周期修正，这里不要再加一次
    ref = np.array([pcyc[j, 2] for j, k in pairs])
    ks = np.array([k for j, k in pairs])
    return ch, tp, pairs, ref, ks


def dftN(ch, tp, k, N):
    """以第 k 个周期为中心、跨度 N 个周期的窗口，周期取窗口平均，返回峰峰位移"""
    a = k - (N - 1) // 2
    b = a + N
    if a < 0 or b >= len(tp):
        return np.nan
    ta, tb = tp[a], tp[b]
    T = (tb - ta) / N
    fs = ch.fs
    i0 = int(np.ceil(ta * fs)); i1 = int(np.floor(tb * fs))
    seg = ch.bp[i0:i1]
    if len(seg) < 20:
        return np.nan
    w = 2 * np.pi / T
    t = ch.t[i0:i1] - (ta + tb) / 2
    n = len(seg)
    A = 2.0 / n * np.hypot(np.dot(seg, np.sin(w * t)), np.dot(seg, np.cos(w * t)))
    return 2.0 * A * G / (w * w)


def dftN_joint(ch, tp, k, N):
    """N 周期窗口里联合搜索 omega（把周期误差吸收掉）"""
    a = k - (N - 1) // 2
    b = a + N
    if a < 0 or b >= len(tp):
        return np.nan
    ta, tb = tp[a], tp[b]
    T0 = (tb - ta) / N
    fs = ch.fs
    i0 = int(np.ceil(ta * fs)); i1 = int(np.floor(tb * fs))
    seg = ch.bp[i0:i1]
    if len(seg) < 20:
        return np.nan
    t = ch.t[i0:i1] - (ta + tb) / 2.0
    best = (1e18, T0)
    for T in np.arange(T0 * 0.94, T0 * 1.06, 0.0004):
        w = 2 * np.pi / T
        M = np.column_stack([np.sin(w * t), np.cos(w * t)])
        c, *_ = np.linalg.lstsq(M, seg, rcond=None)
        r = seg - M.dot(c)
        if r.dot(r) < best[0]:
            best = (r.dot(r), T)
    T = best[1]
    w = 2 * np.pi / T
    M = np.column_stack([np.sin(w * t), np.cos(w * t)])
    c, *_ = np.linalg.lstsq(M, seg, rcond=None)
    A = np.hypot(c[0], c[1])
    return 2.0 * A * G / (w * w)


def leaky_cycle(ch, tp, k):
    a, b = tp[k], tp[k + 1]
    i0 = int(np.ceil(a * ch.fs)); i1 = int(np.floor(b * ch.fs))
    seg = ch.disp[i0:i1]
    return seg.max() - seg.min() if len(seg) > 5 else np.nan


def movavg_fixed(x, n):
    """滑动平均，但只用"过去和当前"（因果），前面不足就填 nan"""
    out = np.full(len(x), np.nan)
    for i in range(len(x)):
        lo = max(0, i - n + 1)
        seg = x[lo:i + 1]
        seg = seg[np.isfinite(seg)]
        if len(seg) >= max(1, (i - lo + 1) // 2):
            out[i] = seg.mean()
    return out


def score(est, ref):
    ok = np.isfinite(est) & np.isfinite(ref)
    e = est[ok]; r = ref[ok]
    K = r.mean() / e.mean()
    d = K * e - r
    return K, np.abs(d).mean(), (np.abs(d) / r).mean() * 100, np.corrcoef(e, r)[0, 1]


def intzero_cycle(ch, tp, k, f_lo=0.5, f_hi=12.0):
    """零相位频域二次积分（等价于"块内零相位"，延迟 2 个周期即可实现）"""
    if k + 1 >= len(tp):
        return np.nan
    a, b = tp[k], tp[k + 1]
    i0 = int(np.ceil(a * ch.fs)); i1 = int(np.floor(b * ch.fs))
    if i1 - i0 < 5:
        return np.nan
    d = ch.disp_zp
    return d[i0:i1].max() - d[i0:i1].min()


def main():
    lines = []
    def say(s=""):
        print(s); lines.append(s)

    D = {s: load(s) for s in SIMS}

    say("=" * 98)
    say("A. 跨 N 次按压的滑动窗（单频正交，周期取窗口平均）")
    say("   %-4s %-14s %-8s %-8s %-8s %8s %8s %6s" %
        ("N", "延迟", "sim01", "sim02", "sim03", "平均MAE", "MAPE", "r"))
    best = {}
    for N in (1, 2, 3, 4, 5):
        res = []
        for s in SIMS:
            ch, tp, pairs, ref, ks = D[s]
            est = np.array([dftN(ch, tp, k, N) for k in ks])
            res.append(score(est, ref))
        mae = np.mean([r[1] for r in res])
        mape = np.mean([r[2] for r in res])
        rr = np.mean([r[3] for r in res])
        say("   %-4d %-14s %-8.2f %-8.2f %-8.2f %8.2f %7.1f%% %6.3f"
            % (N, "%.1f 个周期" % ((N - 1) / 2.0), res[0][1], res[1][1], res[2][1],
               mae, mape, rr))
        best[N] = res

    say("")
    say("=" * 98)
    say("B. 同上，但窗口内**联合搜索频率**（消掉 d ∝ T² 的敏感度）")
    say("   %-4s %-14s %-8s %-8s %-8s %8s %8s %6s" %
        ("N", "延迟", "sim01", "sim02", "sim03", "平均MAE", "MAPE", "r"))
    for N in (1, 2, 3, 4):
        res = []
        for s in SIMS:
            ch, tp, pairs, ref, ks = D[s]
            est = np.array([dftN_joint(ch, tp, k, N) for k in ks])
            res.append(score(est, ref))
        mae = np.mean([r[1] for r in res])
        say("   %-4d %-14s %-8.2f %-8.2f %-8.2f %8.2f %7.1f%% %6.3f"
            % (N, "%.1f 个周期" % ((N - 1) / 2.0), res[0][1], res[1][1], res[2][1],
               mae, np.mean([r[2] for r in res]), np.mean([r[3] for r in res])))

    say("")
    say("=" * 98)
    say("C. 漏积分（逐次）再做 N 次因果滑动平均")
    say("   %-4s %-14s %-8s %-8s %-8s %8s %8s %6s" %
        ("N", "延迟", "sim01", "sim02", "sim03", "平均MAE", "MAPE", "r"))
    for N in (1, 2, 3, 4, 5):
        res = []
        for s in SIMS:
            ch, tp, pairs, ref, ks = D[s]
            raw = np.array([leaky_cycle(ch, tp, k) for k in ks])
            est = movavg_fixed(raw, N)
            res.append(score(est, ref))
        say("   %-4d %-14s %-8.2f %-8.2f %-8.2f %8.2f %7.1f%% %6.3f"
            % (N, "%.1f 个周期" % ((N - 1) / 2.0), res[0][1], res[1][1], res[2][1],
               np.mean([r[1] for r in res]), np.mean([r[2] for r in res]),
               np.mean([r[3] for r in res])))

    say("")
    say("=" * 98)
    say("D. 对比：逐次 dft1 再做 N 次因果滑动平均（最省事的改法）")
    say("   %-4s %-14s %-8s %-8s %-8s %8s %8s %6s" %
        ("N", "延迟", "sim01", "sim02", "sim03", "平均MAE", "MAPE", "r"))
    for N in (1, 2, 3, 4, 5):
        res = []
        for s in SIMS:
            ch, tp, pairs, ref, ks = D[s]
            raw = np.array([dftN(ch, tp, k, 1) for k in ks])
            est = movavg_fixed(raw, N)
            res.append(score(est, ref))
        say("   %-4d %-14s %-8.2f %-8.2f %-8.2f %8.2f %7.1f%% %6.3f"
            % (N, "%.1f 个周期" % ((N - 1) / 2.0), res[0][1], res[1][1], res[2][1],
               np.mean([r[1] for r in res]), np.mean([r[2] for r in res]),
               np.mean([r[3] for r in res])))

    say("")
    say("=" * 98)
    say("E. 零相位二次积分（块内零相位，延迟 1~2 个周期可实现）+ N 次平均")
    say("   %-4s %-14s %-8s %-8s %-8s %8s %8s %6s" %
        ("N", "延迟", "sim01", "sim02", "sim03", "平均MAE", "MAPE", "r"))
    for N in (1, 2, 3, 4):
        res = []
        for s in SIMS:
            ch, tp, pairs, ref, ks = D[s]
            raw = np.array([intzero_cycle(ch, tp, k) for k in ks])
            est = movavg_fixed(raw, N)
            res.append(score(est, ref))
        say("   %-4d %-14s %-8.2f %-8.2f %-8.2f %8.2f %7.1f%% %6.3f"
            % (N, "%.1f 个周期" % ((N - 1) / 2.0), res[0][1], res[1][1], res[2][1],
               np.mean([r[1] for r in res]), np.mean([r[2] for r in res]),
               np.mean([r[3] for r in res])))

    with io.open(os.path.join(OUT, "budget.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
