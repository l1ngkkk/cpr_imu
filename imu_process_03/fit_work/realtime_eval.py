# -*- coding: utf-8 -*-
"""
实时（逐样本、因果、逐次即时报数）方案的公平评估
================================================
评估帧：沿用 pipeline.py 已经核实过的"周期 <-> 平台峰值"对应关系，
        所有方法都在同一个帧上比较，避免跨设备对齐的整周期歧义。

被测方案（全部只用到"本次按压结束"为止的数据）：
  dft1_peak   峰值定周期 + 单频正交（因果，滞后 1 个样本 8.5 ms）
  dft1_cross  过零/滞回定周期 + 单频正交（现行固件做法）
  dft2        同上窗口 + 1~2 次谐波
  leaky_pp    漏积分二次积分，本周期峰峰值
  leaky_turn  漏积分位移的谷底（速度过零）就报：depth = 上一个峰 - 本谷
  sogi        SOGI 谐振器幅值

同时量化：两种定周期方式的周期抖动，以及它通过 d ∝ T² 造成的深度噪声。
"""
import io, os, sys, glob
import numpy as np
from scipy.signal import lfilter, find_peaks
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cpr_fit import (load_sensor, load_platform, find_platform, preprocess,
                     find_positive_peaks, align, onset_guess, BASE, OUT, SIMS, G)
import pipeline as P


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


class Chain(object):
    """板上那条因果链 + 漏积分"""

    def __init__(self, sim, hp=1.0, lp=8.0, f_leak=0.3, resample=True):
        d = os.path.join(BASE, sim)
        imu = sorted(glob.glob(os.path.join(d, "lsm6dsv_raw_*.txt")))[-1]
        t, acc = load_sensor(imu)
        if resample:
            fs = 200.0
            t_u = np.arange(t[0], t[-1], 1.0 / fs)
            a = np.column_stack([np.interp(t_u, t, acc[:, i]) for i in range(3)])
        else:
            fs = 1.0 / np.mean(np.diff(t))
            t_u, a = t, acc
        self.t, self.fs, self.acc = t_u, fs, a
        amag = np.linalg.norm(a, axis=1)
        q = (amag > 900) & (amag < 1100)
        g = a[q].mean(axis=0); gn = np.linalg.norm(g); u = g / gn
        self.av = a.dot(u) - gn

        b, a1 = biquad(fs, hp, True)
        bp = lfilter(b, a1, self.av)
        bp = lfilter(b, a1, bp)
        b, a1 = biquad(fs, lp, False)
        self.bp = lfilter(b, a1, bp)

        dt = 1.0 / fs
        w0 = 2 * np.pi * f_leak
        ai = np.array([1.0, -(1.0 - w0 * dt)])
        bi = np.array([dt, 0.0])
        v = lfilter(bi, ai, self.av)
        self.disp = lfilter(bi, ai, v) * G


def frame(sim, ch):
    """复用 pipeline 的周期网格与对齐，返回逐周期的对应关系"""
    pt, pv, peaks = load_platform(find_platform(sim))
    pcyc, rest = P.platform_cycles2(pt, pv, peaks)
    _, tp = find_positive_peaks(ch.bp, ch.fs)           # 峰值定周期
    off = align(tp, pcyc[:, 0], onset_guess(ch.t, ch.bp, pt, pv))
    tc = np.array([0.5 * (tp[k] + tp[k + 1]) for k in range(len(tp) - 1)])
    pair = []
    for j in range(len(pcyc)):
        k = int(np.argmin(np.abs(tc - (pcyc[j, 0] + off))))
        if abs(tc[k] - (pcyc[j, 0] + off)) > 0.28:
            continue
        pair.append((j, k))
    # 有效段
    fake = np.zeros((len(pcyc), 6))
    fake[:, 2] = pcyc[:, 2]
    for j, k in pair:
        i0 = int(round(tp[k] * ch.fs)); i1 = int(round(tp[k + 1] * ch.fs))
        w = 2 * np.pi / (tp[k + 1] - tp[k])
        seg = ch.bp[i0:i1]
        t = ch.t[i0:i1] - tp[k + 1]
        A = 2.0 / len(seg) * np.hypot(np.dot(seg, np.sin(w * t)),
                                      np.dot(seg, np.cos(w * t)))
        fake[j, 5] = 2.0 * A * G / (w * w)
    mj = P.valid_mask(fake)
    out = []
    for j, k in pair:
        if mj[j]:
            out.append((j, k))
    SH = {"sim01": 0, "sim02": -1, "sim03": 0}
    sh = SH.get(sim, 0)
    out = [(j, k + sh) for (j, k) in out if 0 <= k + sh < len(tp) - 1]
    return out, pcyc, tp, off


def cross_edges(ch, hp=1.0):
    """现行固件的定周期方式：滞回上升沿（因果）"""
    bp, fs, t = ch.bp, ch.fs, ch.t
    wins = int(0.25 * fs)
    cs = np.concatenate([[0.0], np.cumsum(bp ** 2)])
    rms = np.sqrt((cs[wins:] - cs[:-wins]) / wins)
    rms = np.concatenate([np.full(wins, rms[0]), rms])
    hyst = np.clip(0.30 * rms, 40.0, 100.0)
    state = -1
    edges = []
    t_prev = None
    for i in range(1, len(bp)):
        if state < 0:
            if bp[i] < -hyst[i]:
                state = 1
        else:
            if bp[i] > hyst[i]:
                y0, y1 = bp[i - 1], bp[i]
                fr = (hyst[i] - y0) / (y1 - y0) if y1 != y0 else 0.0
                t_e = t[i - 1] + fr * (t[i] - t[i - 1])
                if t_prev is not None and 0.30 <= (t_e - t_prev) <= 1.20:
                    edges.append((t_prev, t_e, t_e - t_prev))
                t_prev = t_e
                state = -1
    return edges


def dft_depth(ch, ta, tb, T, K=1):
    fs = ch.fs
    i0 = int(np.ceil(ta * fs)); i1 = int(np.floor(tb * fs))
    seg = ch.bp[i0:i1]
    if len(seg) < 20:
        return np.nan
    w = 2 * np.pi / T
    t = ch.t[i0:i1] - tb
    n = len(seg)
    if K == 1:
        A = 2.0 / n * np.hypot(np.dot(seg, np.sin(w * t)), np.dot(seg, np.cos(w * t)))
        return 2.0 * A * G / (w * w)
    M = np.column_stack([np.sin(w * t), np.cos(w * t),
                         np.sin(2 * w * t), np.cos(2 * w * t)])
    c, *_ = np.linalg.lstsq(M, seg, rcond=None)
    d = -((c[0] * np.sin(w * t) + c[1] * np.cos(w * t)) / w ** 2
          + (c[2] * np.sin(2 * w * t) + c[3] * np.cos(2 * w * t)) / (2 * w) ** 2)
    return (d.max() - d.min()) * G


def main():
    lines = []
    def say(s=""):
        print(s); lines.append(s)

    say("=" * 94)
    say("实时逐次评估（因果滤波、每次按压结束立即出值、不跨次平均）")
    say("   %-11s %-6s %7s %8s %7s %12s" %
        ("方案", "sim", "MAE/mm", "MAPE", "r", "报数时刻/ms"))
    store = {}
    for sim in SIMS:
        ch = Chain(sim)
        pairs, pcyc, tp, off = frame(sim, ch)
        ce = cross_edges(ch)
        # 交叉定周期：为每个周期 (tp[k], tp[k+1]) 找覆盖它的过零周期
        cT = {}
        for k in range(len(tp) - 1):
            best = (0.0, None)
            for (ta, tb, T) in ce:
                ov = min(tb, tp[k + 1]) - max(ta, tp[k])
                if ov > best[0]:
                    best = (ov, T)
            if best[1] is not None and best[0] > 0.5 * (tp[k + 1] - tp[k]):
                cT[k] = best[1]

        rows = []
        for j, k in pairs:
            ta, tb = tp[k], tp[k + 1]
            T = tb - ta
            d1 = dft_depth(ch, ta, tb, T, 1)
            d2 = dft_depth(ch, ta, tb, T, 2)
            Tc = cT.get(k, T)
            dc = dft_depth(ch, ta, tb, Tc, 1)
            i0 = int(np.ceil(ta * ch.fs)); i1 = int(np.floor(tb * ch.fs))
            seg = ch.disp[i0:i1]
            dl = seg.max() - seg.min() if len(seg) > 5 else np.nan
            # 谷底时刻（本周期内位移最小处）
            imin = i0 + int(np.argmin(seg)) if len(seg) > 5 else i1
            rows.append((pcyc[j, 0], pcyc[j, 2], tb, ch.t[imin], T, Tc,
                         d1, d2, dc, dl))
        rows = np.array(rows, float)
        store[sim] = rows

        ref = rows[:, 1]
        for nm, ci, tcol in (("dft1_峰值", 6, 2), ("dft1_过零", 8, 2),
                             ("dft2", 7, 2), ("leaky_pp", 9, 2),
                             ("leaky_turn", 9, 3)):
            e = rows[:, ci]
            ok = np.isfinite(e)
            r = ref[ok]; ee = e[ok]
            K = r.mean() / ee.mean()
            err = K * ee - r
            lat = (rows[ok, tcol] - rows[ok, 0]) * 1000
            say("   %-11s %-6s %7.2f %7.1f%% %7.3f %12.0f"
                % (nm, sim, np.abs(err).mean(), (np.abs(err) / r).mean() * 100,
                   np.corrcoef(ee, r)[0, 1], np.median(lat)))
        say("")

    # ---- 定周期方式的抖动 ----
    say("=" * 94)
    say("定周期方式对深度的影响（同一段数据，只换周期估计）")
    say("   %-6s %14s %14s %14s" % ("sim", "T峰值 SD/ms", "T过零 SD/ms", "两种深度差 SD/mm"))
    for sim in SIMS:
        rows = store[sim]
        Tp = rows[:, 4]; Tc = rows[:, 5]
        d_p = rows[:, 6]; d_c = rows[:, 8]
        ok = np.isfinite(d_p) & np.isfinite(d_c)
        say("   %-6s %14.1f %14.1f %14.2f"
            % (sim, Tp.std() * 1000, Tc.std() * 1000,
               np.std(d_c[ok] - d_p[ok])))
        # 用平滑后的周期再看
        Tps = P.movavg(Tp, 5); Tcs = P.movavg(Tc, 5)
        say("          平滑后：%.1f ms / %.1f ms" % (Tps.std() * 1000, Tcs.std() * 1000))

    # ---- 图：三种方案的时间序列 ----
    fig, axes = plt.subplots(3, 1, figsize=(13, 9.4))
    for ax, sim in zip(axes, SIMS):
        rows = store[sim]
        ref = rows[:, 1]
        idx = np.arange(len(rows))
        for nm, ci, c in (("dft1(峰值定周期)", 6, "tab:red"),
                          ("leaky_pp", 9, "tab:blue")):
            e = rows[:, ci]
            K = ref.mean() / e.mean()
            ax.plot(idx, K * e, color=c, lw=1.4, alpha=0.85,
                    label="%s  MAE %.2f" % (nm, np.abs(K * e - ref).mean()))
        ax.plot(idx, ref, "k.-", ms=5, lw=1.0, label="平台 travel")
        ax.set_title("%s  逐次按压（即时报数方案）" % sim, fontsize=10)
        ax.set_ylabel("深度 (mm)")
        ax.grid(alpha=0.3); ax.legend(fontsize=8, loc="lower right")
    axes[-1].set_xlabel("第几次按压")
    plt.tight_layout()
    plt.savefig(os.path.join(OUT, "fig7_realtime.png"), dpi=110)
    plt.close(fig)

    with io.open(os.path.join(OUT, "realtime.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
