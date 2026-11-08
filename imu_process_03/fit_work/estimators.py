# -*- coding: utf-8 -*-
"""
估计量对比实验：怎样把逐次按压的深度做得更稳
=============================================
候选：
  fit1       周期取相邻正峰间隔 + 1 次谐波拟合（现行最优）
  fit1s      周期做 5 点滑动平均后再拟合（压掉周期抖动）
  fitw       周期由 2 周期窗口联合估计（更准的 omega）
  env        1.5~3.5 Hz 窄带 + 解析包络，depth=2*env/omega^2
  int        逐周期二次积分
  ints       逐周期二次积分，窗长取平滑周期
每个再叠加 5 次按压滑动平均（feedback 窗口）
"""
import io, os, sys, csv, glob
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cpr_fit import (load_sensor, load_platform, find_platform, preprocess,
                     find_positive_peaks, _design, harmonic_travel, G,
                     platform_cycles, align, onset_guess, BASE, OUT, SIMS)

SKIP_HEAD, SKIP_TAIL = 8, 2


def movavg(x, n=5):
    x = np.asarray(x, float)
    k = np.ones(n) / n
    pad = n // 2
    xp = np.concatenate([np.repeat(x[0], pad), x, np.repeat(x[-1], pad)])
    y = np.convolve(xp, k, mode="same")
    return y[pad:pad + len(x)]


def freq_from_window(bp, fs, tp, k, nwin=2):
    """在 k..k+nwin 个周期上联合拟合一个 omega（用基波相位）"""
    i0 = int(round(tp[k] * fs))
    i1 = int(round(tp[min(k + nwin, len(tp) - 1)] * fs))
    if i1 - i0 < 20:
        return np.nan
    seg = bp[i0:i1]
    n = len(seg)
    dt = 1.0 / fs
    tt = np.arange(n) * dt
    T0 = tp[min(k + nwin, len(tp) - 1)] - tp[k]
    best = (1e18, T0)
    for T in np.arange(T0 * 0.94, T0 * 1.06, 0.0005):
        w = 2 * np.pi / T
        M = np.column_stack([np.sin(w * tt), np.cos(w * tt), np.ones(n)])
        c, *_ = np.linalg.lstsq(M, seg, rcond=None)
        r = seg - M.dot(c)
        s = float(r.dot(r))
        if s < best[0]:
            best = (s, T)
    return 2 * np.pi / best[1]


def envelope_depth(bp, fs, f_lo=1.4, f_hi=3.6):
    b, a = signal.butter(3, [f_lo / (fs / 2), f_hi / (fs / 2)], btype="band")
    nb = signal.filtfilt(b, a, bp)
    env = np.abs(signal.hilbert(nb))
    return env


def build(sim):
    d = os.path.join(BASE, sim)
    imu = sorted(glob.glob(os.path.join(d, "lsm6dsv_raw_*.txt")))[-1]
    t, acc = load_sensor(imu)
    pt, pv, peaks = load_platform(find_platform(sim))
    t_u, bp, av, fs_u = preprocess(t, acc)
    pk_idx, tp = find_positive_peaks(bp, fs_u)
    dt = 1.0 / fs_u
    pcyc, rest = platform_cycles(pt, pv, peaks)
    off = align(tp, pcyc[:, 0], onset_guess(t_u, bp, pt, pv))

    Traw = np.array([tp[k + 1] - tp[k] for k in range(len(tp) - 1)])
    Ts = movavg(Traw, 5)
    env = envelope_depth(bp, fs_u)

    out = []
    for k in range(len(tp) - 1):
        i0 = int(round(tp[k] * fs_u)); i1 = int(round(tp[k + 1] * fs_u))
        if i1 - i0 < int(0.30 * fs_u) or i1 - i0 > int(1.2 * fs_u):
            continue
        seg = bp[i0:i1]
        tc = 0.5 * (tp[k] + tp[k + 1])
        w_raw = 2 * np.pi / Traw[k]
        w_s = 2 * np.pi / Ts[k]
        w_w = freq_from_window(bp, fs_u, tp, k, 2)
        r = dict(t=tc, T=Traw[k],
                 fit1=harmonic_travel(seg, dt, w_raw, 1),
                 fit1s=harmonic_travel(seg, dt, w_s, 1),
                 fitw=harmonic_travel(seg, dt, w_w, 1) if np.isfinite(w_w) else np.nan,
                 env=2.0 * env[i0:i1].mean() * G / (w_s * w_s),
                 intg=None)
        # int
        a = seg * G
        v = np.cumsum(a) * dt
        v -= np.linspace(v[0], v[-1], len(seg))
        dd = np.cumsum(v) * dt
        dd -= np.linspace(dd[0], dd[-1], len(seg))
        r["intg"] = dd.max() - dd.min()
        out.append(r)
    ts = np.array([o["t"] for o in out])

    rows = []
    for j in range(len(pcyc)):
        k = int(np.argmin(np.abs(ts - (pcyc[j, 0] + off))))
        if abs(ts[k] - (pcyc[j, 0] + off)) > 0.28:
            continue
        o = out[k]
        rows.append((pcyc[j, 0], pcyc[j, 1], pcyc[j, 2], pcyc[j, 3],
                     o["fit1"], o["fit1s"], o["fitw"], o["env"], o["intg"]))
    return np.array(rows, float), dict(off=off, rest=rest, Traw=Traw)


METHODS = [("fit1", 4), ("fit1s", 5), ("fitw", 6), ("env", 7), ("intg", 8)]


def main():
    lines = []
    def say(s=""):
        print(s); lines.append(s)

    data = {}
    for s in SIMS:
        rows, meta = build(s)
        rows = rows[SKIP_HEAD:len(rows) - SKIP_TAIL]
        data[s] = rows
        say("%s: %d 周期  T sd=%.4f s (%.1f%%)" %
            (s, len(rows), meta["Traw"].std(), meta["Traw"].std() / meta["Traw"].mean() * 100))

    say("")
    say("%-7s %-7s %8s %8s %8s %8s %8s" %
        ("sim", "方法", "MAE", "MAPE", "偏差", "SD_est", "r"))
    for s in SIMS:
        rows = data[s]
        gt = rows[:, 2]
        say("  --- %s (真值 sd=%.1f mm) ---" % (s, gt.std()))
        for nm, ci in METHODS:
            est = rows[:, ci]
            ok = np.isfinite(est)
            e = est[ok] - gt[ok]
            say("  %-6s %-7s %8.1f %7.1f%% %+8.1f %8.1f %8.3f"
                % (s, nm, np.abs(e).mean(), (np.abs(e) / gt[ok]).mean() * 100,
                   e.mean(), est[ok].std(), np.corrcoef(est[ok], gt[ok])[0, 1]))
        say("")

    say("=" * 70)
    say("5 次按压滑动平均之后（相当于设备显示的深度）")
    say("%-7s %-7s %8s %8s %8s %8s" % ("sim", "方法", "MAE", "MAPE", "偏差", "r"))
    for s in SIMS:
        rows = data[s]
        gt = movavg(rows[:, 2], 5)
        for nm, ci in METHODS:
            est = movavg(rows[:, ci], 5)
            e = est - gt
            say("  %-6s %-7s %8.1f %7.1f%% %+8.1f %8.3f"
                % (s, nm, np.abs(e).mean(), (np.abs(e) / gt).mean() * 100,
                   e.mean(), np.corrcoef(est, gt)[0, 1]))
        say("")

    with io.open(os.path.join(OUT, "estimators.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


if __name__ == "__main__":
    main()
