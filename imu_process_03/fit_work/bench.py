# -*- coding: utf-8 -*-
"""
深度估计方法横向基准（bench）
============================
统一口径：每个平台峰值 j 对应一个 IMU 周期 k，估计量 est[k] 与之比较。
指标：逐次 MAE/MAPE（vs 平台 travel）、5 次滑动平均 MAE/MAPE、r。

被测方法
  dft1       单周期单频正交检测（现行）
  dft1_2T    2 个周期窗口做同一个正交检测（天然 2 次平均）
  dft3T      3 个周期窗口
  dft2       1~2 次谐波（拟合出位移再取峰峰）
  intwin     全段频域二次积分（连续位移），逐周期取峰峰
  sogi       SOGI 自适应谐振器（逐样本跟踪基波幅值）
  sogi2k     SOGI k 扫描后在最优 k 上的结果
  pca        在"运动主方向"投影而不是重力方向
  ridge      特征回归 A1,A2,A3,T,rms -> 深度（留一实验交叉验证）
"""
import io, os, sys, glob
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cpr_fit import (load_sensor, load_platform, find_platform, preprocess,
                     find_positive_peaks, align, onset_guess, G, BASE, OUT, SIMS)
import pipeline as P

WARM = 6


# ----------------------------------------------------------------------
def harness(sim):
    d = os.path.join(BASE, sim)
    imu = sorted(glob.glob(os.path.join(d, "lsm6dsv_raw_*.txt")))[-1]
    t, acc = load_sensor(imu)
    pt, pv, peaks = load_platform(find_platform(sim))
    t_u, bp, av, fs_u = preprocess(t, acc)
    _, tp = find_positive_peaks(bp, fs_u)
    pcyc, rest = P.platform_cycles2(pt, pv, peaks)
    off = align(tp, pcyc[:, 0], onset_guess(t_u, bp, pt, pv))

    # 周期中心
    tc = np.array([0.5 * (tp[k] + tp[k + 1]) for k in range(len(tp) - 1)])

    # 平台峰值 j -> IMU 周期 k
    pair = []
    for j in range(len(pcyc)):
        k = int(np.argmin(np.abs(tc - (pcyc[j, 0] + off))))
        if abs(tc[k] - (pcyc[j, 0] + off)) > 0.28:
            continue
        pair.append((j, k))
    return dict(sim=sim, t=t, acc=acc, t_u=t_u, bp=bp, av=av, fs=fs_u, tp=tp,
                tc=tc, pcyc=pcyc, off=off, pair=pair, acc_raw=acc)


# ----------------------------------------------------------------------
def m_dft1(h, span=1):
    """span 个周期窗口的单频正交检测"""
    bp, fs, tp = h["bp"], h["fs"], h["tp"]
    dt = 1.0 / fs
    out = np.full(len(tp) - 1, np.nan)
    for k in range(len(tp) - span):
        i0 = int(round(tp[k] * fs)); i1 = int(round(tp[k + span] * fs))
        if i1 - i0 < 20:
            continue
        T = (tp[k + span] - tp[k]) / span
        w = 2 * np.pi / T
        seg = bp[i0:i1]
        n = len(seg)
        tt = np.arange(n) * dt
        S = np.dot(seg, np.sin(w * tt)); C = np.dot(seg, np.cos(w * tt))
        A = 2.0 / n * np.hypot(S, C)
        # 该值代表窗口中心那次按压
        out[min(k + span // 2, len(out) - 1)] = 2.0 * A * G / (w * w)
    return out


def m_dft2(h):
    bp, fs, tp = h["bp"], h["fs"], h["tp"]
    dt = 1.0 / fs
    out = np.full(len(tp) - 1, np.nan)
    for k in range(len(tp) - 1):
        i0 = int(round(tp[k] * fs)); i1 = int(round(tp[k + 1] * fs))
        if i1 - i0 < 20:
            continue
        T = tp[k + 1] - tp[k]; w = 2 * np.pi / T
        seg = bp[i0:i1]; n = len(seg); t = np.arange(n) * dt
        M = np.column_stack([np.sin(w * t), np.cos(w * t),
                             np.sin(2 * w * t), np.cos(2 * w * t)])
        c, *_ = np.linalg.lstsq(M, seg, rcond=None)
        d = -((c[0] * np.sin(w * t) + c[1] * np.cos(w * t)) / w ** 2
              + (c[2] * np.sin(2 * w * t) + c[3] * np.cos(2 * w * t)) / (2 * w) ** 2)
        out[k] = (d.max() - d.min()) * G
    return out


def m_intwin(h, f_lo=0.5, f_hi=12.0):
    """全段频域二次积分 -> 连续位移 -> 逐周期峰峰"""
    bp, fs, tp = h["bp"], h["fs"], h["tp"]
    n = len(bp)
    X = np.fft.rfft(bp)
    fr = np.fft.rfftfreq(n, 1.0 / fs)
    H = np.zeros_like(fr)
    band = (fr >= f_lo) & (fr <= f_hi)
    H[band] = -1.0 / ((2 * np.pi * fr[band]) ** 2)
    d = np.fft.irfft(X * H, n=n) * G
    out = np.full(len(tp) - 1, np.nan)
    for k in range(len(tp) - 1):
        i0 = int(round(tp[k] * fs)); i1 = int(round(tp[k + 1] * fs))
        if i1 - i0 < 10:
            continue
        out[k] = d[i0:i1].max() - d[i0:i1].min()
    return out, d


def m_sogi(h, k_gain=0.6, f_range=(1.0, 3.0)):
    """逐样本 SOGI：跟踪基波，输出幅值；频率用一个慢速自适应（自适应律简化：
    先用带通信号的过零估计频率，再固定给 SOGI）"""
    bp, fs, tp = h["bp"], h["fs"], h["tp"]
    dt = 1.0 / fs
    n = len(bp)
    period = np.interp(np.arange(n) / fs, h["tc"], np.gradient(h["tc"]) * 0 +
                       np.array([tp[i + 1] - tp[i] for i in range(len(tp) - 1)]))
    w = 2 * np.pi / period
    w = np.clip(w, 2 * np.pi * f_range[0], 2 * np.pi * f_range[1])
    # 零相位起点的平滑
    v1 = 0.0; v2 = 0.0
    amp = np.zeros(n)
    for i in range(n):
        x = bp[i]
        # SOGI-QSG (2nd order generalized integrator)
        v1n = v1 + dt * (w[i] * (k_gain * (x - v1) - v2))
        v2n = v2 + dt * (w[i] * v1)
        v1, v2 = v1n, v2n
        amp[i] = np.hypot(v1, v2)
    out = np.full(len(tp) - 1, np.nan)
    for k in range(len(tp) - 1):
        i0 = int(round(tp[k] * fs)); i1 = int(round(tp[k + 1] * fs))
        if i1 - i0 < 10:
            continue
        T = tp[k + 1] - tp[k]; ww = 2 * np.pi / T
        out[k] = 2.0 * amp[i0:i1].mean() * G / (ww * ww)
    return out


def m_axis(h, mode="pca"):
    """把加速度投影到运动主方向（PCA）再走 dft1"""
    t, acc, t_u = h["t"], h["acc"], h["t_u"]
    fs = h["fs"]
    ax = np.interp(t_u, t, acc[:, 0]); ay = np.interp(t_u, t, acc[:, 1])
    az = np.interp(t_u, t, acc[:, 2])
    A = np.column_stack([ax, ay, az])
    amag = np.linalg.norm(A, axis=1)
    quiet = (amag > 900) & (amag < 1100)
    g = A[quiet].mean(axis=0); u = g / np.linalg.norm(g)
    # 活跃段
    bp = h["bp"]
    act = np.abs(bp) > 0.5 * np.percentile(np.abs(bp), 90)
    C = np.cov((A[act] - A[act].mean(axis=0)).T)
    w_, v_ = np.linalg.eigh(C)
    p = v_[:, np.argmax(w_)]
    if np.dot(p, u) < 0:
        p = -p
    ang = np.degrees(np.arccos(min(1.0, abs(np.dot(p, u)))))
    av2 = A.dot(p) - A[quiet].dot(p).mean()
    b, a = signal.butter(3, [0.6 / (fs / 2), 15.0 / (fs / 2)], btype="band")
    bp2 = signal.filtfilt(b, a, av2)
    h2 = dict(h); h2["bp"] = bp2
    return m_dft1(h2), ang, np.dot(p, u)


# ----------------------------------------------------------------------
def score(est, ref, mask):
    est = est[mask]; ref = ref[mask]
    ok = np.isfinite(est)
    est = est[ok]; ref = ref[ok]
    if len(est) < 5:
        return None
    # 增益用均值比（不做逐文件线性回归，避免衰减偏置）
    K = ref.mean() / est.mean()
    e = K * est - ref
    return dict(K=K, mae=np.abs(e).mean(), mape=(np.abs(e) / ref).mean() * 100,
                r=np.corrcoef(est, ref)[0, 1], n=len(est))


def score5(est, ref, mask):
    ok = np.isfinite(est)
    mask = mask & ok
    idx = np.where(mask)[0]
    if len(idx) < 8:
        return None
    e = est[idx]; r = ref[idx]
    e5 = P.movavg(e, 5); r5 = P.movavg(r, 5)
    K = r.mean() / e.mean()
    d = K * e5 - r5
    return dict(mae=np.abs(d).mean(), mape=(np.abs(d) / r5).mean() * 100,
                r=np.corrcoef(e5, r5)[0, 1])


def m_leaky(h, f_leak=0.3):
    """因果漏积分二次积分（MCU 可实现）：v' = a - w0 v ; d' = v - w0 d"""
    av, fs, tp = h["av"], h["fs"], h["tp"]
    dt = 1.0 / fs
    w0 = 2 * np.pi * f_leak
    v = 0.0
    d = 0.0
    dd = np.empty(len(av))
    for i in range(len(av)):
        v += (av[i] - w0 * v) * dt
        d += (v - w0 * d) * dt
        dd[i] = d
    dd *= G
    out = np.full(len(tp) - 1, np.nan)
    for k in range(len(tp) - 1):
        i0 = int(round(tp[k] * fs)); i1 = int(round(tp[k + 1] * fs))
        if i1 - i0 < 10:
            continue
        out[k] = dd[i0:i1].max() - dd[i0:i1].min()
    return out


def feats(h):
    """逐周期特征：[A1, A2, A3, T, rms, a_pp]"""
    bp, fs, tp = h["bp"], h["fs"], h["tp"]
    dt = 1.0 / fs
    n = len(tp) - 1
    F = np.full((n, 6), np.nan)
    for k in range(n):
        i0 = int(round(tp[k] * fs)); i1 = int(round(tp[k + 1] * fs))
        if i1 - i0 < 20:
            continue
        T = tp[k + 1] - tp[k]; w = 2 * np.pi / T
        seg = bp[i0:i1]; m = len(seg); t = np.arange(m) * dt
        A = []
        for hh in (1, 2, 3):
            S = np.dot(seg, np.sin(hh * w * t)); C = np.dot(seg, np.cos(hh * w * t))
            A.append(2.0 / m * np.hypot(S, C))
        F[k, :3] = A
        F[k, 3] = T
        F[k, 4] = np.sqrt(np.mean(seg ** 2))
        F[k, 5] = seg.max() - seg.min()
    return F