# -*- coding: utf-8 -*-
"""
CPR 深度/频率拟合主脚本
========================
读取 imu_process_03/sim01..sim03 的 IMU 原始数据 + 标准测试平台 csv，
在每一次按压上计算多种深度估计，并与平台真值对比，
最后给出可写进固件的标定系数。

估计量：
  rms      d = 2*sqrt(2)*a_rms/w^2             （旧固件）
  fit1     一次谐波最小二乘                    （基波拟合）
  fit2     1~2 次谐波最小二乘
  fit3     1~3 次谐波最小二乘
  int      逐周期二次积分 + 线性去趋势

真值口径：
  travel  平台波形一个周期内的峰峰值（扣除“手压着台面的静息载荷”）
  report  平台自己报的 mannequin_depth（= 峰 - 初始静置基线）

输出：fit_work/out/*.csv, *.png, report.md
"""
import io, os, csv, glob, json
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
SIMS = ["sim01", "sim02", "sim03"]
ACC_LSB = 0.122          # mg/LSB  (+-4 g)
G = 9.80665

if not os.path.isdir(OUT):
    os.makedirs(OUT)


# ----------------------------------------------------------------------
#  读取
# ----------------------------------------------------------------------
def load_sensor(path):
    t, acc = [], []
    with io.open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            p = s.split(",")
            if len(p) != 8:
                continue
            try:
                v = [int(x) for x in p]
            except ValueError:
                continue
            t.append(v[0] / 1000.0)
            acc.append(v[1:4])
    return np.array(t), np.array(acc, float) * ACC_LSB


def find_platform(sim):
    d = os.path.join(BASE, sim)
    p = os.path.join(d, "platform_raw.csv")
    if os.path.exists(p):
        return p
    return [c for c in glob.glob(os.path.join(d, "*", "*.csv"))
            if "校验" not in os.path.basename(c)][0]


def load_platform(path):
    wave_t, wave_v, peaks = [], [], []
    with io.open(path, encoding="utf-8-sig", errors="replace") as f:
        for r in csv.DictReader(f):
            if r["source"] == "mannequin" and r["mannequin_raw"]:
                try:
                    wave_t.append(float(r["elapsed_s"]))
                    wave_v.append(float(r["mannequin_raw"]))
                except ValueError:
                    pass
            elif r["source"] == "mannequin_peak" and r["mannequin_depth"]:
                try:
                    peaks.append((float(r["elapsed_s"]), float(r["mannequin_depth"]) * 10.0,
                                  float(r["mannequin_frequency"]) if r["mannequin_frequency"] else np.nan))
                except ValueError:
                    pass
    o = np.argsort(wave_t)
    wave_t = np.array(wave_t)[o]
    wave_v = np.array(wave_v)[o]
    peaks.sort()
    return wave_t, wave_v, np.array(peaks, float)


# ----------------------------------------------------------------------
#  预处理
# ----------------------------------------------------------------------
def preprocess(t, acc, fs_u=200.0, f_lo=0.6, f_hi=15.0):
    """重采样到均匀网格 + 重力投影 + 带通，返回 (t_u, bp, av, fs_u)"""
    t_u = np.arange(t[0], t[-1], 1.0 / fs_u)
    ax = np.interp(t_u, t, acc[:, 0])
    ay = np.interp(t_u, t, acc[:, 1])
    az = np.interp(t_u, t, acc[:, 2])
    amag = np.sqrt(ax ** 2 + ay ** 2 + az ** 2)
    quiet = (amag > 900) & (amag < 1100)
    gvec = np.array([ax[quiet].mean(), ay[quiet].mean(), az[quiet].mean()])
    gn = np.linalg.norm(gvec)
    u = gvec / gn
    av = ax * u[0] + ay * u[1] + az * u[2] - gn
    b, a = signal.butter(3, [f_lo / (fs_u / 2), f_hi / (fs_u / 2)], btype="band")
    bp = signal.filtfilt(b, a, av)
    return t_u, bp, av, fs_u


def find_positive_peaks(bp, fs_u):
    """带通信号的正峰 + 抛物线插值，返回亚采样精度的时刻"""
    idx, _ = signal.find_peaks(bp, distance=int(0.30 * fs_u),
                               prominence=0.15 * np.std(bp))
    tp = []
    for i in idx:
        if i <= 0 or i >= len(bp) - 1:
            continue
        y0, y1, y2 = bp[i - 1], bp[i], bp[i + 1]
        den = (y0 - 2 * y1 + y2)
        d = 0.5 * (y0 - y2) / den if abs(den) > 1e-12 else 0.0
        tp.append((i + d) / fs_u)
    return idx, np.array(tp)


# ----------------------------------------------------------------------
#  逐周期深度估计
# ----------------------------------------------------------------------
def _design(n, dt, w, K, const=True):
    tt = np.arange(n) * dt
    cols = []
    names = []
    for k in range(1, K + 1):
        cols.append(np.sin(k * w * tt)); names.append("s%d" % k)
        cols.append(np.cos(k * w * tt)); names.append("c%d" % k)
    if const:
        cols.append(np.ones(n)); names.append("1")
    return np.column_stack(cols)


def harmonic_travel(seg, dt, w, K, const=True):
    """最小二乘拟合 a(t)=sum_k[A_k sin+b_k cos]+c，两次积分得位移，返回峰峰值(mm)"""
    n = len(seg)
    if n < 2 * K + 4:
        return np.nan
    M = _design(n, dt, w, K, const)
    coef, *_ = np.linalg.lstsq(M, seg, rcond=None)
    tt = np.arange(n) * dt
    d = np.zeros(n)
    for k in range(1, K + 1):
        A = coef[2 * (k - 1)]
        B = coef[2 * (k - 1) + 1]
        wk = k * w
        # a = A sin + B cos  ->  d = -(A sin + B cos)/wk^2
        d += -(A * np.sin(wk * tt) + B * np.cos(wk * tt)) / (wk * wk)
    d *= G                      # mg -> mm/s^2 , then /w^2 gives mm
    return d.max() - d.min()


def integral_travel(seg, dt):
    n = len(seg)
    if n < 8:
        return np.nan
    a = seg * G
    v = np.cumsum(a) * dt
    v -= np.linspace(v[0], v[-1], n)
    d = np.cumsum(v) * dt
    d -= np.linspace(d[0], d[-1], n)
    return d.max() - d.min()


def cycle_features(bp, bp_idx, tp, fs_u):
    """返回每个周期一行: t_center, T, rms, fit1, fit2, fit3, int"""
    dt = 1.0 / fs_u
    rows = []
    for k in range(len(tp) - 1):
        i0 = int(round(tp[k] * fs_u))
        i1 = int(round(tp[k + 1] * fs_u))
        if i1 - i0 < int(0.30 * fs_u) or i1 - i0 > int(1.2 * fs_u):
            continue
        T = tp[k + 1] - tp[k]
        w = 2 * np.pi / T
        seg = bp[i0:i1]
        d_rms = 2.8284271 * np.sqrt(np.mean(seg ** 2)) * G / (w * w)
        rows.append((0.5 * (tp[k] + tp[k + 1]), T, d_rms,
                     harmonic_travel(seg, dt, w, 1),
                     harmonic_travel(seg, dt, w, 2),
                     harmonic_travel(seg, dt, w, 3),
                     integral_travel(seg, dt)))
    return np.array(rows)


# ----------------------------------------------------------------------
#  平台真值
# ----------------------------------------------------------------------
def platform_cycles(pt, pv, peaks):
    """每个平台峰值对应一个周期：返回 (tp, T_plat, travel, report)"""
    pks = peaks[:, 0]
    rep = peaks[:, 1]
    rest = np.median(pv[pt < 1.5]) if (pt < 1.5).sum() > 3 else pv.min()
    out = []
    for i in range(len(pks)):
        lo = pks[i] - (pks[i] - pks[i - 1] if i > 0 else 0.55)
        m = (pt >= lo) & (pt <= pks[i] + 0.15)
        if m.sum() < 4:
            continue
        mx = pv[m].max(); mi = pv[m].min()
        T = pks[i] - pks[i - 1] if i > 0 else np.nan
        out.append((pks[i], T, mx - mi, rep[i], mi - rest, mx, mi))
    return np.array(out, float), rest


def align(cyc_t, pk_t, guess):
    """让 cyc_t 与 pk_t 对齐：在 guess 附近挑匹配数最多、平均距离最小的偏移"""
    best = (1e9, 0.0)
    for off in np.arange(guess - 0.30, guess + 0.30, 0.005):
        d = np.abs(cyc_t[None, :] - (pk_t[:, None] + off))
        dist = d.min(axis=1)
        dist = dist[dist < 0.30]
        if len(dist) < 5:
            continue
        score = dist.mean() - 1e-4 * len(dist)
        if score < best[0]:
            best = (score, off)
    return best[1]


def onset_guess(t, bp, pt, pv):
    fs = 1.0 / np.mean(np.diff(t)) if hasattr(t, "__len__") else 117.0
    env = np.convolve(np.abs(bp), np.ones(int(0.2 * 200)) / int(0.2 * 200), mode="same")
    ti = np.arange(len(bp)) / 200.0
    oi = ti[np.argmax(env > 0.25 * env.max())]
    pvs = np.convolve(pv, np.ones(5) / 5.0, mode="same")
    thr = pv.min() + 0.25 * (pv.max() - pv.min())
    op = pt[np.argmax(pvs > thr)]
    return oi - op


# ----------------------------------------------------------------------
def process(sim, verbose=True):
    d = os.path.join(BASE, sim)
    imu = sorted(glob.glob(os.path.join(d, "lsm6dsv_raw_*.txt")))[-1]
    t, acc = load_sensor(imu)
    pt, pv, peaks = load_platform(find_platform(sim))

    t_u, bp, av, fs_u = preprocess(t, acc)
    pk_idx, tp = find_positive_peaks(bp, fs_u)
    cyc = cycle_features(bp, pk_idx, tp, fs_u)
    pcyc, rest = platform_cycles(pt, pv, peaks)

    guess = onset_guess(t_u, bp, pt, pv)
    off = align(cyc[:, 0], pcyc[:, 0], guess)

    # 一对一匹配
    rows = []
    for j in range(len(pcyc)):
        tp_j = pcyc[j, 0] + off
        k = int(np.argmin(np.abs(cyc[:, 0] - tp_j)))
        if abs(cyc[:, 0][k] - tp_j) > 0.28:
            continue
        rows.append((pcyc[j, 0], pcyc[j, 1], pcyc[j, 2], pcyc[j, 3], pcyc[j, 4],
                     cyc[k, 0], cyc[k, 1], cyc[k, 2], cyc[k, 3], cyc[k, 4],
                     cyc[k, 5], cyc[k, 6]))
    rows = np.array(rows, float)
    if verbose:
        print("%s: IMU cycles %d, platform peaks %d, matched %d, offset %.2f (onset %.2f), rest %.1f mm"
              % (sim, len(cyc), len(pcyc), len(rows), off, guess, rest))
    return dict(sim=sim, imu=os.path.basename(imu), t=t, acc=acc, t_u=t_u, bp=bp,
                cyc=cyc, pcyc=pcyc, rows=rows, off=off, rest=rest,
                pt=pt, pv=pv, peaks=peaks)


NAMES = ["rms", "fit1", "fit2", "fit3", "int"]


def stats(est, gt):
    err = est - gt
    return dict(bias=err.mean(), mae=np.abs(err).mean(),
                rmse=np.sqrt((err ** 2).mean()),
                mape=(np.abs(err) / gt).mean() * 100,
                r=np.corrcoef(est, gt)[0, 1] if len(gt) > 3 else np.nan)


def main():
    res = {}
    for s in SIMS:
        res[s] = process(s)

    # 有效窗口：去掉每个文件开头 6 个和后 2 个周期（平台前几次不稳）
    SKIP_HEAD, SKIP_TAIL = 8, 2

    allrows = []
    lines = []
    def say(x=""):
        print(x); lines.append(x)

    say("=" * 78)
    say("CPR 深度/频率拟合报告")
    say("=" * 78)
    for s in SIMS:
        r = res[s]
        say("%s  IMU=%s  匹配 %d 周期  对齐 %.2f s  静置基线 %.1f mm"
            % (s, r["imu"], len(r["rows"]), r["off"], r["rest"]))
    say("")

    # ---------- per-file table ----------
    summary = []
    for s in SIMS:
        r = res[s]
        rows = r["rows"][SKIP_HEAD:len(r["rows"]) - SKIP_TAIL]
        gt_t = rows[:, 2]
        gt_r = rows[:, 3]
        say("-" * 78)
        say("%s  (统计 %d 个周期)" % (s, len(rows)))
        say("  平台: travel 均值 %.1f mm | report 均值 %.1f mm | 差值(手压静息) %.1f mm"
            % (gt_t.mean(), gt_r.mean(), (gt_r - gt_t).mean()))
        say("  %-6s %8s %8s %8s %8s %8s" % ("方法", "均值", "偏差", "MAE", "MAPE", "r"))
        for k, nm in enumerate(NAMES):
            est = rows[:, 7 + k]
            st = stats(est, gt_t)
            say("  %-6s %8.1f %+8.1f %8.1f %7.1f%% %8.3f"
                % (nm, est.mean(), st["bias"], st["mae"], st["mape"], st["r"]))
            summary.append(dict(sim=s, method=nm, mean=est.mean(), **st))
        say("")
        for i in range(rows.shape[0]):
            allrows.append([s] + list(rows[i]))

    # ---------- 汇总：三种口径 ----------
    say("=" * 78)
    say("三种估计量在三个文件上的总体表现（vs 平台 travel）")
    say("  %-6s %8s %8s %8s %8s %8s" % ("方法", "MAE", "RMSE", "MAPE", "偏差", "r"))
    for nm in NAMES:
        sub = [x for x in summary if x["method"] == nm]
        say("  %-6s %8.1f %8.1f %7.1f%% %+8.1f %8.3f"
            % (nm, np.mean([x["mae"] for x in sub]),
               np.mean([x["rmse"] for x in sub]),
               np.mean([x["mape"] for x in sub]),
               np.mean([x["bias"] for x in sub]),
               np.mean([x["r"] for x in sub])))

    # ---------- 标定：est -> platform travel ----------
    say("")
    say("=" * 78)
    say("标定 y = a*x + b  (x = IMU 估计, y = 平台 travel)  逐文件检验")
    for nm in ["fit1", "fit2", "fit3", "int"]:
        say("  -- %s --" % nm)
        for s in SIMS:
            r = res[s]
            rows = r["rows"][SKIP_HEAD:len(r["rows"]) - SKIP_TAIL]
            x = rows[:, 7 + NAMES.index(nm)]
            y = rows[:, 2]
            a, b = np.polyfit(x, y, 1)
            pred = a * x + b
            say("     %s  a=%.3f b=%+.1f  标定后 MAE %.1f mm (%.1f%%)"
                % (s, a, b, np.abs(pred - y).mean(),
                   (np.abs(pred - y) / y).mean() * 100))
        # 汇总三文件联合拟合 + 留一文件交叉验证
        X, Y, Sid = [], [], []
        for s in SIMS:
            rows = res[s]["rows"][SKIP_HEAD:len(res[s]["rows"]) - SKIP_TAIL]
            X.append(rows[:, 7 + NAMES.index(nm)]); Y.append(rows[:, 2])
            Sid.append(np.array([s] * len(X[-1])))
        X = np.concatenate(X); Y = np.concatenate(Y); Sid = np.concatenate(Sid)
        a, b = np.polyfit(X, Y, 1)
        pred = a * X + b
        say("     联合: a=%.3f b=%+.1f  MAE %.1f mm (%.1f%%)  r=%.3f"
            % (a, b, np.abs(pred - Y).mean(), (np.abs(pred - Y) / Y).mean() * 100,
               np.corrcoef(X, Y)[0, 1]))
        for s in SIMS:
            m = Sid != s
            a2, b2 = np.polyfit(X[m], Y[m], 1)
            pred2 = a2 * X[~m] + b2
            say("     留出 %s: a=%.3f b=%+.1f -> MAE %.1f mm (%.1f%%)"
                % (s, a2, b2, np.abs(pred2 - Y[~m]).mean(),
                   (np.abs(pred2 - Y[~m]) / Y[~m]).mean() * 100))

    # ---------- 频率 ----------
    say("")
    say("=" * 78)
    say("按压频率（CPR rate）")
    say("  %-6s %10s %10s %10s %10s" % ("", "IMU cpm", "平台T cpm", "平台上报", "IMU-上报"))
    for s in SIMS:
        r = res[s]
        rows = r["rows"][SKIP_HEAD:len(r["rows"]) - SKIP_TAIL]
        imu_cpm = 60.0 / rows[:, 6]
        plat_cpm = 60.0 / rows[:, 1]
        rep_cpm = r["peaks"][SKIP_HEAD:len(r["peaks"]) - SKIP_TAIL, 2]
        say("  %-6s %10.1f %10.1f %10.1f %10.1f"
            % (s, imu_cpm.mean(), plat_cpm.mean(), np.nanmean(rep_cpm),
               (imu_cpm - rep_cpm).mean() if len(rep_cpm) == len(imu_cpm) else np.nan))

    # ---------- 写 csv ----------
    hdr = ("sim,plat_t,plat_T,plat_travel,plat_report,plat_restoff,"
           "imu_t,imu_T,imu_rms,imu_fit1,imu_fit2,imu_fit3,imu_int\n")
    with io.open(os.path.join(OUT, "cycles_all.csv"), "w", encoding="utf-8", newline="") as f:
        f.write(hdr)
        for r in allrows:
            f.write("%s," % r[0] + ",".join("%.3f" % v for v in r[1:]) + "\n")

    with io.open(os.path.join(OUT, "report.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("\n-> out/cycles_all.csv, out/report.md")


if __name__ == "__main__":
    main()
