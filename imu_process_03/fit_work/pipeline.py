# -*- coding: utf-8 -*-
"""
CPR 深度/频率 拟合与标定  ——  最终流程
======================================
输入： imu_process_03/sim01|sim02|sim03 下的
       - lsm6dsv_raw_*.txt            IMU 原始数据 (t_ms, ax..gz, temp)  117 Hz
       - platform_raw.csv / 测量数据.csv   标准测试平台 (20 Hz, mannequin_raw 单位 mm)
输出： imu_process_03/fit_work/out/ 下的图表、csv、txt 报告

处理链
------
1. 重采样到 200 Hz 均匀网格（原始 117 Hz 有 ±2 ms 抖动）
2. 用静置段估计重力方向 u，取竖直分量 av = a·u - |g|
3. 3 阶 Butterworth 0.6~15 Hz 零相位带通 -> bp
4. bp 的正峰（抛物线插值到亚采样）作为周期分界，T = 相邻正峰间隔
5. 每个周期求单频正交分量：
       S = Σ bp_i sin(w t_i),  C = Σ bp_i cos(w t_i),  w = 2π/T
       A = 2/N·√(S²+C²)                 [mg]
       d = 2·A·9.80665 / w²             [mm]    = 峰峰位移（行程）
6. 平台真值：一个周期内 mannequin_raw 的峰峰值（travel）
   平台自报 mannequin_depth = travel + 手上静息载荷偏置
7. 标定： travel ≈ K · d ，K 由三个文件联合确定
"""
import io, os, sys, glob, csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Arial Unicode MS",
                                          "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cpr_fit import (load_sensor, load_platform, find_platform, preprocess,
                     find_positive_peaks, platform_cycles, align, onset_guess,
                     G, BASE, OUT, SIMS)

K_DEPTH = 0.9595        # 统一深度增益（由本脚本 step 3 重新计算并覆盖）
N_AVG = 5               # 设备显示用的滑动平均按压次数
WARMUP = 6              # 平台前几个周期不稳，至少丢掉这么多个


def movavg(x, n=N_AVG):
    x = np.asarray(x, float)
    if len(x) < n:
        return x.copy()
    pad = n // 2
    xp = np.concatenate([np.repeat(x[0], pad), x, np.repeat(x[-1], pad)])
    y = np.convolve(xp, np.ones(n) / n, mode="same")
    return y[pad:pad + len(x)]


def dft_travel(seg, dt, T):
    """单周期单频正交检测 -> 峰峰位移 mm（固件同款）"""
    n = len(seg)
    if n < 8:
        return np.nan
    w = 2 * np.pi / T
    t = np.arange(n) * dt
    S = float(np.dot(seg, np.sin(w * t)))
    C = float(np.dot(seg, np.cos(w * t)))
    A = 2.0 / n * np.hypot(S, C)
    return 2.0 * A * G / (w * w)


def platform_cycles2(pt, pv, peaks):
    """平台逐周期真值，同时带上平台自报频率（pcyc 与 peaks 一一对应）"""
    rest = np.median(pv[pt < 1.5]) if (pt < 1.5).sum() > 3 else pv.min()
    out = []
    for i in range(len(peaks)):
        lo = peaks[i, 0] - (peaks[i, 0] - peaks[i - 1, 0] if i > 0 else 0.55)
        m = (pt >= lo) & (pt <= peaks[i, 0] + 0.15)
        if m.sum() < 4:
            continue
        mx = pv[m].max(); mi = pv[m].min()
        T = peaks[i, 0] - peaks[i - 1, 0] if i > 0 else np.nan
        out.append((peaks[i, 0], T, mx - mi, peaks[i, 1], mi - rest, peaks[i, 2]))
    return np.array(out, float), rest


def process(sim):
    d = os.path.join(BASE, sim)
    imu = sorted(glob.glob(os.path.join(d, "lsm6dsv_raw_*.txt")))[-1]
    t, acc = load_sensor(imu)
    pt, pv, peaks = load_platform(find_platform(sim))
    t_u, bp, av, fs_u = preprocess(t, acc)
    pk_idx, tp = find_positive_peaks(bp, fs_u)
    pcyc, rest = platform_cycles2(pt, pv, peaks)
    off = align(tp, pcyc[:, 0], onset_guess(t_u, bp, pt, pv))
    dt = 1.0 / fs_u

    cyc = []
    for k in range(len(tp) - 1):
        i0 = int(round(tp[k] * fs_u)); i1 = int(round(tp[k + 1] * fs_u))
        if i1 - i0 < int(0.30 * fs_u) or i1 - i0 > int(1.2 * fs_u):
            continue
        T = tp[k + 1] - tp[k]
        cyc.append((0.5 * (tp[k] + tp[k + 1]), T, dft_travel(bp[i0:i1], dt, T)))
    cyc = np.array(cyc, float)

    rows = []
    for j in range(len(pcyc)):
        k = int(np.argmin(np.abs(cyc[:, 0] - (pcyc[j, 0] + off))))
        if abs(cyc[:, 0][k] - (pcyc[j, 0] + off)) > 0.28:
            continue
        rows.append((pcyc[j, 0], pcyc[j, 1], pcyc[j, 2], pcyc[j, 3],
                     cyc[k, 1], cyc[k, 2], pcyc[j, 5]))
    return dict(sim=sim, rows=np.array(rows, float), off=off, rest=rest,
                tp=tp, bp=bp, fs_u=fs_u, pt=pt, pv=pv, peaks=peaks, acc=acc, t=t)


def valid_mask(rows):
    """有效段：丢掉平台预热期与收尾塌陷的几次按压"""
    d = rows[:, 5]
    med = np.median(d)
    ok = d > 0.75 * med
    n = len(rows)
    lo = WARMUP
    while lo < n and not (ok[lo] and ok[min(lo + 1, n - 1)] and ok[min(lo + 2, n - 1)]):
        lo += 1
    hi = n - 1
    while hi > lo and not ok[hi]:
        hi -= 1
    m = np.zeros(n, bool)
    m[lo:hi + 1] = True
    return m


def trimmed(rows):
    """返回 (连续有效段的行, 起始下标)"""
    idx = np.where(valid_mask(rows))[0]
    if len(idx) == 0:
        return rows, 0
    return rows[idx[0]:idx[-1] + 1], idx[0]


def main():
    lines = []
    def say(s=""):
        print(s); lines.append(s)

    res = {s: process(s) for s in SIMS}

    # ---------------- 标定 ----------------
    ks = []
    for s in SIMS:
        r = res[s]["rows"]
        m = valid_mask(r)
        ks.append(r[m, 2].mean() / r[m, 5].mean())
    K = float(np.mean(ks))
    say("=" * 76)
    say("深度增益标定   travel = K * d_imu")
    for s, k in zip(SIMS, ks):
        say("   %s  K=%.4f" % (s, k))
    say("   统一取 K = %.4f   (三次实验离散 %.2f%%)" % (K, (max(ks) - min(ks)) / K * 100))
    say("")

    # ---------------- 误差 ----------------
    say("=" * 76)
    say("标定后深度误差")
    say("   %-6s %7s %10s %8s %12s %8s %12s %8s"
        % ("sim", "周期数", "MAE逐次", "MAPE", "MAE 5次均", "MAPE", "vs自报深度", "偏差"))
    for s in SIMS:
        r = res[s]["rows"]; m = valid_mask(r)
        r = r[m]
        y = r[:, 2]                       # 平台 travel
        x = K * r[:, 5]                   # IMU 标定后
        e = x - y
        xs, ys = movavg(x), movavg(y)
        e2 = xs - ys
        rep = r[:, 3]
        say("   %-6s %7d %7.2f mm %7.1f%% %9.2f mm %7.1f%% %9.2f mm %+7.1f"
            % (s, len(r), np.abs(e).mean(), (np.abs(e) / y).mean() * 100,
               np.abs(e2).mean(), (np.abs(e2) / ys).mean() * 100,
               np.abs(x - rep).mean(), (x - rep).mean()))
    say("")
    say("   相关性 / 平台偏置随时间的漂移")
    say("   %-6s %10s %10s %12s %12s"
        % ("sim", "r逐次", "r 5次均", "偏置起-止(mm)", "偏置斜率mm/次"))
    for s in SIMS:
        r = res[s]["rows"]; m = valid_mask(r); r = r[m]
        y = r[:, 2]; x = K * r[:, 5]
        xs, ys = movavg(x), movavg(y)
        off = r[:, 3] - r[:, 2]
        k = np.polyfit(np.arange(len(off)), off, 1)[0]
        say("   %-6s %10.3f %10.3f %5.1f -> %4.1f %12.3f"
            % (s, np.corrcoef(x, y)[0, 1], np.corrcoef(xs, ys)[0, 1],
               off[:3].mean(), off[-3:].mean(), k))
    say("")
    say("   平台自报深度 = travel + 手上静息载荷偏置：")
    for s in SIMS:
        r = res[s]["rows"]; m = valid_mask(r); r = r[m]
        say("      %s  travel %.1f, 自报 %.1f, 偏置 %.1f mm (%.0f%%)"
            % (s, r[:, 2].mean(), r[:, 3].mean(),
               (r[:, 3] - r[:, 2]).mean(),
               (r[:, 3] - r[:, 2]).mean() / r[:, 2].mean() * 100))

    # ---------------- 频率 ----------------
    say("")
    say("=" * 76)
    say("按压频率（平台上报值 mannequin_frequency 作为参考）")
    say("   %-6s %10s %10s %10s %10s %10s"
        % ("sim", "IMU 逐次", "IMU 5次", "平台上报", "逐次MAE", "5次MAE"))
    for s in SIMS:
        r = res[s]["rows"]; m = valid_mask(r); r = r[m]
        imu = 60.0 / r[:, 4]
        imu5 = 60.0 / movavg(r[:, 4])
        rep = r[:, 6]
        ok = np.isfinite(rep)
        say("   %-6s %10.1f %10.1f %10.1f %10.2f %10.2f"
            % (s, imu[ok].mean(), imu5[ok].mean(), rep[ok].mean(),
               np.abs(imu[ok] - rep[ok]).mean(), np.abs(imu5[ok] - rep[ok]).mean()))
    say("   平均偏差：")
    for s in SIMS:
        r = res[s]["rows"]; m = valid_mask(r); r = r[m]
        rep = r[:, 6]; ok = np.isfinite(rep)
        imu = 60.0 / r[:, 4]
        say("      %s  IMU %.1f cpm, 平台上报 %.1f cpm, 差 %+.2f cpm (%.2f%%)"
            % (s, imu[ok].mean(), rep[ok].mean(), imu[ok].mean() - rep[ok].mean(),
               (imu[ok].mean() - rep[ok].mean()) / rep[ok].mean() * 100))

    # ---------------- CSV ----------------
    with io.open(os.path.join(OUT, "cycles_final.csv"), "w", encoding="utf-8", newline="") as f:
        f.write("sim,plat_t,plat_T,plat_travel,plat_report,imu_T,imu_depth_raw,"
                "imu_depth_cal,imu_depth_cal_avg5,imu_cpm\n")
        for s in SIMS:
            r = res[s]["rows"]; m = valid_mask(r)
            x = K * r[:, 5]
            xa = movavg(x)
            for i in range(len(r)):
                f.write("%s,%.3f,%.3f,%.1f,%.1f,%.4f,%.2f,%.2f,%.2f,%.1f,%s\n"
                        % (s, r[i, 0], r[i, 1], r[i, 2], r[i, 3], r[i, 4],
                           r[i, 5], x[i], xa[i], 60.0 / r[i, 4],
                           "1" if m[i] else "0"))

    # ---------------- 图 1 ----------------
    fig, axes = plt.subplots(3, 1, figsize=(13, 9.6))
    for ax, s in zip(axes, SIMS):
        allr = res[s]["rows"]; mm = valid_mask(allr)
        r, i0 = trimmed(allr)
        idx = np.arange(len(allr))
        idxv = np.arange(i0, i0 + len(r))
        x = K * r[:, 5]
        ax.plot(idx, allr[:, 2], ".", color="0.75", ms=4, label="_")
        ax.plot(idxv, r[:, 2], "k.-", ms=5, lw=1.1, label="平台 travel（真值）")
        ax.plot(idxv, r[:, 3], color="0.55", ls="--", lw=1.0,
                label="平台自报深度（=travel+手压偏置）")
        ax.plot(idxv, x, color="tab:red", alpha=0.40, lw=0.8, label="IMU 逐次")
        ax.plot(idxv, movavg(x), color="tab:red", lw=1.9, label="IMU 5 次平均")
        bad = ~mm
        ax.scatter(idx[bad], allr[bad, 2], s=40, facecolors="none", edgecolors="0.4",
                   marker="s", label="无效段（预热/收尾）")
        ax.set_ylabel("深度 (mm)")
        ax.set_title("%s   MAE %.1f mm / %.1f%%（5 次平均 %.1f mm）"
                     % (s, np.abs(x - r[:, 2]).mean(),
                        (np.abs(x - r[:, 2]) / r[:, 2]).mean() * 100,
                        np.abs(movavg(x) - movavg(r[:, 2])).mean()), fontsize=10)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, loc="lower right", ncol=2)
    axes[-1].set_xlabel("第几次按压")
    plt.tight_layout()
    plt.savefig(os.path.join(OUT, "fig1_depth_timeline.png"), dpi=110)
    plt.close(fig)

    # ---------------- 图 2 散点 ----------------
    fig, axes = plt.subplots(1, 2, figsize=(12.5, 5.6))
    cols = {"sim01": "tab:blue", "sim02": "tab:orange", "sim03": "tab:green"}
    for s in SIMS:
        r, _ = trimmed(res[s]["rows"])
        axes[0].scatter(r[:, 2], K * r[:, 5], s=24, c=cols[s], alpha=0.7, label=s)
    lim = [38, 62]
    axes[0].plot(lim, lim, "k--", lw=0.9, label="y = x")
    axes[0].set_xlim(lim); axes[0].set_ylim(lim)
    axes[0].set_xlabel("平台 travel (mm)"); axes[0].set_ylabel("IMU 深度 × 0.96 (mm)")
    axes[0].set_title("逐次按压深度")
    axes[0].grid(alpha=0.3); axes[0].legend(fontsize=8)
    for s in SIMS:
        r, _ = trimmed(res[s]["rows"])
        axes[1].scatter(movavg(r[:, 2]), movavg(K * r[:, 5]), s=24, c=cols[s],
                        alpha=0.7, label=s)
    axes[1].plot(lim, lim, "k--", lw=0.9)
    axes[1].set_xlim(lim); axes[1].set_ylim(lim)
    axes[1].set_xlabel("平台 travel 5 次平均 (mm)"); axes[1].set_ylabel("IMU 5 次平均 (mm)")
    axes[1].set_title("5 次滑动平均后（设备实际显示的口径）")
    axes[1].grid(alpha=0.3); axes[1].legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(os.path.join(OUT, "fig2_scatter.png"), dpi=115)
    plt.close(fig)

    # ---------------- 图 3 频率 ----------------
    fig, axes = plt.subplots(1, 2, figsize=(12.5, 4.3))
    allp, alli = [], []
    for s in SIMS:
        r, _ = trimmed(res[s]["rows"])
        imu = 60.0 / r[:, 4]
        rep = r[:, 6]
        axes[0].plot(np.arange(len(r)), imu, lw=1.2, color=cols[s], label=s)
        axes[0].plot(np.arange(len(rep)), rep, ls="--", lw=1.0, color=cols[s], alpha=0.55)
        ok = np.isfinite(rep)
        allp += list(rep[ok]); alli += list(imu[ok])
    allp = np.array(allp); alli = np.array(alli)
    axes[0].set_xlabel("第几次按压"); axes[0].set_ylabel("频率 (cpm)")
    axes[0].set_title("按压频率：实线 IMU，虚线平台上报")
    axes[0].grid(alpha=0.3); axes[0].legend(fontsize=8)
    axes[1].scatter(allp, alli, s=13, c="tab:purple", alpha=0.65)
    lim = [allp.min() - 3, allp.max() + 3]
    axes[1].plot(lim, lim, "k--", lw=0.9)
    axes[1].set_xlabel("平台上报 (cpm)"); axes[1].set_ylabel("IMU (cpm)")
    axes[1].set_title("逐次 MAE %.2f cpm；均值偏差 %+.2f cpm"
                      % (np.abs(alli - allp).mean(), (alli - allp).mean()))
    axes[1].grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(OUT, "fig3_rate.png"), dpi=115)
    plt.close(fig)

    # ---------------- 图 4 RMS 为何失效 ----------------
    fig, axes = plt.subplots(1, 2, figsize=(12.5, 4.3))
    for s in SIMS:
        r = res[s]; tp = r["tp"]; bp = r["bp"]; fs = r["fs_u"]
        m = valid_mask(res[s]["rows"])
        idxs = np.where(m)[0]
        accs = []
        for j in idxs[5:-3]:
            k = int(np.argmin(np.abs(tp - res[s]["rows"][j, 0])))
            i0 = int(round(tp[k] * fs)); i1 = int(round(tp[k + 1] * fs))
            if i1 - i0 < 20:
                continue
            T = tp[k + 1] - tp[k]
            seg = bp[i0:i1]
            accs.append(np.interp(np.linspace(0, 1, 200),
                                  np.arange(len(seg)) * (1.0 / fs) / T, seg))
        mm = np.mean(accs, axis=0)
        axes[0].plot(np.linspace(0, 1, 200), mm / np.abs(mm).max(), lw=1.7,
                     color=cols[s], label=s)
    axes[0].plot(np.linspace(0, 1, 200),
                 np.sin(2 * np.pi * np.linspace(0, 1, 200)), "k--", lw=1.0, label="纯正弦")
    axes[0].set_xlabel("归一化周期"); axes[0].set_ylabel("竖直加速度（归一化）")
    axes[0].set_title("一个按压周期的加速度波形：远不是正弦")
    axes[0].grid(alpha=0.3); axes[0].legend(fontsize=8)

    hh = []
    for s in SIMS:
        r = res[s]; tp = r["tp"]; bp = r["bp"]; fs = r["fs_u"]
        amps = []
        for k in range(6, len(tp) - 6):
            i0 = int(round(tp[k] * fs)); i1 = int(round(tp[k + 1] * fs))
            T = tp[k + 1] - tp[k]; n = i1 - i0
            t = np.arange(n) / fs; w = 2 * np.pi / T
            A = []
            for h in range(1, 6):
                S = np.dot(bp[i0:i1], np.sin(h * w * t))
                C = np.dot(bp[i0:i1], np.cos(h * w * t))
                A.append(2.0 / n * np.hypot(S, C))
            amps.append(A)
        amps = np.array(amps)
        hh.append(amps.mean(axis=0) / amps.mean(axis=0)[0])
    hh = np.array(hh)
    x = np.arange(1, 6); wbar = 0.26
    for i, s in enumerate(SIMS):
        axes[1].bar(x + (i - 1) * wbar, hh[i], width=wbar, color=cols[s], label=s)
    axes[1].set_xlabel("谐波次数"); axes[1].set_ylabel("幅度 / 基波")
    axes[1].set_title("竖直加速度的谐波含量（决定 RMS 法误差）")
    axes[1].grid(alpha=0.3); axes[1].legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(os.path.join(OUT, "fig4_why_rms_fails.png"), dpi=115)
    plt.close(fig)

    # ---------------- 图 5 波形对照 ----------------
    s = "sim03"
    r = res[s]; tp = r["tp"]; bp = r["bp"]; fs = r["fs_u"]
    k0 = len(tp) // 2
    tt_all, dd_all = [], []
    for k in range(k0, k0 + 5):
        ia = int(round(tp[k] * fs)); ib = int(round(tp[k + 1] * fs))
        T = tp[k + 1] - tp[k]; w = 2 * np.pi / T; n = ib - ia
        t = np.arange(n) / fs
        S = np.dot(bp[ia:ib], np.sin(w * t)); C = np.dot(bp[ia:ib], np.cos(w * t))
        A = 2.0 / n * np.hypot(S, C); ph = np.arctan2(C, S)
        tt_all.append(tp[k] + t)
        dd_all.append(-A * G / (w * w) * np.sin(w * t + ph))
    tt_all = np.concatenate(tt_all); dd_all = np.concatenate(dd_all)
    fig, axes = plt.subplots(2, 1, figsize=(12.5, 6.6))
    axes[0].plot(tt_all - tt_all[0], dd_all, lw=1.5, color="tab:red")
    axes[0].set_ylabel("位移 (mm)")
    axes[0].set_title("IMU 重建的竖直位移（带通、直流已去除），连续 5 次按压")
    axes[0].grid(alpha=0.3)
    pt, pv, off = r["pt"], r["pv"], r["off"]
    m = (pt >= tp[k0] - off - 0.20) & (pt <= tp[k0 + 5] - off)
    axes[1].plot(pt[m] - pt[m][0], pv[m] - pv[m].mean(), "k.-", ms=4, lw=1.2)
    axes[1].set_ylabel("台面读数 (mm，去均值)"); axes[1].set_xlabel("时间 (s)")
    axes[1].set_title("同一段 5 次按压的平台波形（20 Hz，1 mm 量化）")
    axes[1].grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(OUT, "fig5_waveform.png"), dpi=110)
    plt.close(fig)

    with io.open(os.path.join(OUT, "fit_report.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("\nOK ->", OUT)


if __name__ == "__main__":
    main()
