# -*- coding: utf-8 -*-
"""
板级验证比对工具
================
三种用法：

1) 只看 PC 复刻（不用板子，先知道应该是什么样）
       python board_check.py sim03
       python board_check.py imu_process_03/sim03/lsm6dsv_raw_*.txt

2) 比对板子回放日志（board_replay.py 会自动存盘）
       python board_check.py sim03 board_logs/board_sim03.txt

3) 只解析日志
       python board_check.py - board_logs/board_sim03.txt

做了什么：
    * 用和新固件**完全相同**的漏积分逻辑在 PC 上复刻一遍（EVL 行）
    * 读板子回传的 EV（旧方案 dft1）与 EVL（新方案漏积分）
    * 三者都和平台 travel 比，出表 + 出图

协议（新固件）：
    EV,<序号>,<板载tick>,<间隔ms>,<深度mm>,<频率>,<数据时间ms>
    EVL,<序号>,<数据时间ms>,<本次深度mm>,<2次平均mm>
    DV,<数据时间ms>,<位移mm>,<速度mm/s>          （CPR_DEBUG_DISP=1 时）
"""

import os
import re
import sys
import io
import csv
import glob
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "fit_work", "out")
if not os.path.isdir(OUT):
    os.makedirs(OUT)

# ---------------- 与固件一致的常数 ----------------
ACC_LSB = 0.122          # mg/LSB
LEAK_HZ = 0.5            # CPR_LEAK_HZ
DEPTH_K = 0.914          # CPR_DEPTH2_K
MIN_DEPTH2 = 6.0         # CPR_MIN_DEPTH_MM
MIN_INTERVAL = 250.0     # CPR_MIN_INTERVAL_S * 1000
MG2MMS2 = 9.80665
DT_DEFAULT = 1.0 / 120.0


# ----------------------------------------------------------------------
#  数据读取
# ----------------------------------------------------------------------
def read_raw(path):
    """读 IMU 原始采集文件 -> t_ms(int), acc(mg), 原始行"""
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
            t.append(v[0])
            acc.append(v[1:4])
    return np.array(t, float), np.array(acc, float) * ACC_LSB


def read_platform(csv_path):
    """平台 csv -> (波形 t, 波形 mm, 峰值 t, 峰值 深度mm, 峰值 频率)"""
    wt, wv, pk = [], [], []
    with io.open(csv_path, encoding="utf-8-sig", errors="replace") as f:
        for r in csv.DictReader(f):
            if r["source"] == "mannequin" and r["mannequin_raw"]:
                try:
                    wt.append(float(r["elapsed_s"]))
                    wv.append(float(r["mannequin_raw"]))
                except ValueError:
                    pass
            elif r["source"] == "mannequin_peak" and r["mannequin_depth"]:
                try:
                    pk.append((float(r["elapsed_s"]), float(r["mannequin_depth"]) * 10.0,
                               float(r["mannequin_frequency"]) if r["mannequin_frequency"] else np.nan))
                except ValueError:
                    pass
    o = np.argsort(wt)
    wt = np.array(wt)[o]; wv = np.array(wv)[o]
    pk.sort()
    return wt, wv, np.array(pk, float)


def platform_travel(wt, wv, peaks):
    """每个平台峰值对应一个周期的峰峰值（真值：行程）"""
    out = []
    rest = np.median(wv[wt < 1.5]) if (wt < 1.5).sum() > 3 else wv.min()
    for i in range(len(peaks)):
        lo = peaks[i, 0] - (peaks[i, 0] - peaks[i - 1, 0] if i > 0 else 0.55)
        m = (wt >= lo) & (wt <= peaks[i, 0] + 0.15)
        if m.sum() < 4:
            out.append(np.nan); continue
        out.append(wv[m].max() - wv[m].min())
    return np.array(out), rest


# ----------------------------------------------------------------------
#  新固件逻辑的 PC 复刻（逐行对应 cpr_leaky_step）
# ----------------------------------------------------------------------
def replica_leaky(t_ms, acc_mg):
    """逐行对应 imu_process.c：

        dt 由 t_ms 差分（<=0 或 >50 ms 时按 1/120 保护）
        重力方向用一阶低通（tau = 1 s），首帧直接取 a/|a|
        av = a·u - 1000
        v += (av*9.80665 - w0*v) * dt
        d += (v - w0*d) * dt
        累计 hi/lo，速度由负变正时出 depth = K*(hi-lo)
    """
    t_ms = np.asarray(t_ms, float)
    n_s = len(t_ms)
    w0 = 6.2831853 * LEAK_HZ
    v = 0.0
    d = 0.0
    hi = 0.0
    lo = 0.0
    ev = []
    prev = 0.0
    ncnt = 0
    t_last = -1e9
    disp = np.empty(n_s)
    vel = np.empty(n_s)
    av_out = np.empty(n_s)
    g = None
    t_prev = None

    for i in range(n_s):
        if t_prev is None:
            dt = DT_DEFAULT
        else:
            dt = (t_ms[i] - t_prev) / 1000.0
            if dt <= 0.0 or dt > 0.05:
                dt = DT_DEFAULT
        t_prev = t_ms[i]

        x, y, z = acc_mg[i]
        amag = math.sqrt(x * x + y * y + z * z)
        if amag < 100.0:
            disp[i] = d
            vel[i] = v
            av_out[i] = 0.0
            continue

        # 1. 重力方向（一阶低通）
        if g is None:
            g = np.array([x / amag, y / amag, z / amag])
        else:
            alpha = dt / (1.0 + dt)          # gravity_tau_s = 1.0
            g = g + (np.array([x / amag, y / amag, z / amag]) - g) * alpha
            gn = math.sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2])
            if gn < 0.1:
                gn = 1.0
            g = g / gn

        # 2. 竖直加速度，去掉 1 g
        av = x * g[0] + y * g[1] + z * g[2] - 1000.0
        av_out[i] = av

        # 3. 漏积分两次
        a = av * MG2MMS2
        v_prev = v
        v += (a - w0 * v) * dt
        d += (v - w0 * d) * dt
        disp[i] = d
        vel[i] = v

        # 4. 累计极值 + 速度过零
        if d > hi:
            hi = d
        if d < lo:
            lo = d

        if (v_prev < 0.0) and (v >= 0.0):
            # 速度由负变正 -> 刚过位移谷底（压到底）
            dpt = DEPTH_K * (hi - lo)
            if (dpt >= MIN_DEPTH2) and ((t_ms[i] - t_last) >= MIN_INTERVAL):
                if ncnt < 2:
                    avg = dpt
                    ncnt += 1
                else:
                    avg = 0.5 * (prev + dpt)
                prev = dpt
                ev.append((t_ms[i] / 1000.0, dpt, avg))
                t_last = t_ms[i]
            hi = d
            lo = d
    return np.array(ev, float), disp, vel, av_out


# ----------------------------------------------------------------------
#  日志解析
# ----------------------------------------------------------------------
def parse_log(path):
    ev, evl, dv = [], [], []
    with io.open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            s = line.strip()
            if not s or "," not in s:
                continue
            p = s.split(",")
            try:
                if p[0] == "EV" and len(p) >= 6:
                    # EV,n,tick,interval,depth,rate[,data_t]
                    ev.append(((float(p[6]) if len(p) >= 7 else float(p[2])) / 1000.0,
                               float(p[4]), float(p[5])))
                elif p[0] == "EVL" and len(p) >= 5:
                    evl.append((float(p[2]) / 1000.0, float(p[3]), float(p[4])))
                elif p[0] == "DV" and len(p) >= 4:
                    dv.append((float(p[1]) / 1000.0, float(p[2]), float(p[3])))
            except (ValueError, IndexError):
                continue
    return np.array(ev, float), np.array(evl, float), np.array(dv, float)


# ----------------------------------------------------------------------
#  对齐 + 打分
# ----------------------------------------------------------------------
def movavg(x, n=5):
    x = np.asarray(x, float)
    if len(x) < n:
        return x.copy()
    pad = n // 2
    xp = np.concatenate([np.repeat(x[0], pad), x, np.repeat(x[-1], pad)])
    return np.convolve(xp, np.ones(n) / n, mode="same")[pad:pad + len(x)]


def match(ev_t, ev_v, peak_t, ref):
    """把事件序列对到平台峰值序列。

    注意：时间轴本身没有共同起点，所以"错几个周期"无法用时间定，
    这里改用 **5 次平滑后的深度序列相关性** 来定整数错位
    （深度在整段按压里是缓慢变化的，这个趋势就是天然的对齐指纹）。
    """
    ev_t = np.asarray(ev_t, float)
    ev_v = np.asarray(ev_v, float)
    ok = np.isfinite(ev_t) & np.isfinite(ev_v)
    ev_t, ev_v = ev_t[ok], ev_v[ok]
    n = min(len(ev_t), len(peak_t))
    if n < 10:
        return None

    best = (-2.0, 0, 0.0)
    for s in (-4, -3, -2, -1, 0, 1, 2, 3, 4):
        if s >= 0:
            a = ev_v[:n - s] if s > 0 else ev_v[:n]
            b = ref[s:n]
            ta = ev_t[:n - s] if s > 0 else ev_t[:n]
            tb = peak_t[s:n]
        else:
            a = ev_v[-s:n]
            b = ref[0:n + s]
            ta = ev_t[-s:n]
            tb = peak_t[0:n + s]
        m = min(len(a), len(b))
        if m < 10:
            continue
        a2, b2 = a[:m], b[:m]
        ok2 = np.isfinite(a2) & np.isfinite(b2)
        a2, b2 = a2[ok2], b2[ok2]
        if len(a2) < 10:
            continue
        aa, bb = movavg(a2, 5), movavg(b2, 5)
        r = np.corrcoef(aa, bb)[0, 1]
        off = np.median(ta[:m][ok2] - tb[:m][ok2])
        print("      [对齐候选] 错位 %+d 周期: 5次平滑相关 r=%+.3f  时差 %+.2f s"
              % (s, r, off))
        if r > best[0]:
            best = (r, s, off)
    r, s, off = best
    est, gt = [], []
    for j in range(len(peak_t)):
        k = j - s
        if 0 <= k < len(ev_v):
            est.append(ev_v[k]); gt.append(ref[j])
    return np.array(gt), np.array(est), off, s


def score(gt, est):
    ok = np.isfinite(gt) & np.isfinite(est)
    gt, est = gt[ok], est[ok]
    if len(gt) < 8:
        return None
    # 去掉开头 6 次（平台预热）和末尾 2 次
    gt, est = gt[6:-2], est[6:-2]
    K = gt.mean() / est.mean()
    e = K * est - gt
    return dict(K=K, mae=np.abs(e).mean(), mape=(np.abs(e) / gt).mean() * 100,
                r=np.corrcoef(est, gt)[0, 1], n=len(gt))


# ----------------------------------------------------------------------
def find_sim_dir(arg):
    """参数可以是 sim03 / 目录 / 原始 txt"""
    if os.path.isdir(arg):
        return arg
    cand = os.path.join(HERE, arg)
    if os.path.isdir(cand):
        return cand
    cand2 = os.path.join(HERE, "imu_process_03", arg)
    if os.path.isdir(cand2):
        return cand2
    return None


def load_sim(d):
    raw = sorted(glob.glob(os.path.join(d, "lsm6dsv_raw_*.txt")))
    if not raw:
        raise SystemExit("目录里找不到 lsm6dsv_raw_*.txt: %s" % d)
    raw = raw[-1]
    plat = os.path.join(d, "platform_raw.csv")
    if not os.path.exists(plat):
        c = [x for x in glob.glob(os.path.join(d, "*", "*.csv"))
             if "校验" not in os.path.basename(x)]
        if not c:
            raise SystemExit("找不到平台 csv: %s" % d)
        plat = c[0]
    return raw, plat


def main():
    args = [a for a in sys.argv[1:]]
    log_path = None
    sim_arg = None
    if len(args) >= 2:
        sim_arg, log_path = args[0], args[1]
    elif len(args) == 1:
        if args[0].lower().endswith((".txt", ".log")) and "raw" not in os.path.basename(args[0]):
            log_path = args[0]
        else:
            sim_arg = args[0]
    if sim_arg in (None, "-"):
        sim_arg = "sim03"
    d = find_sim_dir(sim_arg)
    if d is None:
        raise SystemExit("找不到 sim 目录: %s（在 %s 下）" % (sim_arg, HERE))
    raw, plat = load_sim(d)
    print("IMU  :", os.path.basename(raw))
    print("平台 :", os.path.basename(plat))
    if log_path:
        print("板子 :", log_path)

    t_ms, acc = read_raw(raw)
    wt, wv, peaks = read_platform(plat)
    ref, rest = platform_travel(wt, wv, peaks)

    ev_rep, disp, vel, av = replica_leaky(t_ms, acc)
    print("\nPC 复刻（漏积分）：%d 次按压" % len(ev_rep))
    if len(ev_rep):
        print("   深度 平均 %.1f mm, 范围 %.0f~%.0f mm"
              % (ev_rep[:, 1].mean(), ev_rep[:, 1].min(), ev_rep[:, 1].max()))

    rows = []
    m = match(ev_rep[:, 0], ev_rep[:, 1], peaks[:, 0], ref)
    if m:
        gt, est, off, s = m
        sc = score(gt, est)
        print("   对齐: 偏移 %+.3f s, 整数错位 %+d" % (off, s))
        print("   vs 平台 travel: MAE %.2f mm (%.1f%%), r=%.3f, K=%.3f"
              % (sc["mae"], sc["mape"], sc["r"], sc["K"]))
        rows.append(("PC复刻 EVL", sc, gt, sc["K"] * est))

    board_ev = board_evl = None
    if log_path:
        if not os.path.exists(log_path):
            print("\n[警告] 日志不存在: %s" % log_path)
        else:
            board_ev, board_evl, dv = parse_log(log_path)
            print("\n板子日志：EV %d 条, EVL %d 条, DV %d 条"
                  % (len(board_ev), len(board_evl), len(dv)))
            if len(board_evl):
                m = match(board_evl[:, 0], board_evl[:, 1], peaks[:, 0], ref)
                if m:
                    gt, est, off, s = m
                    sc = score(gt, est)
                    print("   EVL vs 平台 travel: MAE %.2f mm (%.1f%%), r=%.3f, K=%.3f"
                          % (sc["mae"], sc["mape"], sc["r"], sc["K"]))
                    rows.append(("板子 EVL(新)", sc, gt, sc["K"] * est))
            if len(board_ev):
                m = match(board_ev[:, 0], board_ev[:, 1], peaks[:, 0], ref)
                if m:
                    gt, est, off, s = m
                    sc = score(gt, est)
                    print("   EV  vs 平台 travel: MAE %.2f mm (%.1f%%), r=%.3f, K=%.3f"
                          % (sc["mae"], sc["mape"], sc["r"], sc["K"]))
                    rows.append(("板子 EV(旧)", sc, gt, sc["K"] * est))
            # 板子 vs 复刻：按时间最近匹配（两边都是同一份数据的数据时间轴，
            # 不能按序号硬对，因为开头可能多/少一个事件，后面会整体错位）
            if len(board_evl) and len(ev_rep):
                rep_t = ev_rep[:, 0]
                rep_v = ev_rep[:, 1]
                # 只比较"按压开始之后"的事件：前 2 s 是漏积分收敛期，
                # 板子和 PC 的浮点/重力初值差异在这里会被放大，没有比较意义
                t_begin = peaks[0, 0] - 0.3 if len(peaks) else 2.0
                diffs = []
                used = set()
                for (bt, bv, ba) in board_evl:
                    if bt < t_begin:
                        continue
                    k = int(np.argmin(np.abs(rep_t - bt)))
                    if abs(rep_t[k] - bt) > 0.05 or k in used:
                        continue
                    used.add(k)
                    diffs.append(bv - rep_v[k])
                if diffs:
                    diffs = np.array(diffs)
                    print("   板子 EVL vs PC 复刻：匹配 %d 次，最大差 %.2f mm，"
                          "平均差 %.2f mm" % (len(diffs), np.max(np.abs(diffs)),
                                              np.mean(np.abs(diffs))))
                    if np.max(np.abs(diffs)) > 0.5:
                        print("   [注意] 差异超过 0.5 mm，检查是否烧的是新固件、"
                              "以及采样率/量程设置是否一致")
                else:
                    print("   板子 EVL vs PC 复刻：没有能按时间对上的事件")

    if not rows:
        print("\n没有可比对的结果")
        return

    print("\n%-14s %8s %8s %8s %7s %6s" %
          ("序列", "MAE/mm", "MAPE", "r", "K", "样本"))
    for name, sc, gt, est in rows:
        print("%-14s %8.2f %7.1f%% %8.3f %7.3f %6d"
              % (name, sc["mae"], sc["mape"], sc["r"], sc["K"], sc["n"]))

    # ---------------- 出图 ----------------
    fig, axes = plt.subplots(2, 1, figsize=(13, 7.6),
                             gridspec_kw={"height_ratios": [2.2, 1]})
    gt = rows[0][2]
    idx = np.arange(len(gt))
    axes[0].plot(idx, gt, "k.-", ms=5, lw=1.2, label="平台 travel（真值）")
    colors = {"PC复刻 EVL": "tab:blue", "板子 EVL(新)": "tab:cyan",
              "板子 EV(旧)": "tab:red"}
    for name, sc, g, est in rows:
        axes[0].plot(np.arange(len(est)), est, lw=1.4, alpha=0.85,
                     color=colors.get(name, None),
                     label="%s  MAE %.2f" % (name, sc["mae"]))
        axes[1].plot(np.arange(len(est)), est - gt[:len(est)], lw=1.2,
                     alpha=0.85, color=colors.get(name, None), label=name)
    axes[0].set_ylabel("深度 (mm)")
    axes[0].set_title("%s  逐次按压深度对比" % os.path.basename(d))
    axes[0].grid(alpha=0.3); axes[0].legend(fontsize=9)
    axes[1].axhline(0, color="k", lw=0.8)
    axes[1].set_xlabel("第几次按压"); axes[1].set_ylabel("误差 (mm)")
    axes[1].grid(alpha=0.3); axes[1].legend(fontsize=8)
    plt.tight_layout()
    tag = "board" if log_path else "replica"
    png = os.path.join(OUT, "check_%s_%s.png" % (os.path.basename(d), tag))
    plt.savefig(png, dpi=115)
    plt.close(fig)
    print("\n图: %s" % png)

    # ---------------- 位移波形（复刻的） ----------------
    fig, ax = plt.subplots(figsize=(12, 4))
    t0 = t_ms[0] / 1000.0
    ax.plot(t_ms / 1000.0 - t0, disp, lw=1.0, color="tab:blue")
    for (tt, dd, aa) in ev_rep:
        ax.plot(tt / 1000.0 - t0, 0, "rv", ms=4)
    ax.set_xlabel("时间 (s)"); ax.set_ylabel("位移 (mm)")
    ax.set_title("PC 复刻的漏积分位移（红三角 = 检测到的压到底）")
    ax.grid(alpha=0.3)
    plt.tight_layout()
    png2 = os.path.join(OUT, "check_%s_wave.png" % os.path.basename(d))
    plt.savefig(png2, dpi=110)
    plt.close(fig)
    print("图: %s" % png2)


if __name__ == "__main__":
    main()
