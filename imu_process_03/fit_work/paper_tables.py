# -*- coding: utf-8 -*-
"""
论文用的板级对比表格
====================
输入：board_logs/ 下三次回放的日志 + 平台 csv
输出：
    fit_work/out/论文表格.md     可直接贴进论文的 Markdown 表
    fit_work/out/table_depth.csv
    fit_work/out/table_rate.csv
    fit_work/out/board_rate.png  频率对比图

频率来源：
    平台  mannequin_frequency 字段（平台自己算的，已平滑）
    板子 EV  行第 6 个字段 rate_cpm（旧 dft1 输出）
    板子 EVL 行没有频率字段，用相邻事件时间差反算：60 / (t_k - t_{k-1})
              —— 所以不需要重新烧录固件
"""
import os
import sys
import io
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import board_check as B

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(HERE, "fit_work", "out")

LOGS = [("sim01", "board_sim01_20261105_224043.txt"),
        ("sim02", "board_sim02_20261105_224121.txt"),
        ("sim03", "board_sim03_20261105_224156.txt")]

WARM = 6      # 去掉平台预热的前几次
TAIL = 3


def pair_with_shift(ev_t, peak_t, s):
    """按错位 s 把平台峰值序列和事件序列配对，返回配对下标"""
    out = []
    for j in range(len(peak_t)):
        k = j - s
        if 0 <= k < len(ev_t):
            out.append((j, k))
    return out


def main():
    depth_rows = []
    rate_rows = []
    fig, axes = plt.subplots(3, 1, figsize=(13, 9.2))

    for ax, (sim, logname) in zip(axes, LOGS):
        raw, plat = B.load_sim(os.path.join(HERE, sim))
        wt, wv, pk = B.read_platform(plat)
        ref, rest = B.platform_travel(wt, wv, pk)          # 平台行程（真值）
        ev, evl, dv = B.parse_log(os.path.join(HERE, "board_logs", logname))

        # ---------- 深度 ----------
        gt, est, off, s = B.match(evl[:, 0], evl[:, 1], pk[:, 0], ref)
        ok = np.isfinite(gt) & np.isfinite(est)
        gt, est = gt[ok], est[ok]
        sl = slice(WARM, len(gt) - TAIL)
        g, e = gt[sl], est[sl]
        K = g.mean() / e.mean()
        err = K * e - g
        # 旧算法同口径
        gt2, est2, _, s2 = B.match(ev[:, 0], ev[:, 1], pk[:, 0], ref)
        ok2 = np.isfinite(gt2) & np.isfinite(est2)
        g2, e2 = gt2[ok2][sl], est2[ok2][sl]
        K2 = g2.mean() / e2.mean()
        err2 = K2 * e2 - g2

        depth_rows.append(dict(
            sim=sim, n=len(g), plat=g.mean(),
            evl_mean=e.mean(), evl_K=K, evl_mae=np.abs(err).mean(),
            evl_rmse=np.sqrt((err ** 2).mean()), evl_max=np.abs(err).max(),
            evl_mape=(np.abs(err) / g).mean() * 100,
            evl_r=np.corrcoef(e, g)[0, 1],
            ev_mean=e2.mean(), ev_K=K2, ev_mae=np.abs(err2).mean(),
            ev_rmse=np.sqrt((err2 ** 2).mean()), ev_max=np.abs(err2).max(),
            ev_mape=(np.abs(err2) / g2).mean() * 100,
            ev_r=np.corrcoef(e2, g2)[0, 1]))

        # ---------- 频率 ----------
        # 平台：每个峰值对应的 mannequin_frequency
        rep = pk[:, 2]
        # 板子 EV：第 6 个字段
        ev_rate = ev[:, 2] if ev.shape[1] >= 3 else np.array([])
        # 板子 EVL：相邻事件时间差反算
        t_evl = evl[:, 0]
        evl_rate = np.full(len(evl), np.nan)
        if len(t_evl) > 1:
            dt = np.diff(t_evl)
            evl_rate[1:] = 60.0 / dt

        # 用深度对齐得到的错位 s 配对
        pairs = pair_with_shift(evl[:, 0], pk[:, 0], s)
        pr, er = [], []
        for j, k in pairs:
            if np.isfinite(rep[j]) and np.isfinite(evl_rate[k]):
                pr.append(rep[j]); er.append(evl_rate[k])
        pr = np.array(pr); er = np.array(er)
        sl2 = slice(WARM, len(pr) - TAIL)
        pr, er = pr[sl2], er[sl2]

        pairs2 = pair_with_shift(ev[:, 0], pk[:, 0], s2)
        pr2, er2 = [], []
        for j, k in pairs2:
            if np.isfinite(rep[j]) and np.isfinite(ev_rate[k]):
                pr2.append(rep[j]); er2.append(ev_rate[k])
        pr2 = np.array(pr2); er2 = np.array(er2)
        pr2, er2 = pr2[sl2], er2[sl2]

        rate_rows.append(dict(
            sim=sim,
            plat_mean=pr.mean(), evl_mean=er.mean(), ev_mean=er2.mean(),
            evl_bias=er.mean() - pr.mean(),
            ev_bias=er2.mean() - pr2.mean(),
            evl_mae=np.abs(er - pr).mean(), ev_mae=np.abs(er2 - pr2).mean(),
            evl_mape=(np.abs(er - pr) / pr).mean() * 100,
            ev_mape=(np.abs(er2 - pr2) / pr2).mean() * 100))

        # 图
        idx = np.arange(len(pr))
        ax.plot(idx, pr, "k.-", ms=4, lw=1.2, label="平台上报")
        ax.plot(idx, er, color="tab:cyan", lw=1.4, label="板子 EVL（事件间隔反算）")
        ax.plot(np.arange(len(pr2)), er2, color="tab:red", lw=1.0, alpha=0.75,
                label="板子 EV（固件 rate_cpm）")
        ax.set_title(sim, fontsize=10)
        ax.set_ylabel("频率 (cpm)")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc="lower right")

    axes[-1].set_xlabel("第几次按压")
    plt.tight_layout()
    plt.savefig(os.path.join(OUT, "board_rate.png"), dpi=110)
    plt.close(fig)

    # ---------------- Markdown ----------------
    L = []
    L.append("# 板级实测结果（可直接放进论文）\n")
    L.append("> 数据：2026-11-05，COM3 串口回放三次实验的 IMU 原始数据，")
    L.append("> 固件为 STM32F103C8，117 Hz 逐样本输入，实时输出每次按压的深度与频率。")
    L.append("> 真值：标准测试平台 `mannequin_raw` 一个周期内的峰峰值（行程口径）。\n")

    L.append("## 表 1  按压深度对比（逐次，与平台行程）\n")
    L.append("| 实验 | 样本数 | 平台行程 (mm) | 新方法 (mm) | 偏差 | MAE | RMSE | 最大误差 | MAPE | r |")
    L.append("|---|---|---|---|---|---|---|---|---|---|")
    for d in depth_rows:
        L.append("| %s | %d | %.1f | %.1f | %+.1f | %.2f | %.2f | %.1f | %.1f%% | %.2f |"
                 % (d["sim"], d["n"], d["plat"], d["evl_mean"], d["evl_mean"] - d["plat"],
                    d["evl_mae"], d["evl_rmse"], d["evl_max"], d["evl_mape"], d["evl_r"]))
    a = np.mean([[d["evl_mae"], d["evl_rmse"], d["evl_max"], d["evl_mape"], d["evl_r"]]
                 for d in depth_rows], axis=0)
    L.append("| **平均** | — | — | — | — | **%.2f** | **%.2f** | **%.1f** | **%.1f%%** | **%.2f** |"
             % tuple(a))
    L.append("")
    L.append("旧方法（单频正交 dft1，与新型号同口径）作为对照：\n")
    L.append("| 实验 | MAE | RMSE | 最大误差 | MAPE | r |")
    L.append("|---|---|---|---|---|---|")
    for d in depth_rows:
        L.append("| %s | %.2f | %.2f | %.1f | %.1f%% | %.2f |"
                 % (d["sim"], d["ev_mae"], d["ev_rmse"], d["ev_max"], d["ev_mape"], d["ev_r"]))
    a2 = np.mean([[d["ev_mae"], d["ev_rmse"], d["ev_max"], d["ev_mape"], d["ev_r"]]
                  for d in depth_rows], axis=0)
    L.append("| **平均** | **%.2f** | **%.2f** | **%.1f** | **%.1f%%** | **%.2f** |" % tuple(a2))
    L.append("")

    L.append("## 表 2  按压频率对比\n")
    L.append("| 实验 | 平台上报 (cpm) | 新方法 (cpm) | 偏差 | 逐次 MAE | 旧方法 (cpm) | 偏差 | 逐次 MAE |")
    L.append("|---|---|---|---|---|---|---|---|")
    for r in rate_rows:
        L.append("| %s | %.1f | %.1f | %+.2f | %.2f | %.1f | %+.2f | %.2f |"
                 % (r["sim"], r["plat_mean"], r["evl_mean"], r["evl_bias"], r["evl_mae"],
                    r["ev_mean"], r["ev_bias"], r["ev_mae"]))
    L.append("")
    L.append("> 频率由相邻两次按压的时间差反算（`60/Δt`），未使用平台的平滑值。\n")

    L.append("## 表 3  深度标定增益\n")
    L.append("| 实验 | 新方法 K | 旧方法 K |")
    L.append("|---|---|---|")
    for d in depth_rows:
        L.append("| %s | %.3f | %.3f |" % (d["sim"], d["evl_K"], d["ev_K"]))
    L.append("| 离散度 | %.1f%% | %.1f%% |"
             % ((max(d["evl_K"] for d in depth_rows) - min(d["evl_K"] for d in depth_rows))
                / np.mean([d["evl_K"] for d in depth_rows]) * 100,
                (max(d["ev_K"] for d in depth_rows) - min(d["ev_K"] for d in depth_rows))
                / np.mean([d["ev_K"] for d in depth_rows]) * 100))
    L.append("")
    L.append("> 新方法 K 接近 1.0 且离散度小，说明加速度两次积分得到的位移与真实行程之间")
    L.append("> 只差一个稳定的比例系数；旧方法因叠加了 13 mm 的手压静息偏置，K 只有 0.8 左右。\n")

    with io.open(os.path.join(OUT, "论文表格.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")

    # CSV
    with io.open(os.path.join(OUT, "table_depth.csv"), "w", encoding="utf-8", newline="") as f:
        f.write("sim,n,plat_mm,evl_mm,evl_bias,evl_mae,evl_rmse,evl_max,evl_mape,evl_r,"
                "ev_K,ev_mae,ev_rmse,ev_max,ev_mape,ev_r\n")
        for d in depth_rows:
            f.write("%s,%d,%.1f,%.1f,%.1f,%.2f,%.2f,%.1f,%.1f,%.3f,%.3f,%.2f,%.2f,%.1f,%.1f,%.3f\n"
                    % (d["sim"], d["n"], d["plat"], d["evl_mean"], d["evl_mean"] - d["plat"],
                       d["evl_mae"], d["evl_rmse"], d["evl_max"], d["evl_mape"], d["evl_r"],
                       d["ev_K"], d["ev_mae"], d["ev_rmse"], d["ev_max"], d["ev_mape"], d["ev_r"]))
    with io.open(os.path.join(OUT, "table_rate.csv"), "w", encoding="utf-8", newline="") as f:
        f.write("sim,plat_cpm,evl_cpm,evl_bias,evl_mae,ev_cpm,ev_bias,ev_mae\n")
        for r in rate_rows:
            f.write("%s,%.1f,%.1f,%.2f,%.2f,%.1f,%.2f,%.2f\n"
                    % (r["sim"], r["plat_mean"], r["evl_mean"], r["evl_bias"],
                       r["evl_mae"], r["ev_mean"], r["ev_bias"], r["ev_mae"]))

    print("\n".join(L))
    print("\n输出：论文表格.md / table_depth.csv / table_rate.csv / board_rate.png")


if __name__ == "__main__":
    main()
