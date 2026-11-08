# -*- coding: utf-8 -*-
"""
核对：+13 mm 到底是"算法需要"还是"口径换算"
=============================================
比较三个量：
    平台 travel          一个周期内的峰峰值（真正的行程）
    平台自报 depth       谷底 - 最初静置（含手压静息偏置）
    板子 EV 的原始输出   旧方法 depth = gain*d_fit (+13)
    板子 EVL 的原始输出  新方法 K*(峰-谷)，没有加法项
"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import board_check as B

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

LOGS = [("sim01", "board_sim01_20261105_224043.txt"),
        ("sim02", "board_sim02_20261105_224121.txt"),
        ("sim03", "board_sim03_20261105_224156.txt")]

print("%-6s %9s %9s %9s %9s %9s %9s"
      % ("sim", "平台行程", "平台自报", "自报-行程", "EV原始", "EV-行程", "EVL原始"))
for sim, log in LOGS:
    raw, plat = B.load_sim(os.path.join(HERE, sim))
    wt, wv, pk = B.read_platform(plat)
    ref, rest = B.platform_travel(wt, wv, pk)
    ev, evl, dv = B.parse_log(os.path.join(HERE, "board_logs", log))

    # 平台自报 = mannequin_depth（cm->mm 已在 parse 里乘 10）
    rep = pk[:, 1]

    m = (np.isfinite(ref) & np.isfinite(rep))
    travel = np.nanmean(ref[m][6:-3])
    report = np.nanmean(rep[m][6:-3])

    ev_mean = np.nanmean(ev[:, 1][6:-3])
    evl_mean = np.nanmean(evl[:, 1][6:-3])

    print("%-6s %9.1f %9.1f %+9.1f %9.1f %+9.1f %9.1f"
          % (sim, travel, report, report - travel, ev_mean, ev_mean - travel, evl_mean))

print("")
print("如果 EVL 也想报平台的'从头算'口径，需要加的常数 = 平台自报 - 平台行程：")
tot = []
for sim, log in LOGS:
    raw, plat = B.load_sim(os.path.join(HERE, sim))
    wt, wv, pk = B.read_platform(plat)
    ref, rest = B.platform_travel(wt, wv, pk)
    rep = pk[:, 1]
    m = (np.isfinite(ref) & np.isfinite(rep))
    tot.append(np.nanmean(rep[m][6:-3]) - np.nanmean(ref[m][6:-3]))
    print("   %s  +%.1f mm" % (sim, tot[-1]))
print("   平均 +%.1f mm，三次之间抖 %.1f mm" % (np.mean(tot), max(tot) - min(tot)))
