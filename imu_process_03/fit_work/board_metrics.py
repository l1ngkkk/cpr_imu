# -*- coding: utf-8 -*-
"""板级复测后：两种算法的误差对比（MAE / RMSE / 最大误差 / 所需增益）"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import board_check as B

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

LOGS = [("sim01", "board_logs/board_sim01_20261105_224043.txt"),
        ("sim02", "board_logs/board_sim02_20261105_224121.txt"),
        ("sim03", "board_logs/board_sim03_20261105_224156.txt")]

agg = {"EV": [], "EVL": []}
for sim, log in LOGS:
    raw, plat = B.load_sim(os.path.join(HERE, sim))
    wt, wv, pk = B.read_platform(plat)
    ref, rest = B.platform_travel(wt, wv, pk)
    ev, evl, dv = B.parse_log(os.path.join(HERE, log))
    for name, arr in (("EV", ev), ("EVL", evl)):
        m = B.match(arr[:, 0], arr[:, 1], pk[:, 0], ref)
        if not m:
            continue
        gt, est, off, s = m
        ok = np.isfinite(gt) & np.isfinite(est)
        gt, est = gt[ok][6:-2], est[ok][6:-2]
        K = gt.mean() / est.mean()
        e = K * est - gt
        agg[name].append((np.abs(e).mean(), np.sqrt((e ** 2).mean()),
                          np.max(np.abs(e)), K, len(gt)))

print("%-5s %6s %7s %7s %9s %6s" % ("序列", "样本", "MAE", "RMSE", "最大误差", "K"))
for name in ("EV", "EVL"):
    for (sim, _), row in zip(LOGS, agg[name]):
        print("%-5s %6d %7.2f %7.2f %9.1f %6.3f  (EVL)" % ((name + " ")[:5], row[4],
              row[0], row[1], row[2], row[3]) if name == "EVL" else
              "%-5s %6d %7.2f %7.2f %9.1f %6.3f" % (name + " " + sim, row[4],
              row[0], row[1], row[2], row[3]))

print("")
for name in ("EV", "EVL"):
    a = np.array(agg[name])
    print("%-5s 三次平均: MAE %.2f mm  RMSE %.2f mm  最大误差 %.1f mm  K %.3f~%.3f"
          % (name, a[:, 0].mean(), a[:, 1].mean(), a[:, 2].max(),
             a[:, 3].min(), a[:, 3].max()))
