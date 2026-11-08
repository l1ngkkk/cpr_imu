# -*- coding: utf-8 -*-
"""
预测 imu_process_01（实机固件）烧录后的串口输出
================================================
用 PC 逐行复刻跑一遍 sim 的原始数据，按 imu_process_01/Core/Src/main.c
里新改的格式打印，烧之前就知道该看到什么。

用法： python fit_work/predict_output.py [sim03]
"""
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # imu_process_03
sys.path.insert(0, HERE)
import board_check as B

sim = sys.argv[1] if len(sys.argv) > 1 else "sim03"
raw, plat = B.load_sim(os.path.join(HERE, sim))
t_ms, a = B.read_raw(raw)

ev, disp, vel, av = B.replica_leaky(t_ms, a)

wt, wv, pk = B.read_platform(plat)
travel, rest = B.platform_travel(wt, wv, pk)

print("IMU  : %s" % os.path.basename(raw))
print("模拟 imu_process_01 的串口输出（前 12 行 + 末尾 3 行）：")
print("")
last = None
shown = 0
for k in range(len(ev)):
    tt, dep, avg = ev[k]
    t = int(round(tt * 1000.0))
    dt_ev = 0 if last is None else (t - last)
    rate = 0.0 if dt_ev == 0 else 60000.0 / dt_ev
    last = t
    if tt < 2.0:          # 前 2 s 是积分器收敛期，板子会照常打印，这里略过
        continue
    if shown < 12 or k >= len(ev) - 3:
        print("#%-3d  t=%lu ms  dt=%lu ms  depth=%5.1f mm  avg=%5.1f mm  rate=%3.0f cpm"
              % (k + 1, t, dt_ev, dep, avg, rate))
    shown += 1

print("")
print("共 %d 次按压" % len(ev))
d = ev[:, 1]
print("深度: 平均 %.1f mm, 范围 %.0f~%.0f mm" % (d.mean(), d.min(), d.max()))
print("对照: 平台行程 %.1f mm（新方法应该贴着这个数）"
      % np.nanmean(travel[6:-3]))
print("      平台自报 %.1f mm（含手压偏置，比上面高约 12 mm）"
      % np.nanmean(pk[:, 1][6:-3]))
