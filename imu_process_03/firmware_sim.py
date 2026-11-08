# -*- coding: utf-8 -*-
"""
固件算法复刻（逐行对齐 imu_process.c 里的 CPR_Update）

目的：用采集到的原始数据，在 PC 上跑一遍和单片机完全相同的逻辑，
      看清每个半周是被"接受"还是"为什么被拒绝"，用来指导调参。

复刻的内容：
    - 重力方向：一阶低通（tau = 1.0 s），首帧直接取 a/|a|
    - av = a·u - 1000
    - 带通：两级二阶高通（fc = 1.0 Hz）+ 一阶二阶低通（fc = 8.0 Hz）
      双线性变换系数与 C 代码一致（Q = 0.7071）
    - 半周期检测：av < -hyst 开始，av > +hyst 结算
    - 迟滞：0.30 × a_rms，限幅 40~100 mg
    - 节奏门限：2×half >= 0.75 × 平均周期
    - 幅度门限：a_rms >= 0.5 × 近期平均 RMS（平均 <= 20 时不检查）
    - 深度：2√2·a_rms·9.80665/ω²，限幅 10~150 mm
    - 中值 3 + 一阶平滑 0.6/0.4；频率平滑 0.8/0.2
    - 空闲 3 s 复位（只在刚进入空闲时执行一次）
    - 预热 2 s

用法：
    python firmware_sim.py [数据文件.txt]
"""

import os
import sys
import glob
import math
import numpy as np

# ---------------- 固件参数（与 cpr_default_config 一致） ----------------
FS_DESIGN = 120.0      # 滤波器设计用的采样率
HP_HZ = 1.0
LP_HZ = 8.0
WARMUP_S = 2.0
MIN_DEPTH = 10.0
MAX_DEPTH = 150.0
GRAVITY_TAU = 1.0
GAIN = 1.0

SCALE = {"2g": 0.061, "4g": 0.122, "8g": 0.244, "16g": 0.488}


class Biquad(object):
    """转置直接 II 型，和 C 里的 cpr_biquad_run 一致"""

    def __init__(self, fs, fc, highpass):
        fc = min(fc, fs * 0.45)
        fc = max(fc, 0.05)
        w0 = 2.0 * math.pi * fc / fs
        cw = math.cos(w0)
        sw = math.sin(w0)
        alpha = sw / 1.4142136
        a0 = 1.0 + alpha

        if highpass:
            self.b0 = (1.0 + cw) * 0.5 / a0
            self.b1 = -(1.0 + cw) / a0
            self.b2 = self.b0
        else:
            self.b0 = (1.0 - cw) * 0.5 / a0
            self.b1 = (1.0 - cw) / a0
            self.b2 = self.b0

        self.a1 = -2.0 * cw / a0
        self.a2 = (1.0 - alpha) / a0
        self.z1 = 0.0
        self.z2 = 0.0

    def run(self, x):
        y = self.b0 * x + self.z1
        self.z1 = self.b1 * x - self.a1 * y + self.z2
        self.z2 = self.b2 * x - self.a2 * y
        return y


def find_file(argv):
    if len(argv) > 1:
        return argv[1]
    here = os.path.dirname(os.path.abspath(__file__))
    files = sorted(glob.glob(os.path.join(here, "lsm6dsv_raw_*.txt")))
    if not files:
        raise SystemExit("找不到 lsm6dsv_raw_*.txt，请传入文件名")
    return files[-1]


def read_capture(path):
    lsb = None
    t, acc, gyr = [], [], []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            s = line.strip()
            if not s:
                continue
            if s.startswith("#"):
                for k, v in SCALE.items():
                    if "FS_XL=+-" + k in s:
                        lsb = v
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
            gyr.append(v[4:7])
    if not t:
        raise SystemExit("没有有效数据")
    if lsb is None:
        lsb = SCALE["4g"]
    return (np.array(t), np.array(acc, float) * lsb,
            np.array(gyr, float) * 0.0175, lsb)


def simulate(t, acc, gyr):
    """逐样本复刻固件 CPR_Update"""
    hp1 = Biquad(FS_DESIGN, HP_HZ, True)
    hp2 = Biquad(FS_DESIGN, HP_HZ, True)
    lp = Biquad(FS_DESIGN, LP_HZ, False)

    g = np.zeros(3)
    g_ok = False
    cpr_t = 0.0
    t_cross = 0.0
    have_cross = False
    positive = False
    av_sq = 0.0
    n = 0
    gmax = 0.0

    hyst = 60.0
    period_avg = 0.5
    idle = True
    rms_avg = 0.0

    depth = 0.0
    depth_avg = 0.0
    d_hist = [0.0, 0.0, 0.0]
    d_idx = 0
    d_cnt = 0
    rate = 0.0
    t_event = 0.0
    count = 0

    events = []      # 接受的按压
    rejected = []    # 被拒绝的半周（带原因）
    dt_prev = 1.0 / 120.0

    for i in range(len(t)):
        if i == 0:
            dt = 1.0 / 120.0
        else:
            dt = t[i] - t[i - 1]
            if (dt <= 0.0) or (dt > 0.05):
                dt = 1.0 / 120.0
        cpr_t += dt

        ax, ay, az = acc[i]
        gx, gy, gz = gyr[i]
        amag = math.sqrt(ax * ax + ay * ay + az * az)
        gmag = math.sqrt(gx * gx + gy * gy + gz * gz)

        if amag < 100.0:
            continue

        # 1. 重力方向
        if not g_ok:
            g[:] = [ax / amag, ay / amag, az / amag]
            g_ok = True
        else:
            alpha = dt / (GRAVITY_TAU + dt)
            g[0] += (ax / amag - g[0]) * alpha
            g[1] += (ay / amag - g[1]) * alpha
            g[2] += (az / amag - g[2]) * alpha
            gn = math.sqrt(g[0] ** 2 + g[1] ** 2 + g[2] ** 2)
            if gn < 0.1:
                gn = 1.0
            g /= gn

        # 2. 竖直加速度
        av = ax * g[0] + ay * g[1] + az * g[2] - 1000.0

        # 3. 带通
        av = hp1.run(av)
        av = hp2.run(av)
        av = lp.run(av)

        if cpr_t < WARMUP_S:
            continue

        # 空闲复位（只在刚进入空闲时执行一次）
        if (cpr_t - t_event) > 3.0:
            if not idle:
                idle = True
                rate = 0.0
                depth = 0.0
                depth_avg = 0.0
                d_cnt = 0
                d_idx = 0
                d_hist = [0.0, 0.0, 0.0]
                have_cross = False
                av_sq = 0.0
                n = 0
                gmax = 0.0
                positive = False

        # 4. 累加能量
        av_sq += av * av
        n += 1
        if gmag > gmax:
            gmax = gmag

        # 5. 半周期检测
        if not positive:
            if av > hyst:
                half = cpr_t - t_cross

                if have_cross and (0.12 <= half <= 0.50) and \
                        ((2.0 * half) >= (0.75 * period_avg)):
                    period = 2.0 * half
                    a_rms = math.sqrt(av_sq / n) if n > 0 else 0.0

                    hyst = 0.30 * a_rms
                    hyst = max(40.0, min(100.0, hyst))

                    if (a_rms > 10.0) and ((rms_avg <= 20.0) or (a_rms >= 0.5 * rms_avg)):
                        w = 6.2831853 / period
                        depth = GAIN * (2.8284271 * a_rms * 9.80665) / (w * w)

                        if MIN_DEPTH <= depth <= MAX_DEPTH:
                            rms_avg = a_rms if rms_avg <= 0.0 else (0.8 * rms_avg + 0.2 * a_rms)
                            count += 1
                            t_event = cpr_t

                            d_hist[d_idx] = depth
                            d_idx = (d_idx + 1) % 3
                            if d_cnt < 3:
                                d_cnt += 1

                            med = depth
                            if d_cnt >= 3:
                                h0, h1, h2 = d_hist
                                if (h0 >= h1 and h0 <= h2) or (h0 >= h2 and h0 <= h1):
                                    med = h0
                                elif (h1 >= h0 and h1 <= h2) or (h1 >= h2 and h1 <= h0):
                                    med = h1
                                else:
                                    med = h2

                            depth_avg = med if depth_avg <= 0.0 else (0.6 * depth_avg + 0.4 * med)

                            r = 60.0 / period
                            rate = r if rate <= 0.0 else (0.8 * rate + 0.2 * r)

                            idle = False
                            period_avg = 0.8 * period_avg + 0.2 * period

                            events.append({
                                "t": cpr_t, "period": period, "half": half,
                                "a_rms": a_rms, "depth": depth,
                                "count": count, "rate": rate,
                            })
                        else:
                            rejected.append({"t": cpr_t, "period": period,
                                             "a_rms": a_rms, "depth": depth,
                                             "why": "深度超范围(%.0f mm)" % depth})
                    else:
                        rejected.append({"t": cpr_t, "period": period,
                                         "a_rms": a_rms, "depth": 0.0,
                                         "why": "幅度门限(%.0f < %.0f)" % (a_rms, 0.5 * rms_avg)})
                elif have_cross:
                    period = 2.0 * half
                    if not (0.12 <= half <= 0.50):
                        why = "半周超范围(%.3f s)" % half
                    else:
                        why = "节奏门限(%.3f < %.3f)" % (period, 0.75 * period_avg)
                    rejected.append({"t": cpr_t, "period": period,
                                     "a_rms": 0.0, "depth": 0.0, "why": why})

                positive = True
                have_cross = False

        else:
            if av < -hyst:
                positive = False
                have_cross = True
                t_cross = cpr_t
                av_sq = av * av
                n = 1
                gmax = gmag

    return events, rejected


def main():
    path = find_file(sys.argv)
    t, acc, gyr, lsb = read_capture(path)

    print("=" * 70)
    print("文件:", os.path.basename(path))
    print("样本 %d, 时长 %.1f s, 量程系数 %.3f mg/LSB" %
          (len(t), t[-1] - t[0], lsb))

    events, rejected = simulate(t, acc, gyr)

    print("\n[固件复刻结果]")
    print("接受按压: %d 次" % len(events))
    if events:
        per = np.array([e["period"] for e in events])
        dep = np.array([e["depth"] for e in events])
        print("周期: 平均 %.3f s (%.0f 次/分), 范围 %.2f~%.2f" %
              (per.mean(), 60 / per.mean(), per.min(), per.max()))
        print("深度: 平均 %.0f mm, 中位 %.0f mm, 范围 %.0f~%.0f" %
              (dep.mean(), np.median(dep), dep.min(), dep.max()))

    print("\n拒绝统计: %d 个半周" % len(rejected))
    kinds = {}
    for r in rejected:
        k = r["why"].split("(")[0]
        kinds[k] = kinds.get(k, 0) + 1
    for k, v in kinds.items():
        print("  %-12s %d" % (k, v))

    if rejected:
        print("\n前 10 个被拒绝的半周:")
        print("  %8s  %8s  %8s  %s" % ("t(s)", "周期(s)", "a_rms", "原因"))
        for r in rejected[:10]:
            print("  %8.2f  %8.3f  %8.0f  %s" %
                  (r["t"], r["period"], r["a_rms"], r["why"]))

    if events:
        print("\n前 10 次接受的按压:")
        print("  %4s %8s %8s %8s %8s" % ("#", "t(s)", "周期", "a_rms", "深度"))
        for e in events[:10]:
            print("  %4d %8.2f %8.3f %8.0f %8.1f" %
                  (e["count"], e["t"], e["period"], e["a_rms"], e["depth"]))

    # 冲突检查：同一时刻既有接受又有拒绝（说明一次按压被拆开）
    return events, rejected


if __name__ == "__main__":
    main()
