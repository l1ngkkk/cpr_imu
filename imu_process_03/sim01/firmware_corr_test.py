# -*- coding: utf-8 -*-
"""
固件逻辑复刻 + 残差常数修正，对比平台绝对深度

复刻内容与 firmware_sim.py 完全一致（逐帧回放），额外加：
    depth_out = 行程 + (count > warm_beats ? corr : 0)

用法：
    python firmware_corr_test.py <sim目录> [corr] [warm_beats]
"""

import os
import sys
import io
import csv
import glob
import math
import numpy as np


FS_DESIGN = 120.0
HP_HZ = 1.0
LP_HZ = 8.0
WARMUP_S = 2.0
MIN_DEPTH = 10.0
MAX_DEPTH = 150.0
GRAVITY_TAU = 1.0


class Biquad(object):
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


def load_sensor(path):
    t, acc, gyr = [], [], []
    for line in io.open(path, encoding="utf-8", errors="replace"):
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
        gyr.append(v[4:7])
    return np.array(t), np.array(acc, float) * 0.122, np.array(gyr, float) * 0.0175


def load_platform(path):
    wt, wv, pt = [], [], []
    with io.open(path, encoding="utf-8-sig", errors="replace", newline="") as f:
        for row in csv.DictReader(f):
            if row["source"] == "mannequin" and row["mannequin_raw"]:
                try:
                    wt.append(float(row["elapsed_s"]))
                    wv.append(float(row["mannequin_raw"]))
                except ValueError:
                    pass
            elif row["source"] == "mannequin_peak":
                try:
                    pt.append(float(row["elapsed_s"]))
                except ValueError:
                    pass
    o = np.argsort(wt)
    return np.array(wt)[o], np.array(wv)[o], np.array(sorted(pt))


def simulate(t, acc, gyr, corr, warm_beats):
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

    hyst = 60.0
    period_avg = 0.5
    idle = True
    rms_avg = 0.0
    count = 0
    t_event = 0.0
    d_hist = [0.0, 0.0, 0.0]
    d_idx = 0
    d_cnt = 0
    depth_avg = 0.0

    events = []

    for i in range(len(t)):
        dt = 1.0 / 120.0 if i == 0 else t[i] - t[i - 1]
        if (dt <= 0.0) or (dt > 0.05):
            dt = 1.0 / 120.0
        cpr_t += dt

        ax, ay, az = acc[i]
        amag = math.sqrt(ax * ax + ay * ay + az * az)
        if amag < 100.0:
            continue

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

        av = ax * g[0] + ay * g[1] + az * g[2] - 1000.0
        av = hp1.run(av)
        av = hp2.run(av)
        av = lp.run(av)

        if cpr_t < WARMUP_S:
            continue

        if (cpr_t - t_event) > 3.0:
            if not idle:
                idle = True
                d_cnt = 0
                d_idx = 0
                d_hist = [0.0, 0.0, 0.0]
                have_cross = False
                av_sq = 0.0
                n = 0
                positive = False

        av_sq += av * av
        n += 1

        if not positive:
            if av > hyst:
                half = cpr_t - t_cross
                period = 2.0 * half

                if (have_cross and (0.12 <= half <= 0.50) and
                        (period >= 0.75 * period_avg)):
                    a_rms = math.sqrt(av_sq / n) if n > 0 else 0.0
                    hyst = max(40.0, min(100.0, 0.30 * a_rms))

                    if (a_rms > 10.0) and ((rms_avg <= 20.0) or (a_rms >= 0.5 * rms_avg)):
                        w = 6.2831853 / period
                        depth = (2.8284271 * a_rms * 9.80665) / (w * w)

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

                            idle = False
                            period_avg = 0.8 * period_avg + 0.2 * period

                            applied = 0.0 if count <= warm_beats else corr
                            events.append({
                                "t": cpr_t, "count": count,
                                "travel": depth, "out": depth + applied,
                                "applied": applied,
                            })
                positive = True
                have_cross = False
        else:
            if av < -hyst:
                positive = False
                have_cross = True
                t_cross = cpr_t
                av_sq = av * av
                n = 1

    return events


def main():
    base = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else ".")
    corr = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    warm = int(sys.argv[3]) if len(sys.argv) > 3 else 2

    imu = sorted(glob.glob(os.path.join(base, "lsm6dsv_raw_*.txt")))[-1]
    plat = os.path.join(base, "platform_raw.csv")
    if not os.path.exists(plat):
        plat = [c for c in glob.glob(os.path.join(base, "*", "*.csv"))
                if "校验" not in os.path.basename(c)][0]

    t, acc, gyr = load_sensor(imu)
    wt, wv, pt = load_platform(plat)
    rest = np.median(wv[wt < 1.0])

    abs_true = []
    for tp in pt:
        m = (wt >= tp - 0.5) & (wt <= tp + 0.12)
        abs_true.append(wv[m].max() - rest if m.sum() >= 5 else np.nan)
    abs_true = np.array(abs_true)

    ev = simulate(t, acc, gyr, corr, warm)

    # 对齐
    et = np.array([e["t"] for e in ev])
    best = (1e9, 0.0)
    for off in np.arange(-6, 12, 0.01):
        d = np.abs((et + off)[None, :] - pt[:, None]).min(axis=1).mean()
        if d < best[0]:
            best = (d, off)
    off = best[1]

    # 逐次配对
    pairs = []
    for k, tp in enumerate(pt):
        idx = int(np.argmin(np.abs(et - (tp - off))))
        if abs(et[idx] - (tp - off)) > 0.25:
            continue
        pairs.append((k + 1, abs_true[k], ev[idx]["travel"], ev[idx]["out"]))

    pairs = np.array(pairs)
    ok = np.isfinite(pairs[:, 1])
    pairs = pairs[ok]

    print("=" * 70)
    print("目录 %s  修正 +%.1f mm（前 %d 次不修正）" %
          (os.path.basename(base), corr, warm))
    print("匹配 %d 次" % len(pairs))
    print("")
    print("%-4s %10s %10s %10s %10s" % ("次", "平台绝对", "行程", "修正后", "误差"))
    for r in pairs[:12]:
        print("%-4d %10.1f %10.1f %10.1f %+10.1f" %
              (r[0], r[1], r[2], r[3], r[3] - r[1]))

    for name, col in (("行程(不修正)", 2), ("修正后", 3)):
        v = pairs[:, col]
        e = v - pairs[:, 1]
        print("\n%-12s 平均误差 %+.1f mm, MAE %.1f mm, MAPE %.1f%%" %
              (name, e.mean(), np.abs(e).mean(),
               np.abs(e / pairs[:, 1] * 100).mean()))

    # 稳态（第 8 次之后）
    st = pairs[pairs[:, 0] >= 8]
    if len(st):
        e = st[:, 3] - st[:, 1]
        print("稳态(第8次后) 平均误差 %+.1f mm, MAE %.1f mm" %
              (e.mean(), np.abs(e).mean()))

    with io.open(os.path.join(base, "firmware_corr_result.txt"), "w",
                 encoding="utf-8") as f:
        f.write("修正 +%.1f mm, 前 %d 次不修正\n" % (corr, warm))
        f.write("匹配 %d 次\n" % len(pairs))
        for r in pairs:
            f.write("%d,%.1f,%.1f,%.1f,%+.1f\n" %
                    (r[0], r[1], r[2], r[3], r[3] - r[1]))


if __name__ == "__main__":
    main()
