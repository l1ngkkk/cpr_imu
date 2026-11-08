# -*- coding: utf-8 -*-
"""
 v2  imu_process_03 


    ?tau=1s) -> av = au - 1000
    -> ?1Hz) + ?8Hz)
    -> av < -hyst av > +hyst 
    ->  = ?2?    ->  = ?period120 =2/period?             ?RMS 
    ->  += 13mm 3 ?    ->  = 0.30a_rms?40~100
    ->  3s have_cross/av_sq/period_avg/rms_avg
    -> ?0.12~0.5s??0.75avg_rms ?0.5avg
    -> ?float32 

?    python fw_replica_v2.py <sim>
"""

import os
import sys
import io
import csv
import glob
import math
import numpy as np

FS = 120.0
HP_HZ = 1.0
LP_HZ = 8.0
WARMUP_S = 2.0
MIN_DEPTH = 10.0
MAX_DEPTH = 150.0
TAU = 1.0
FIT_BUF = 160
OFFSET_MM = 13.0
OFFSET_SKIP = 2


def f32(x):
    return np.float32(x)


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


def simulate(t, acc):
    hp1 = Biquad(FS, HP_HZ, True)
    hp2 = Biquad(FS, HP_HZ, True)
    lp = Biquad(FS, LP_HZ, False)

    g = np.zeros(3, dtype=np.float32)
    g_ok = False
    cpr_t = 0.0
    t_cross = 0.0
    have_cross = False
    positive = False
    av_sq = 0.0
    n = 0

    hyst = 60.0
    period_avg = 0.5
    rms_avg = 0.0
    idle = True
    t_event = 0.0
    count = 0
    d_hist = [0.0, 0.0, 0.0]
    d_idx = 0
    d_cnt = 0
    depth_avg = 0.0
    rate = 0.0

    fit_buf = np.zeros(FIT_BUF, dtype=np.float32)
    fit_idx = 0

    events = []
    reject = {"half": 0, "rhythm": 0, "amp": 0, "depth": 0}

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
            g = np.array([ax / amag, ay / amag, az / amag], dtype=np.float32)
            g_ok = True
        else:
            alpha = dt / (TAU + dt)
            g = g + (np.array([ax / amag, ay / amag, az / amag], dtype=np.float32) - g) * alpha
            gn = float(np.linalg.norm(g))
            if gn < 0.1:
                gn = 1.0
            g = (g / gn).astype(np.float32)

        av = float(np.dot(np.array([ax, ay, az], dtype=np.float32), g) - np.float32(1000.0))
        av = hp1.run(av)
        av = hp2.run(av)
        av = lp.run(av)
        av = float(np.float32(av))

        fit_buf[fit_idx] = av
        fit_idx = (fit_idx + 1) % FIT_BUF

        if cpr_t < WARMUP_S:
            continue

        # ?        if (cpr_t - t_event) > 3.0:
            if not idle:
                idle = True
                d_cnt = 0
                d_idx = 0
                d_hist = [0.0, 0.0, 0.0]
                have_cross = False
                av_sq = 0.0
                n = 0
                positive = False
                period_avg = 0.5
                rms_avg = 0.0

        av_sq += av * av
        n += 1

        if not positive:
            if av > hyst:
                half = cpr_t - t_cross

                # ?2half
                if count > 0:
                    period = cpr_t - t_event
                    if (period < 0.30) or (period > 1.00):
                        period = 2.0 * half
                else:
                    period = 2.0 * half

                ok_half = (0.12 <= half <= 0.50)
                ok_rhythm = (period >= 0.75 * period_avg)

                if have_cross and ok_half and ok_rhythm:
                    a_rms = math.sqrt(av_sq / n) if n > 0 else 0.0
                    hyst = max(40.0, min(100.0, 0.30 * a_rms))

                    if (a_rms > 10.0) and ((rms_avg <= 20.0) or (a_rms >= 0.5 * rms_avg)):
                        # 
                        w = 2.0 * math.pi / period
                        nn = int(period * FS + 0.5)
                        d_fit = -1.0
                        if 8 <= nn <= FIT_BUF:
                            ss = cc = sc = aa_s = aa_c = 0.0
                            for k in range(nn):
                                idx = (fit_idx - nn + k) % FIT_BUF
                                tt = (k + 1) / FS
                                sn = math.sin(w * tt)
                                cs = math.cos(w * tt)
                                x = float(fit_buf[idx])
                                aa_s += x * sn
                                aa_c += x * cs
                                ss += sn * sn
                                cc += cs * cs
                                sc += sn * cs
                            det = ss * cc - sc * sc
                            if abs(det) > 1e-3:
                                A = (cc * aa_s - sc * aa_c) / det
                                B = (-sc * aa_s + ss * aa_c) / det
                                d_fit = 2.0 * math.hypot(A, B) * 9.80665 / (w * w)

                        if d_fit > 1.0:
                            depth = d_fit
                        else:
                            depth = 2.8284271 * a_rms * 9.80665 / (w * w)

                        if count >= OFFSET_SKIP:
                            depth += OFFSET_MM

                        if MIN_DEPTH <= depth <= MAX_DEPTH:
                            if (a_rms > 20.0) and (a_rms < 800.0):
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
                            if (period >= 0.30) and (period <= 0.90):
                                period_avg = 0.8 * period_avg + 0.2 * period
                            events.append({"t": cpr_t, "count": count,
                                           "depth": depth, "rate": rate,
                                           "period": period})
                        else:
                            reject["depth"] += 1
                    else:
                        reject["amp"] += 1
                elif have_cross:
                    if not ok_half:
                        reject["half"] += 1
                    else:
                        reject["rhythm"] += 1

                positive = True
                have_cross = False
        else:
            if av < -hyst:
                positive = False
                have_cross = True
                t_cross = cpr_t
                av_sq = av * av
                n = 1

    return events, reject


def main():
    base = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else ".")
    imu = sorted(glob.glob(os.path.join(base, "lsm6dsv_raw_*.txt")))[-1]
    t, acc, _ = load_sensor(imu)
    ev, rej = simulate(t, acc)

    print("=" * 64)
    print(os.path.basename(base))
    print("events %d accepted %d" % (len(ev) + sum(rej.values()), len(ev)))
    print(":  %d,  %d,  %d,  %d" %
          (rej["half"], rej["rhythm"], rej["amp"], rej["depth"]))
    if ev:
        dep = np.array([e["depth"] for e in ev])
        per = np.array([e["period"] for e in ev])
        print(": ?%.1f,  %.1f~%.1f" % (dep.mean(), dep.min(), dep.max()))
        print(": ?%.3f s (%.0f ??" % (per.mean(), 60 / per.mean()))
        gaps = [(ev[i]["t"] - ev[i - 1]["t"]) for i in range(1, len(ev))
                if ev[i]["t"] - ev[i - 1]["t"] > 1.5]
        if gaps:
            print(": %s" % ["%.1f" % g for g in gaps])


if __name__ == "__main__":
    main()

