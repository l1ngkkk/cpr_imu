# -*- coding: utf-8 -*-
"""Exploratory dump of sim01/02/03 IMU + platform data."""
import io, os, sys, csv, glob
import numpy as np

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIMS = ["sim01", "sim02", "sim03"]


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
    return np.array(t), np.array(acc, float) * 0.122


def find_platform(sim):
    d = os.path.join(BASE, sim)
    p = os.path.join(d, "platform_raw.csv")
    if os.path.exists(p):
        return p
    cands = [c for c in glob.glob(os.path.join(d, "*", "*.csv"))
             if "校验" not in os.path.basename(c)]
    return cands[0]


def explore(sim):
    d = os.path.join(BASE, sim)
    imu_files = sorted(glob.glob(os.path.join(d, "lsm6dsv_raw_*.txt")))
    imu = imu_files[-1]
    plat = find_platform(sim)
    print("=" * 78)
    print(sim, "| IMU:", os.path.basename(imu), "| PLAT:", os.path.basename(plat))

    t, acc = load_sensor(imu)
    dt = np.diff(t)
    print("IMU: n=%d  dur=%.1fs  fs=%.1f Hz  dt med %.1f ms p99 %.1f ms max %.1f" %
          (len(t), t[-1] - t[0], 1.0 / np.mean(dt), np.median(dt) * 1e3,
           np.percentile(dt, 99) * 1e3, dt.max() * 1e3))

    rows = list(csv.DictReader(io.open(plat, encoding="utf-8-sig", errors="replace")))
    print("PLAT: n=%d" % len(rows))
    from collections import Counter
    print("  sources:", Counter(r["source"] for r in rows))
    print("  cols:", list(rows[0].keys()))

    # mannequin rows
    mq = [r for r in rows if r["mannequin_raw"]]
    print("  mannequin_raw rows: %d" % len(mq))
    v = np.array([float(r["mannequin_raw"]) for r in mq])
    tt = np.array([float(r["elapsed_s"]) for r in mq])
    print("  raw range %.2f .. %.2f  (unit=%s)" % (v.min(), v.max(), mq[0]["raw_unit"]))
    print("  elapsed %.2f .. %.2f s" % (tt.min(), tt.max()))

    pk = [r for r in rows if r["source"] == "mannequin_peak"]
    print("  peak rows: %d" % len(pk))
    if pk:
        pv = np.array([float(r["mannequin_depth"]) for r in pk])
        pt = np.array([float(r["elapsed_s"]) for r in pk])
        pf = [r.get("mannequin_frequency") for r in pk]
        print("  peak depth range %.1f .. %.1f cm ; freq sample %s" %
              (pv.min(), pv.max(), pf[:6]))
        print("  peak t range %.2f .. %.2f" % (pt.min(), pt.max()))

    # show a small window of raw values to see polarity / noise
    m = (tt > tt.min() + 20) & (tt < tt.min() + 24)
    print("  raw@20..24s:", np.round(v[m][:40], 1))


if __name__ == "__main__":
    for s in SIMS:
        explore(s)
