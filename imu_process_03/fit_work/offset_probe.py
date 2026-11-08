# -*- coding: utf-8 -*-
"""Locate the compression onset in both IMU and platform streams to pin the time offset."""
import io, os, csv, glob, re
import numpy as np
from scipy import signal

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIMS = ["sim01", "sim02", "sim03"]


def load_sensor(path):
    t, acc = [], []
    hdr = {}
    with io.open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            s = line.strip()
            if not s:
                continue
            if s.startswith("#"):
                m = re.match(r"#\s*(\w+)=(.+)", s)
                if m:
                    hdr.setdefault(m.group(1), m.group(2))
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
    return np.array(t), np.array(acc, float) * 0.122, hdr


def find_platform(sim):
    d = os.path.join(BASE, sim)
    p = os.path.join(d, "platform_raw.csv")
    if os.path.exists(p):
        return p
    return [c for c in glob.glob(os.path.join(d, "*", "*.csv"))
            if "校验" not in os.path.basename(c)][0]


for sim in SIMS:
    d = os.path.join(BASE, sim)
    imu = sorted(glob.glob(os.path.join(d, "lsm6dsv_raw_*.txt")))[-1]
    t, acc, hdr = load_sensor(imu)
    dt = np.diff(t); fs = 1.0 / np.mean(dt)

    amag = np.sqrt((acc ** 2).sum(axis=1))
    q = (amag > 900) & (amag < 1100)
    g = acc[q].mean(axis=0) if q.sum() > 30 else np.array([0, 0, 1000.])
    gn = np.linalg.norm(g); u = g / gn
    av = acc.dot(u) - gn
    amag_v = np.abs(av)
    # envelope of |vertical deviation|
    env = np.convolve(amag_v, np.ones(20) / 20.0, mode="same")
    thr = 0.25 * env.max()
    onset_imu = t[np.argmax(env > thr)]

    plat = find_platform(sim)
    rows = list(csv.DictReader(io.open(plat, encoding="utf-8-sig", errors="replace")))
    pt, pv = [], []
    for r in rows:
        if r["source"] == "mannequin" and r["mannequin_raw"]:
            pt.append(float(r["elapsed_s"])); pv.append(float(r["mannequin_raw"]))
    o = np.argsort(pt); pt = np.array(pt)[o]; pv = np.array(pv)[o]
    pv_s = np.convolve(pv, np.ones(5) / 5.0, mode="same")
    thr2 = pv.min() + 0.25 * (pv.max() - pv.min())
    onset_plat = pt[np.argmax(pv_s > thr2)]

    pk = [float(r["elapsed_s"]) for r in rows if r["source"] == "mannequin_peak"]
    print("%s  fs=%.1f  IMU hdr start=%s  t0=%.2f t_end=%.2f" % (sim, fs, hdr.get("start"), t[0], t[-1]))
    print("   IMU onset %.2f s   PLAT onset %.2f s  -> implied offset (sensor=plat+off) %.2f" %
          (onset_imu, onset_plat, onset_imu - onset_plat))
    print("   PLAT first/last peak %.2f / %.2f  n=%d   PLAT dur %.1f  IMU dur %.1f" %
          (pk[0], pk[-1], len(pk), pt[-1] - pt[0], t[-1] - t[0]))
