# -*- coding: utf-8 -*-
"""Deep dive into the platform CSV semantics."""
import io, os, csv, glob
import numpy as np

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
import sys
SIM = sys.argv[1] if len(sys.argv) > 1 else "sim01"
d = os.path.join(BASE, SIM)
plat = os.path.join(d, "platform_raw.csv")
if not os.path.exists(plat):
    plat = [c for c in glob.glob(os.path.join(d, "*", "*.csv"))
            if "校验" not in os.path.basename(c)][0]

rows = list(csv.DictReader(io.open(plat, encoding="utf-8-sig", errors="replace")))
print(SIM, len(rows))

def col(name, src=None):
    out_t, out_v = [], []
    for r in rows:
        if src and r["source"] != src:
            continue
        s = r[name]
        if not s:
            continue
        try:
            out_v.append(float(s)); out_t.append(float(r["elapsed_s"]))
        except ValueError:
            pass
    o = np.argsort(out_t)
    return np.array(out_t)[o], np.array(out_v)[o]

for nm in ["mannequin_raw", "mannequin_depth", "mannequin_frequency",
           "mannequin_wave_cm", "camera_raw", "camera_depth",
           "camera_frequency", "imu_camera_angle", "imu_z_angle"]:
    t, v = col(nm)
    if len(v) == 0:
        print("%-20s EMPTY" % nm); continue
    print("%-20s n=%4d  t %.2f..%.2f  min %.2f  max %.2f  mean %.2f  med %.2f" %
          (nm, len(v), t.min(), t.max(), v.min(), v.max(), v.mean(), np.median(v)))

# peak rows detail
print("\n-- mannequin_peak rows (first 12) --")
k = 0
for r in rows:
    if r["source"] == "mannequin_peak":
        print("  t=%7.3f depth=%s cm  freq=%s  wave_cm=%s  confirm=%s  raw=%s" %
              (float(r["elapsed_s"]), r["mannequin_depth"], r["mannequin_frequency"],
               r["mannequin_wave_cm"], r["peak_confirm_elapsed_s"], r["packet_raw"]))
        k += 1
        if k >= 12:
            break

# raw sample spacing
t, v = col("mannequin_raw")
dt = np.diff(t)
print("\nmannequin_raw dt: med %.1f ms  p5 %.1f p95 %.1f" %
      (np.median(dt) * 1e3, np.percentile(dt, 5) * 1e3, np.percentile(dt, 95) * 1e3))

# packet_raw examples around compression
print("\n-- packet_raw examples --")
n = 0
for r in rows:
    if r["source"] == "mannequin" and r["packet_raw"] and n < 15:
        print("   t=%7.3f raw=%s  packet=%s  seq=%s idx=%s" %
              (float(r["elapsed_s"]), r["mannequin_raw"], r["packet_raw"],
               r["packet_seq"], r["sample_index"]))
        n += 1
