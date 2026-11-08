# -*- coding: utf-8 -*-
import io, sys, csv
import numpy as np
p = sys.argv[1] if len(sys.argv) > 1 else "out/cycles_all.csv"
sim = sys.argv[2] if len(sys.argv) > 2 else "sim03"
rows = list(csv.DictReader(io.open(p, encoding="utf-8")))
sel = [r for r in rows if r["sim"] == sim]
print("%-7s %7s %7s %7s | %7s %7s %7s %7s %7s" %
      ("plat_t", "travel", "report", "off", "imu_T", "rms", "fit1", "fit2", "int"))
for r in sel:
    print("%-7s %7s %7s %7s | %7s %7s %7s %7s %7s" %
          (r["plat_t"][:6], r["plat_travel"], r["plat_report"], r["plat_restoff"],
           r["imu_T"], r["imu_rms"], r["imu_fit1"], r["imu_fit2"], r["imu_int"]))
a = np.array([[float(r[k]) for k in ("plat_travel", "imu_rms", "imu_fit1", "imu_fit2", "imu_fit3", "imu_int")] for r in sel])
print("\nmean:", np.round(a.mean(axis=0), 1), " sd:", np.round(a.std(axis=0), 1))
