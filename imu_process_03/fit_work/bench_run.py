# -*- coding: utf-8 -*-
"""跑 bench 里所有方法，输出对比表。"""
import io, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bench as B
import pipeline as P
from cpr_fit import SIMS, OUT


def build(sim):
    h = B.harness(sim)
    ncyc = len(h["tp"]) - 1
    pcyc = h["pcyc"]
    # 有效段判据统一用 dft1 的估计（与 pipeline.py 一致，所有方法共用同一 mask）
    base = B.m_dft1(h, 1)
    fake = np.zeros((len(pcyc), 6))
    fake[:, 2] = pcyc[:, 2]
    for j, k in h["pair"]:
        fake[j, 5] = base[k] if np.isfinite(base[k]) else 0.0
    mj = P.valid_mask(fake)

    ref = np.full(ncyc, np.nan)
    mk = np.zeros(ncyc, bool)
    for j, k in h["pair"]:
        ref[k] = pcyc[j, 2]
        mk[k] = mj[j]
    return h, ref, mk


def main():
    lines = []
    def say(s=""):
        print(s); lines.append(s)

    data = {}
    for s in SIMS:
        h, ref, mk = build(s)
        data[s] = (h, ref, mk)

    # 运动主方向 vs 重力方向
    say("=" * 88)
    say("A. 运动主方向 vs 重力方向")
    for s in SIMS:
        h, ref, mk = data[s]
        est, ang, c = B.m_axis(h)
        say("   %s  主方向与重力方向夹角 %.1f°  (cos=%.3f)" % (s, ang, c))

    # 方法集合
    methods = []

    def add(name, fn):
        methods.append((name, fn))

    add("dft1     单周期正交", lambda h: B.m_dft1(h, 1))
    add("dft1_2T  2周期窗", lambda h: B.m_dft1(h, 2))
    add("dft1_3T  3周期窗", lambda h: B.m_dft1(h, 3))
    add("dft2     1-2次谐波", B.m_dft2)
    add("intwin   全段积分", lambda h: B.m_intwin(h)[0])
    add("sogi     k=0.6", lambda h: B.m_sogi(h, 0.6))
    add("sogi     k=1.0", lambda h: B.m_sogi(h, 1.0))
    add("sogi     k=2.0", lambda h: B.m_sogi(h, 2.0))
    add("pca      主方向投影", lambda h: B.m_axis(h)[0])

    say("")
    say("=" * 88)
    say("B. 逐次按压（vs 平台 travel），增益按各文件均值比标定")
    say("   %-18s %-6s %8s %8s %8s" % ("方法", "sim", "MAE/mm", "MAPE", "r"))
    for name, fn in methods:
        for s in SIMS:
            h, ref, mk = data[s]
            est = fn(h)
            sc = B.score(est, ref, mk)
            if sc is None:
                continue
            say("   %-18s %-6s %8.2f %7.1f%% %8.3f"
                % (name, s, sc["mae"], sc["mape"], sc["r"]))
        say("")

    say("=" * 88)
    say("C. 5 次滑动平均后（设备显示口径）")
    say("   %-18s %-6s %8s %8s %8s" % ("方法", "sim", "MAE/mm", "MAPE", "r"))
    for name, fn in methods:
        for s in SIMS:
            h, ref, mk = data[s]
            est = fn(h)
            sc = B.score5(est, ref, mk)
            if sc is None:
                continue
            say("   %-18s %-6s %8.2f %7.1f%% %8.3f"
                % (name, s, sc["mae"], sc["mape"], sc["r"]))
        say("")

    with io.open(os.path.join(OUT, "bench.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
