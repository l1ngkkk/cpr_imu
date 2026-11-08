# -*- coding: utf-8 -*-
"""第二组实验：MCU 可实现的因果漏积分、特征回归(LOSO)、频带敏感性。"""
import io, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bench as B
import pipeline as P
from cpr_fit import SIMS, OUT
from bench_run import build


def main():
    lines = []
    def say(s=""):
        print(s); lines.append(s)

    data = {}
    for s in SIMS:
        h, ref, mk = build(s)
        data[s] = (h, ref, mk)

    # ---- 漏积分 ----
    say("=" * 84)
    say("D. 因果漏积分二次积分（MCU 可实现，只需 2 个状态）")
    say("   %-10s %-6s %8s %8s %8s %8s" % ("方法", "sim", "MAE", "MAPE", "r", "全段增益"))
    for fl in (0.15, 0.3, 0.5):
        ks = []
        for s in SIMS:
            h, ref, mk = data[s]
            est = B.m_leaky(h, fl)
            sc = B.score(est, ref, mk)
            ks.append(sc["K"])
            say("   f_leak=%.2f  %-6s %8.2f %7.1f%% %8.3f %8.3f"
                % (fl, s, sc["mae"], sc["mape"], sc["r"], sc["K"]))
        say("      增益离散 %.1f%%" % ((max(ks) - min(ks)) / np.mean(ks) * 100))
        say("")

    # ---- 频带敏感性：全段积分 ----
    say("=" * 84)
    say("E. 全段频域二次积分对频带的敏感性")
    say("   %-16s %-6s %8s %8s %8s" % ("频带", "sim", "MAE", "MAPE", "r"))
    for lo, hi in ((0.4, 10), (0.5, 12), (0.7, 8), (1.0, 6)):
        for s in SIMS:
            h, ref, mk = data[s]
            est, _ = B.m_intwin(h, lo, hi)
            sc = B.score(est, ref, mk)
            say("   %.1f-%.0f Hz      %-6s %8.2f %7.1f%% %8.3f" % (lo, hi, s, sc["mae"], sc["mape"], sc["r"]))
        say("")

    # ---- 特征回归（留一实验）----
    say("=" * 84)
    say("F. 特征回归（A1,A2,A3,T,rms,a_pp -> 深度）留一实验交叉验证")
    X, Y, sid = [], [], []
    for s in SIMS:
        h, ref, mk = data[s]
        F = B.feats(h)
        for k in range(len(F)):
            if mk[k] and np.all(np.isfinite(F[k])) and np.isfinite(ref[k]):
                X.append(F[k]); Y.append(ref[k]); sid.append(s)
    X = np.array(X); Y = np.array(Y); sid = np.array(sid)
    Xn = X.copy()
    mu = Xn.mean(axis=0); sd = Xn.std(axis=0); sd[sd < 1e-9] = 1.0
    Xn = (Xn - mu) / sd

    # 只用 A1 的线性基线
    d1 = B.m_dft1  # noqa
    say("   %-22s %8s %8s" % ("模型", "MAE", "MAPE"))
    for name, cols in (("仅 A1 (基线)", [0]),
                       ("A1+A2+A3", [0, 1, 2]),
                       ("A1+T", [0, 3]),
                       ("A1+A2+A3+T+rms", [0, 1, 2, 3, 4]),
                       ("全部 6 个特征", list(range(6)))):
        errs = []
        for s in SIMS:
            tr = sid != s
            Xtr, Ytr = Xn[tr][:, cols], Y[tr]
            Xte, Yte = Xn[~tr][:, cols], Y[~tr]
            lam = 1.0
            A = np.column_stack([np.ones(len(Xtr)), Xtr])
            R = np.eye(A.shape[1]) * lam
            R[0, 0] = 0.0
            beta = np.linalg.solve(A.T @ A + R, A.T @ Ytr)
            At = np.column_stack([np.ones(len(Xte)), Xte])
            pred = At @ beta
            errs.append(np.abs(pred - Yte).mean())
        say("   %-22s %8.2f" % (name, np.mean(errs)))
        say("     " + "  ".join("%s:%.2f" % (s, e) for s, e in zip(SIMS, errs)))

    with io.open(os.path.join(OUT, "bench2.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
