# -*- coding: utf-8 -*-
"""
无界面回放（board_replay.py 的命令行版）
=======================================
协议与 GUI 版完全一致：把采集文件里的 t_ms,ax,...,temp 一行行发给单片机，
同时把回传的 EV / EVL / DV 行存成日志，之后直接喂给 board_check.py。

用法：
    python board_replay_cli.py COM3 sim03
    python board_replay_cli.py COM3 imu_process_03/sim03/lsm6dsv_raw_*.txt --speed 1
    python board_replay_cli.py COM3 sim03 --speed 2 --out board_logs/xxx.txt

参数：
    port        串口，如 COM3
    target      sim 目录名 / 目录路径 / 原始 txt 文件
    --speed     回放倍速，默认 1（实时）。大于 2 有丢包风险，因为单片机收缓冲只有 2 KB
    --out       日志路径，默认 board_logs/board_<tag>_<时间>.txt
    --quiet     不打印每一行回传
"""

import os
import io
import sys
import glob
import time
import argparse
import threading

try:
    import serial
except ImportError:
    sys.stderr.write("需要 pyserial： pip install pyserial\n")
    sys.exit(2)

HERE = os.path.dirname(os.path.abspath(__file__))


def load_capture(path):
    """返回 [(t_ms, 原始行)]，只保留 8 字段、全整数的行"""
    rows = []
    with io.open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            p = s.split(",")
            if len(p) != 8:
                continue
            try:
                t = int(p[0])
                for x in p:
                    int(x)
            except ValueError:
                continue
            rows.append((t, s))
    return rows


def resolve_target(arg):
    """sim03 / 目录 / 原始 txt -> (原始文件路径, 标签)"""
    if os.path.isfile(arg):
        return arg, os.path.splitext(os.path.basename(arg))[0]
    d = arg
    if not os.path.isdir(d):
        for cand in (os.path.join(HERE, arg), os.path.join(HERE, "imu_process_03", arg)):
            if os.path.isdir(cand):
                d = cand
                break
    if not os.path.isdir(d):
        raise SystemExit("找不到目标: %s" % arg)
    files = sorted(glob.glob(os.path.join(d, "lsm6dsv_raw_*.txt")))
    if not files:
        raise SystemExit("目录里没有 lsm6dsv_raw_*.txt: %s" % d)
    return files[-1], os.path.basename(os.path.normpath(d))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("target")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()

    raw, tag = resolve_target(a.target)
    rows = load_capture(raw)
    if not rows:
        raise SystemExit("文件里没有可回放的数据行")
    print("原始数据: %s  (%d 行, %.1f s)"
          % (os.path.basename(raw), len(rows), (rows[-1][0] - rows[0][0]) / 1000.0))
    print("串口 %s @115200, 倍速 %.1fx" % (a.port, a.speed))

    if a.out:
        out = a.out
    else:
        logdir = os.path.join(HERE, "board_logs")
        if not os.path.isdir(logdir):
            os.makedirs(logdir)
        out = os.path.join(logdir, "board_%s_%s.txt"
                           % (tag, time.strftime("%Y%m%d_%H%M%S")))
    logf = io.open(out, "w", encoding="utf-8")
    logf.write("# replay %s -> %s @ %s speed %.1f\n"
               % (raw, a.port, time.strftime("%Y-%m-%d %H:%M:%S"), a.speed))
    logf.flush()

    ser = serial.Serial(a.port, 115200, timeout=0.2)
    time.sleep(0.3)
    ser.reset_input_buffer()

    state = {"run": True, "ev": 0, "evl": 0, "dv": 0, "other": 0, "n": 0}

    def rx():
        buf = b""
        while state["run"]:
            try:
                chunk = ser.read(512)
            except Exception as e:
                sys.stderr.write("[接收异常] %s\n" % e)
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                s = line.decode("ascii", errors="replace").strip()
                if not s:
                    continue
                state["n"] += 1
                logf.write(s + "\n")
                logf.flush()
                if s.startswith("EVL,"):
                    state["evl"] += 1
                    if not a.quiet:
                        print("  <- " + s)
                elif s.startswith("EV,"):
                    state["ev"] += 1
                    if not a.quiet:
                        print("  <- " + s)
                elif s.startswith("DV,"):
                    state["dv"] += 1
                else:
                    state["other"] += 1

    th = threading.Thread(target=rx)
    th.daemon = True
    th.start()

    t0 = rows[0][0]
    t_start = time.time()
    sent = 0
    try:
        for t_ms, text in rows:
            target_t = t_start + (t_ms - t0) / 1000.0 / a.speed
            while True:
                dt = target_t - time.time()
                if dt <= 0:
                    break
                time.sleep(min(dt, 0.005))
            ser.write((text + "\r\n").encode("ascii"))
            sent += 1
            if sent % 500 == 0:
                print("  已发 %d/%d 行, 收到 EV %d / EVL %d"
                      % (sent, len(rows), state["ev"], state["evl"]))
    except KeyboardInterrupt:
        print("\n[中断]")
    finally:
        # 等板子把最后几次按压的回传吐完
        time.sleep(1.2)
        state["run"] = False
        th.join(timeout=1.0)
        try:
            ser.close()
        except Exception:
            pass
        logf.close()

    print("\n完成：发送 %d 行，收到 EV %d 条 / EVL %d 条 / DV %d 条 / 其它 %d 条"
          % (sent, state["ev"], state["evl"], state["dv"], state["other"]))
    print("日志: %s" % out)
    if state["evl"] < 10:
        print("[警告] EVL 少于 10 条，检查是否烧的是新固件（应有 EVL 行）")


if __name__ == "__main__":
    main()
