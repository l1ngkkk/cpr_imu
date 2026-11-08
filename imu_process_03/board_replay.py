# -*- coding: utf-8 -*-
"""
板级验证上位机：用串口回放 IMU 原始数据，接收单片机的按压结果

用法：
    1. 单片机烧录 imu_process_03 固件（串口回放版）
    2. pip install pyserial
    3. python board_replay.py
    4. 选串口、选原始数据文件，点“开始回放”

协议：
    上位机 -> 单片机：t_ms,ax,ay,az,gx,gy,gz,temp    （原始 LSB，与采集文件一致）
    单片机 -> 上位机：EV,<序号>,<t_ms>,<间隔ms>,<深度mm>,<频率>
                      启动时发 REPLAY_READY
"""

import os
import io
import time
import queue
import threading
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

BAUD_LIST = ["115200", "230400", "460800", "921600"]
SPEED_LIST = ["1x", "2x", "5x", "10x", "尽快"]


def load_capture(path):
    """读取采集文件，返回 [(t_ms, 行文本)]"""
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


class App(object):
    def __init__(self, root):
        self.root = root
        self.root.title("IMU 板级回放验证")
        self.root.geometry("1100x640")

        self.ser = None
        self.running = False
        self.tx_thread = None
        self.rx_thread = None
        self.tx_q = queue.Queue()
        self.rx_q = queue.Queue()

        self.n_sent = 0
        self.n_events = 0
        self.n_events2 = 0
        self.t_start = 0.0
        self.log_path = None
        self.log_fh = None

        self._build()
        self._refresh_ports()

        if serial is None:
            messagebox.showerror("缺少依赖",
                                 "没有找到 pyserial。\n\n请执行：\n\n    pip install pyserial")
            self.btn_start.config(state="disabled")

    # ------------------------------------------------------------------
    def _build(self):
        pad = {"padx": 8, "pady": 6}

        top = ttk.LabelFrame(self.root, text="回放设置")
        top.pack(fill="x", **pad)

        ttk.Label(top, text="串口：").grid(row=0, column=0, sticky="w", padx=6, pady=4)
        self.cmb_port = ttk.Combobox(top, width=14, state="readonly")
        self.cmb_port.grid(row=0, column=1, padx=6)

        ttk.Button(top, text="刷新", width=6,
                   command=self._refresh_ports).grid(row=0, column=2, padx=6)

        ttk.Label(top, text="波特率：").grid(row=0, column=3, sticky="w", padx=6)
        self.cmb_baud = ttk.Combobox(top, width=10, values=BAUD_LIST)
        self.cmb_baud.set("115200")
        self.cmb_baud.grid(row=0, column=4, padx=6)

        ttk.Label(top, text="速度：").grid(row=0, column=5, sticky="w", padx=6)
        self.cmb_speed = ttk.Combobox(top, width=8, values=SPEED_LIST, state="readonly")
        self.cmb_speed.set("1x")
        self.cmb_speed.grid(row=0, column=6, padx=6)

        ttk.Label(top, text="原始数据：").grid(row=1, column=0, sticky="w", padx=6, pady=4)
        self.var_file = tk.StringVar()
        ttk.Entry(top, textvariable=self.var_file, width=70).grid(
            row=1, column=1, columnspan=5, sticky="we", padx=6)
        ttk.Button(top, text="浏览…", width=8,
                   command=self._browse).grid(row=1, column=6, padx=6)

        ctrl = ttk.Frame(self.root)
        ctrl.pack(fill="x", **pad)
        self.btn_start = ttk.Button(ctrl, text="开始回放", width=12, command=self._start)
        self.btn_start.pack(side="left", padx=6)
        self.btn_stop = ttk.Button(ctrl, text="停止", width=8,
                                   command=self._stop, state="disabled")
        self.btn_stop.pack(side="left", padx=6)
        self.lbl = ttk.Label(ctrl, text="就绪")
        self.lbl.pack(side="left", padx=16)

        # 两个显示区
        body = ttk.Frame(self.root)
        body.pack(fill="both", expand=True, **pad)
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(1, weight=1)

        ttk.Label(body, text="▶ 发送（原始数据）").grid(row=0, column=0, sticky="w")
        ttk.Label(body, text="◀ 接收（单片机结果）").grid(row=0, column=1, sticky="w")

        self.txt_tx = tk.Text(body, height=24, wrap="none", font=("Consolas", 9))
        self.txt_tx.grid(row=1, column=0, sticky="nsew", padx=(0, 4))
        self.txt_rx = tk.Text(body, height=24, wrap="none", font=("Consolas", 9))
        self.txt_rx.grid(row=1, column=1, sticky="nsew", padx=(4, 0))

        for w in (self.txt_tx, self.txt_rx):
            sb = ttk.Scrollbar(body, orient="vertical", command=w.yview)
            w.configure(yscrollcommand=sb.set, state="disabled")

    # ------------------------------------------------------------------
    def _refresh_ports(self):
        ports = []
        if list_ports is not None:
            try:
                ports = [p.device for p in list_ports.comports()]
            except Exception:
                ports = []
        self.cmb_port["values"] = ports
        if ports:
            self.cmb_port.set(ports[0])
        self.lbl.config(text="找到 %d 个串口" % len(ports))

    def _browse(self):
        p = filedialog.askopenfilename(
            title="选择原始数据文件",
            filetypes=[("文本/CSV", "*.txt *.csv"), ("所有文件", "*.*")])
        if p:
            self.var_file.set(p)

    def _append(self, widget, text):
        widget.configure(state="normal")
        widget.insert("end", text + "\n")
        n = int(widget.index("end-1c").split(".")[0])
        if n > 500:
            widget.delete("1.0", "%d.0" % (n - 500))
        widget.see("end")
        widget.configure(state="disabled")

    def _save_log(self, text):
        if self.log_fh is None:
            return
        try:
            self.log_fh.write(text + "\n")
            self.log_fh.flush()
        except Exception:
            pass

    # ------------------------------------------------------------------
    def _start(self):
        if serial is None:
            return
        port = self.cmb_port.get().strip()
        path = self.var_file.get().strip()
        if not port:
            messagebox.showwarning("提示", "请选择串口")
            return
        if not path or not os.path.exists(path):
            messagebox.showwarning("提示", "请选择原始数据文件")
            return

        try:
            rows = load_capture(path)
        except Exception as e:
            messagebox.showerror("读取失败", str(e))
            return
        if not rows:
            messagebox.showwarning("提示", "文件里没有可回放的数据行")
            return

        try:
            self.ser = serial.Serial(port, int(self.cmb_baud.get()), timeout=0.2)
        except Exception as e:
            messagebox.showerror("打开串口失败", str(e))
            return

        speed_txt = self.cmb_speed.get()
        speed = None if speed_txt == "尽快" else float(speed_txt.replace("x", ""))

        self.running = True
        self.n_sent = 0
        self.n_events = 0
        self.n_events2 = 0
        self.t_start = time.time()

        # 自动存盘：回放结束可以直接丢给 board_check.py
        logdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "board_logs")
        try:
            if not os.path.isdir(logdir):
                os.makedirs(logdir)
            tag = os.path.splitext(os.path.basename(path))[0]
            self.log_path = os.path.join(
                logdir, "board_%s_%s.txt" % (tag, time.strftime("%Y%m%d_%H%M%S")))
            self.log_fh = open(self.log_path, "w", encoding="utf-8")
            self.log_fh.write("# replay %s -> %s @ %s\n"
                              % (path, port, time.strftime("%Y-%m-%d %H:%M:%S")))
            self.log_fh.flush()
        except Exception as e:
            self.log_fh = None
            self.log_path = None
            self.rx_q.put("[日志保存失败] %s" % e)

        self.txt_tx.configure(state="normal"); self.txt_tx.delete("1.0", "end")
        self.txt_tx.configure(state="disabled")
        self.txt_rx.configure(state="normal"); self.txt_rx.delete("1.0", "end")
        self.txt_rx.configure(state="disabled")

        self.btn_start.config(state="disabled")
        self.btn_stop.config(state="normal")

        self.rx_thread = threading.Thread(target=self._receiver, daemon=True)
        self.rx_thread.start()
        self.tx_thread = threading.Thread(target=self._sender, args=(rows, speed), daemon=True)
        self.tx_thread.start()
        self.root.after(200, self._poll)

    def _sender(self, rows, speed):
        t0 = rows[0][0]
        try:
            for t_ms, text in rows:
                if not self.running:
                    break
                if speed is not None:
                    target = self.t_start + (t_ms - t0) / 1000.0 / speed
                    while self.running:
                        dt = target - time.time()
                        if dt <= 0:
                            break
                        time.sleep(min(dt, 0.005))
                self.ser.write((text + "\r\n").encode("ascii"))
                self.n_sent += 1
                self.tx_q.put(text)
        except Exception as e:
            self.rx_q.put("[发送异常] %s" % e)
        finally:
            self.running = False

    def _receiver(self):
        buf = b""
        while self.running:
            try:
                chunk = self.ser.read(256)
            except Exception as e:
                self.rx_q.put("[接收异常] %s" % e)
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                s = line.decode("ascii", errors="replace").strip()
                if not s:
                    continue
                self.rx_q.put(s)
                if s.startswith("EVL,"):
                    self.n_events2 += 1
                elif s.startswith("EV,"):
                    self.n_events += 1

    def _stop(self):
        self.running = False
        for th in (self.tx_thread, self.rx_thread):
            if th is not None:
                th.join(timeout=1.0)
        self.tx_thread = None
        self.rx_thread = None
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        if self.log_fh is not None:
            try:
                self.log_fh.close()
            except Exception:
                pass
            self.log_fh = None
        self.btn_start.config(state="normal")
        self.btn_stop.config(state="disabled")
        msg = ("已停止：发送 %d 行，EV(旧) %d 次，EVL(新) %d 次"
               % (self.n_sent, self.n_events, self.n_events2))
        if self.log_path:
            msg += "  日志已存：%s" % os.path.basename(self.log_path)
        self.lbl.config(text=msg)

    def _poll(self):
        while True:
            try:
                s = self.tx_q.get_nowait()
            except queue.Empty:
                break
            self._append(self.txt_tx, s)
        while True:
            try:
                s = self.rx_q.get_nowait()
            except queue.Empty:
                break
            self._append(self.txt_rx, s)
            self._save_log(s)

        if self.running:
            self.lbl.config(text="回放中：已发 %d 行，EV(旧) %d 次，EVL(新) %d 次"
                            % (self.n_sent, self.n_events, self.n_events2))
            self.root.after(200, self._poll)
        else:
            self._stop()


def main():
    root = tk.Tk()
    app = App(root)

    def on_close():
        app._stop()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
