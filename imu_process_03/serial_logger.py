# -*- coding: utf-8 -*-
"""
LSM6DSV 原始数据采集工具（串口 → txt）

用法：
    1. 安装依赖（只需一次）：
           pip install pyserial
    2. 运行：
           python serial_logger.py
    3. 在界面里选择串口、波特率（默认 115200）、保存位置，点“开始采集”。
       采集 60 秒后板子会发 "# EOF"，程序会自动停止并关闭文件。

数据格式（每行一条）：
    t_ms,ax,ay,az,gx,gy,gz,temp
    ax..az : 加速度原始 LSB（±4 g 时 0.122 mg/LSB）
    gx..gz : 陀螺原始 LSB（±500 dps 时 17.5 mdps/LSB）
    temp   : 温度原始值（25 + raw/256 摄氏度）

以 "#" 开头的是注释/进度行，可在界面里选择是否保留。
"""

import os
import sys
import time
import queue
import threading
import datetime
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

# ---------------------------------------------------------------------------
# 依赖检查：pyserial
# ---------------------------------------------------------------------------
try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


BAUD_LIST = ["9600", "19200", "38400", "57600",
             "115200", "230400", "460800", "921600"]

DEFAULT_BAUD = "115200"

# 预览窗口最多显示多少行（防止界面卡顿）
MAX_PREVIEW_LINES = 200


def _is_data_line(text):
    """判断是否是 8 列整数数据行"""
    parts = text.split(",")
    if len(parts) != 8:
        return False
    try:
        for p in parts:
            int(p)
    except ValueError:
        return False
    return True


class LoggerApp(object):
    """串口采集 + 写文件的图形界面"""

    def __init__(self, root):
        self.root = root
        self.root.title("LSM6DSV 原始数据采集")
        self.root.geometry("760x560")
        self.root.minsize(680, 480)

        self.thread = None
        self.running = False
        self.file = None

        # 统计量（工作线程写、界面线程读）
        self.stat_lines = 0
        self.stat_bytes = 0
        self.stat_start = 0.0
        self.stat_error = ""
        self.ui_queue = queue.Queue()

        self._build_ui()
        self._refresh_ports()

        # 没装 pyserial 时直接提示
        if serial is None:
            messagebox.showerror(
                "缺少依赖",
                "没有找到 pyserial。\n\n"
                "请在命令行执行：\n\n"
                "    pip install pyserial\n\n"
                "安装完成后重新运行本程序。")
            self.btn_start.config(state="disabled")

    # ------------------------------------------------------------------
    #  界面
    # ------------------------------------------------------------------
    def _build_ui(self):
        pad = {"padx": 8, "pady": 6}

        # ---- 串口设置 ----
        box_com = ttk.LabelFrame(self.root, text="串口设置")
        box_com.pack(fill="x", **pad)

        ttk.Label(box_com, text="串口：").grid(row=0, column=0, sticky="w", padx=6, pady=6)
        self.cmb_port = ttk.Combobox(box_com, width=18, state="readonly")
        self.cmb_port.grid(row=0, column=1, sticky="w", padx=6, pady=6)

        self.btn_refresh = ttk.Button(box_com, text="刷新", width=8, command=self._refresh_ports)
        self.btn_refresh.grid(row=0, column=2, padx=6, pady=6)

        ttk.Label(box_com, text="波特率：").grid(row=0, column=3, sticky="w", padx=6, pady=6)
        self.cmb_baud = ttk.Combobox(box_com, width=12, values=BAUD_LIST)
        self.cmb_baud.set(DEFAULT_BAUD)
        self.cmb_baud.grid(row=0, column=4, sticky="w", padx=6, pady=6)

        # ---- 保存设置 ----
        box_file = ttk.LabelFrame(self.root, text="保存位置")
        box_file.pack(fill="x", **pad)

        ttk.Label(box_file, text="文件：").grid(row=0, column=0, sticky="w", padx=6, pady=6)
        self.var_path = tk.StringVar(value=self._default_filename())
        self.ent_path = ttk.Entry(box_file, textvariable=self.var_path, width=62)
        self.ent_path.grid(row=0, column=1, sticky="we", padx=6, pady=6)

        self.btn_browse = ttk.Button(box_file, text="浏览…", width=8, command=self._browse)
        self.btn_browse.grid(row=0, column=2, padx=6, pady=6)

        box_file.columnconfigure(1, weight=1)

        self.var_keep_comment = tk.BooleanVar(value=True)
        ttk.Checkbutton(box_file,
                        text="保留 # 开头的注释/进度行（取消勾选则只保存数据行）",
                        variable=self.var_keep_comment).grid(
            row=1, column=0, columnspan=3, sticky="w", padx=6, pady=(0, 6))

        # ---- 控制 ----
        box_ctrl = ttk.Frame(self.root)
        box_ctrl.pack(fill="x", **pad)

        self.btn_start = ttk.Button(box_ctrl, text="开始采集", width=12, command=self._start)
        self.btn_start.pack(side="left", padx=6)

        self.btn_stop = ttk.Button(box_ctrl, text="停止", width=10,
                                   command=self._stop, state="disabled")
        self.btn_stop.pack(side="left", padx=6)

        self.lbl_status = ttk.Label(box_ctrl, text="就绪")
        self.lbl_status.pack(side="left", padx=16)

        # ---- 预览 ----
        box_view = ttk.LabelFrame(self.root, text="接收预览（只显示最后若干行）")
        box_view.pack(fill="both", expand=True, **pad)

        self.txt = tk.Text(box_view, height=18, wrap="none")
        self.txt.pack(side="left", fill="both", expand=True)

        sb = ttk.Scrollbar(box_view, orient="vertical", command=self.txt.yview)
        sb.pack(side="right", fill="y")
        self.txt.configure(yscrollcommand=sb.set)
        self.txt.configure(state="disabled")

    # ------------------------------------------------------------------
    #  小工具
    # ------------------------------------------------------------------
    def _default_filename(self):
        name = "lsm6dsv_raw_%s.txt" % datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        return os.path.join(os.path.dirname(os.path.abspath(__file__)), name)

    def _refresh_ports(self):
        ports = []
        if list_ports is not None:
            try:
                ports = [p.device for p in list_ports.comports()]
            except Exception:
                ports = []

        self.cmb_port["values"] = ports

        if ports:
            if self.cmb_port.get() not in ports:
                self.cmb_port.set(ports[0])
            self.lbl_status.config(text="找到 %d 个串口" % len(ports))
        else:
            self.cmb_port.set("")
            self.lbl_status.config(text="没有找到串口，请插好 USB 线后点“刷新”")

    def _browse(self):
        path = filedialog.asksaveasfilename(
            title="选择保存位置",
            defaultextension=".txt",
            initialfile=os.path.basename(self.var_path.get()),
            filetypes=[("文本文件", "*.txt"), ("CSV 文件", "*.csv"), ("所有文件", "*.*")])
        if path:
            self.var_path.set(path)

    def _append_preview(self, text):
        self.txt.configure(state="normal")
        self.txt.insert("end", text + "\n")
        lines = int(self.txt.index("end-1c").split(".")[0])
        if lines > MAX_PREVIEW_LINES:
            self.txt.delete("1.0", "%d.0" % (lines - MAX_PREVIEW_LINES))
        self.txt.see("end")
        self.txt.configure(state="disabled")

    # ------------------------------------------------------------------
    #  采集线程
    # ------------------------------------------------------------------
    def _start(self):
        if serial is None:
            return

        port = self.cmb_port.get().strip()
        if not port:
            messagebox.showwarning("提示", "请先选择串口。")
            return

        try:
            baud = int(self.cmb_baud.get())
        except ValueError:
            messagebox.showwarning("提示", "波特率必须是数字。")
            return

        path = self.var_path.get().strip()
        if not path:
            messagebox.showwarning("提示", "请选择保存位置。")
            return

        # 先试一下文件能否创建，然后立刻关闭（线程里会重新打开）
        try:
            with open(path, "w", encoding="utf-8"):
                pass
        except Exception as e:
            messagebox.showerror("无法创建文件", str(e))
            return

        self.stat_lines = 0
        self.stat_bytes = 0
        self.stat_start = time.time()
        self.stat_error = ""
        self.running = True

        self.btn_start.config(state="disabled")
        self.btn_stop.config(state="normal")
        self.cmb_port.config(state="disabled")
        self.cmb_baud.config(state="disabled")
        self.btn_browse.config(state="disabled")
        self.txt.configure(state="normal")
        self.txt.delete("1.0", "end")
        self.txt.configure(state="disabled")

        keep_comment = self.var_keep_comment.get()

        self.thread = threading.Thread(target=self._worker,
                                       args=(port, baud, path, keep_comment),
                                       daemon=True)
        self.thread.start()

        self.root.after(200, self._poll)

    def _worker(self, port, baud, path, keep_comment):
        ser = None
        try:
            ser = serial.Serial(port, baud, timeout=0.2)
        except Exception as e:
            self.stat_error = "打开串口失败：%s" % e
            self.running = False
            return

        try:
            with open(path, "w", encoding="utf-8", newline="") as f:
                # 文件头，方便以后追溯
                f.write("# LSM6DSV raw capture\n")
                f.write("# port=%s baud=%d\n" % (port, baud))
                f.write("# start=%s\n" %
                        datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
                f.write("# format: t_ms,ax,ay,az,gx,gy,gz,temp\n")

                preview_buf = []

                while self.running:
                    try:
                        raw = ser.readline()
                    except Exception as e:
                        self.stat_error = "读串口失败：%s" % e
                        break

                    if not raw:
                        continue

                    self.stat_bytes += len(raw)

                    try:
                        text = raw.decode("utf-8").rstrip("\r\n")
                    except UnicodeDecodeError:
                        text = raw.decode("latin-1").rstrip("\r\n")

                    is_comment = text.startswith("#")

                    # 板子复位时会打印启动文字，既不是注释也不是数据，跳过
                    if (not is_comment) and (not _is_data_line(text)):
                        continue

                    if (not is_comment) or keep_comment:
                        f.write(text + "\n")
                        if not is_comment:
                            self.stat_lines += 1
                            if (self.stat_lines % 50) == 0:
                                f.flush()

                    preview_buf.append(text)
                    if len(preview_buf) >= 20:
                        self.ui_queue.put("\n".join(preview_buf))
                        preview_buf = []

                    # 采集结束标记
                    if text.startswith("# EOF"):
                        break

                if preview_buf:
                    self.ui_queue.put("\n".join(preview_buf))

                f.flush()
        finally:
            try:
                if ser is not None:
                    ser.close()
            except Exception:
                pass
            self.running = False

    def _stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=2.0)
            self.thread = None
        self.btn_start.config(state="normal")
        self.btn_stop.config(state="disabled")
        self.cmb_port.config(state="readonly")
        self.cmb_baud.config(state="normal")
        self.btn_browse.config(state="normal")
        self.lbl_status.config(text="已停止，共 %d 条数据" % self.stat_lines)

    # ------------------------------------------------------------------
    #  界面刷新
    # ------------------------------------------------------------------
    def _drain_queue(self):
        while True:
            try:
                chunk = self.ui_queue.get_nowait()
            except queue.Empty:
                break
            self._append_preview(chunk)

    def _poll(self):
        self._drain_queue()
        if self.running:
            elapsed = time.time() - self.stat_start
            self.lbl_status.config(
                text="采集中：%d 条   %.1f KB   %.1f s" %
                     (self.stat_lines, self.stat_bytes / 1024.0, elapsed))
            self.root.after(200, self._poll)
        else:
            self._drain_queue()
            # 线程自己结束了（EOF 或出错）
            self.btn_start.config(state="normal")
            self.btn_stop.config(state="disabled")
            self.cmb_port.config(state="readonly")
            self.cmb_baud.config(state="normal")
            self.btn_browse.config(state="normal")

            if self.stat_error:
                self.lbl_status.config(text=self.stat_error)
                messagebox.showerror("串口错误", self.stat_error)
            else:
                self.lbl_status.config(
                    text="采集完成：%d 条数据，已保存到 %s" %
                         (self.stat_lines, self.var_path.get()))


def main():
    root = tk.Tk()
    try:
        # 高 DPI 下字体更清晰（失败也不影响）
        root.tk.call("tk", "scaling", 1.3)
    except Exception:
        pass
    app = LoggerApp(root)

    def on_close():
        app._stop()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
