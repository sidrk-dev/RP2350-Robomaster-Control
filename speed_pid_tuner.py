import csv
import math
import re
import threading
import time
from collections import deque

import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from tkinter import scrolledtext

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


BAUD_RATE = 115200
MAX_POINTS = 2500


def try_float(v):
    try:
        out = float(v)
        if math.isnan(out) or math.isinf(out):
            return None
        return out
    except Exception:
        return None


class SpeedPIDTunerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Speed PID Tuner")
        self.root.geometry("1200x850")

        self.ser = None
        self.running = True
        self.telemetry_lock = threading.Lock()
        self.telemetry = {i: deque(maxlen=MAX_POINTS) for i in range(1, 9)}
        self.csv_rows = []
        self.sweep_thread = None
        self.stop_sweep_flag = False

        self.step_active = False
        self.step_motor = 1
        self.step_target = 0.0
        self.step_start_ts = 0.0

        self._build_ui()
        self.root.after(120, self._update_plot)

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill="x")

        self.btn_connect = ttk.Button(top, text="Connect", command=self.toggle_connect)
        self.btn_connect.pack(side="left")

        ttk.Label(top, text="Port:").pack(side="left", padx=(10, 4))
        self.var_port = tk.StringVar(value="")
        self.cmb_ports = ttk.Combobox(top, textvariable=self.var_port, width=22)
        self.cmb_ports.pack(side="left")
        ttk.Button(top, text="Refresh Ports", command=self.refresh_ports).pack(side="left", padx=6)

        self.lbl_conn = ttk.Label(top, text="Disconnected", foreground="red")
        self.lbl_conn.pack(side="left", padx=10)

        ttk.Button(top, text="EMERGENCY STOP", command=self.emergency_stop).pack(side="right", padx=4)

        cfg = ttk.LabelFrame(self.root, text="Speed PID Settings", padding=10)
        cfg.pack(fill="x", padx=10, pady=(0, 8))

        self.var_motor = tk.IntVar(value=1)
        self.var_skp = tk.DoubleVar(value=0.30)
        self.var_ski = tk.DoubleVar(value=0.00)
        self.var_skd = tk.DoubleVar(value=0.00)
        self.var_sol = tk.DoubleVar(value=20.0)

        ttk.Label(cfg, text="Motor ID").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(cfg, from_=1, to=8, textvariable=self.var_motor, width=6).grid(row=0, column=1, sticky="w", padx=4)
        ttk.Label(cfg, text="SKP").grid(row=0, column=2, sticky="w", padx=(10, 2))
        ttk.Entry(cfg, textvariable=self.var_skp, width=10).grid(row=0, column=3, sticky="w")
        ttk.Label(cfg, text="SKI").grid(row=0, column=4, sticky="w", padx=(10, 2))
        ttk.Entry(cfg, textvariable=self.var_ski, width=10).grid(row=0, column=5, sticky="w")
        ttk.Label(cfg, text="SKD").grid(row=0, column=6, sticky="w", padx=(10, 2))
        ttk.Entry(cfg, textvariable=self.var_skd, width=10).grid(row=0, column=7, sticky="w")
        ttk.Label(cfg, text="SOL (A)").grid(row=0, column=8, sticky="w", padx=(10, 2))
        ttk.Entry(cfg, textvariable=self.var_sol, width=10).grid(row=0, column=9, sticky="w")
        ttk.Button(cfg, text="Apply Speed Gains", command=self.apply_speed_pid).grid(row=0, column=10, padx=10)

        test = ttk.LabelFrame(self.root, text="Test Controls", padding=10)
        test.pack(fill="x", padx=10, pady=(0, 8))

        self.var_target_rpm = tk.DoubleVar(value=120.0)
        self.var_hold_s = tk.DoubleVar(value=2.0)
        self.var_step_rpm = tk.DoubleVar(value=40.0)
        self.var_sweep_start = tk.DoubleVar(value=0.0)
        self.var_sweep_end = tk.DoubleVar(value=240.0)
        self.var_sweep_dwell = tk.DoubleVar(value=1.0)

        ttk.Label(test, text="Target RPM").grid(row=0, column=0, sticky="w")
        ttk.Entry(test, textvariable=self.var_target_rpm, width=10).grid(row=0, column=1, sticky="w", padx=4)
        ttk.Button(test, text="Start Step (+)", command=lambda: self.start_step(self.var_target_rpm.get())).grid(row=0, column=2, padx=6)
        ttk.Button(test, text="Start Step (-)", command=lambda: self.start_step(-abs(self.var_target_rpm.get()))).grid(row=0, column=3, padx=6)
        ttk.Button(test, text="Stop Motor", command=self.stop_motor).grid(row=0, column=4, padx=6)

        ttk.Label(test, text="Manual Step RPM").grid(row=1, column=0, sticky="w")
        ttk.Entry(test, textvariable=self.var_step_rpm, width=10).grid(row=1, column=1, sticky="w", padx=4)
        ttk.Button(test, text="-Step", command=lambda: self.jog_speed(-abs(self.var_step_rpm.get()))).grid(row=1, column=2, padx=6)
        ttk.Button(test, text="+Step", command=lambda: self.jog_speed(abs(self.var_step_rpm.get()))).grid(row=1, column=3, padx=6)
        ttk.Label(test, text="Hold (s)").grid(row=1, column=4, sticky="e")
        ttk.Entry(test, textvariable=self.var_hold_s, width=8).grid(row=1, column=5, padx=4)
        ttk.Button(test, text="Run Timed Step", command=self.run_timed_step).grid(row=1, column=6, padx=6)

        sweep = ttk.LabelFrame(self.root, text="Sweep Test", padding=10)
        sweep.pack(fill="x", padx=10, pady=(0, 8))
        ttk.Label(sweep, text="Start RPM").grid(row=0, column=0, sticky="w")
        ttk.Entry(sweep, textvariable=self.var_sweep_start, width=10).grid(row=0, column=1, sticky="w", padx=4)
        ttk.Label(sweep, text="End RPM").grid(row=0, column=2, sticky="w")
        ttk.Entry(sweep, textvariable=self.var_sweep_end, width=10).grid(row=0, column=3, sticky="w", padx=4)
        ttk.Label(sweep, text="Step RPM").grid(row=0, column=4, sticky="w")
        ttk.Entry(sweep, textvariable=self.var_step_rpm, width=10).grid(row=0, column=5, sticky="w", padx=4)
        ttk.Label(sweep, text="Dwell (s)").grid(row=0, column=6, sticky="w")
        ttk.Entry(sweep, textvariable=self.var_sweep_dwell, width=10).grid(row=0, column=7, sticky="w", padx=4)
        ttk.Button(sweep, text="Run Sweep", command=self.start_sweep).grid(row=0, column=8, padx=6)
        ttk.Button(sweep, text="Stop Sweep", command=self.stop_sweep).grid(row=0, column=9, padx=6)

        metrics = ttk.LabelFrame(self.root, text="Step Metrics (Selected Motor)", padding=10)
        metrics.pack(fill="x", padx=10, pady=(0, 8))
        self.lbl_metrics = ttk.Label(
            metrics,
            text="Rise: --  Overshoot: --  Settling: --  SSE: --",
            font=("Consolas", 10),
        )
        self.lbl_metrics.pack(anchor="w")

        plot_frame = ttk.Frame(self.root, padding=(10, 0, 10, 0))
        plot_frame.pack(fill="both", expand=True)

        self.fig = Figure(figsize=(10, 6), dpi=100)
        self.ax_rpm = self.fig.add_subplot(211)
        self.ax_cur = self.fig.add_subplot(212, sharex=self.ax_rpm)
        self.ax_rpm.set_ylabel("RPM")
        self.ax_cur.set_ylabel("Current (A)")
        self.ax_cur.set_xlabel("Time (s)")
        self.ax_rpm.grid(True, alpha=0.25)
        self.ax_cur.grid(True, alpha=0.25)

        self.line_rpm, = self.ax_rpm.plot([], [], color="tab:blue", linewidth=1.8, label="Actual RPM")
        self.line_target, = self.ax_rpm.plot([], [], color="tab:orange", linewidth=1.2, linestyle="--", label="Target RPM")
        self.line_cur, = self.ax_cur.plot([], [], color="tab:green", linewidth=1.5, label="Current (A)")
        self.ax_rpm.legend(loc="upper left")
        self.ax_cur.legend(loc="upper left")

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        log_frame = ttk.LabelFrame(self.root, text="Serial Log", padding=8)
        log_frame.pack(fill="both", padx=10, pady=10)

        row = ttk.Frame(log_frame)
        row.pack(fill="x")
        ttk.Button(row, text="Clear", command=self.clear_log).pack(side="left")
        ttk.Button(row, text="Save CSV", command=self.save_csv).pack(side="left", padx=6)
        self.log_box = scrolledtext.ScrolledText(log_frame, height=8, wrap="word", state="disabled")
        self.log_box.pack(fill="both", expand=True, pady=(6, 0))

        self.refresh_ports()

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.cmb_ports["values"] = ports
        if ports and not self.var_port.get():
            self.var_port.set(ports[0])

    def append_log(self, line):
        self.log_box.config(state="normal")
        self.log_box.insert("end", line + "\n")
        self.log_box.see("end")
        self.log_box.config(state="disabled")

    def clear_log(self):
        self.log_box.config(state="normal")
        self.log_box.delete("1.0", "end")
        self.log_box.config(state="disabled")

    def toggle_connect(self):
        if self.ser and self.ser.is_open:
            self.disconnect_serial()
            return

        port = self.var_port.get().strip()
        if not port:
            messagebox.showerror("No Port", "Select a serial port first.")
            return

        try:
            self.ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
            time.sleep(2.0)
            self.btn_connect.config(text="Disconnect")
            self.lbl_conn.config(text=f"Connected ({port})", foreground="green")
            self.append_log(f"[INFO] Connected to {port}")
            threading.Thread(target=self.serial_loop, daemon=True).start()
            self.send_command("S; TON")
        except Exception as e:
            messagebox.showerror("Connect Error", str(e))

    def disconnect_serial(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.btn_connect.config(text="Connect")
        self.lbl_conn.config(text="Disconnected", foreground="red")
        self.append_log("[INFO] Disconnected")

    def send_command(self, cmd):
        if not (self.ser and self.ser.is_open):
            self.append_log(f"[WARN] Not connected. Skipped: {cmd}")
            return
        try:
            self.ser.write((cmd + "\n").encode("utf-8"))
        except Exception as e:
            self.append_log(f"[ERR] send failed: {e}")
            self.disconnect_serial()

    def apply_speed_pid(self):
        mid = int(self.var_motor.get())
        skp = float(self.var_skp.get())
        ski = float(self.var_ski.get())
        skd = float(self.var_skd.get())
        sol = float(self.var_sol.get())
        batch = f"SKP {mid} {skp:.4f}; SKI {mid} {ski:.4f}; SKD {mid} {skd:.4f}; SOL {mid} {sol:.2f}"
        self.send_command(batch)
        self.append_log(f"[CMD] {batch}")

    def start_step(self, target_rpm):
        mid = int(self.var_motor.get())
        self.send_command(f"M {mid} {target_rpm:.2f}")
        self.step_active = True
        self.step_motor = mid
        self.step_target = float(target_rpm)
        self.step_start_ts = time.time()
        self.append_log(f"[TEST] Step start M{mid} -> {target_rpm:.2f} RPM")

    def stop_motor(self):
        mid = int(self.var_motor.get())
        self.send_command(f"M {mid} 0")
        self.step_active = False
        self.append_log(f"[TEST] Stop M{mid}")

    def emergency_stop(self):
        self.stop_sweep_flag = True
        self.step_active = False
        self.send_command("S")
        self.append_log("[SAFETY] EMERGENCY STOP sent")

    def jog_speed(self, delta):
        mid = int(self.var_motor.get())
        with self.telemetry_lock:
            hist = self.telemetry.get(mid, deque())
            cur = hist[-1][1] if hist else 0.0
        target = cur + delta
        self.start_step(target)

    def run_timed_step(self):
        hold = max(0.1, float(self.var_hold_s.get()))
        target = float(self.var_target_rpm.get())
        self.start_step(target)

        def _worker():
            time.sleep(hold)
            self.root.after(0, self.stop_motor)

        threading.Thread(target=_worker, daemon=True).start()

    def start_sweep(self):
        if self.sweep_thread and self.sweep_thread.is_alive():
            self.append_log("[WARN] Sweep already running")
            return

        self.stop_sweep_flag = False
        self.sweep_thread = threading.Thread(target=self._sweep_worker, daemon=True)
        self.sweep_thread.start()

    def stop_sweep(self):
        self.stop_sweep_flag = True
        self.append_log("[TEST] Sweep stop requested")

    def _sweep_worker(self):
        start = float(self.var_sweep_start.get())
        end = float(self.var_sweep_end.get())
        step = abs(float(self.var_step_rpm.get()))
        dwell = max(0.1, float(self.var_sweep_dwell.get()))

        if step < 1e-6:
            self.root.after(0, lambda: self.append_log("[ERR] Sweep step RPM must be > 0"))
            return

        direction = 1.0 if end >= start else -1.0
        val = start
        self.root.after(0, lambda: self.append_log(f"[TEST] Sweep start {start} -> {end}, step={step}, dwell={dwell}s"))

        while True:
            if self.stop_sweep_flag:
                break
            self.root.after(0, lambda rpm=val: self.start_step(rpm))
            t0 = time.time()
            while time.time() - t0 < dwell:
                if self.stop_sweep_flag:
                    break
                time.sleep(0.02)
            if self.stop_sweep_flag:
                break

            if direction > 0:
                if val + step > end:
                    break
                val += step
            else:
                if val - step < end:
                    break
                val -= step

        self.root.after(0, self.stop_motor)
        self.root.after(0, lambda: self.append_log("[TEST] Sweep finished"))

    def serial_loop(self):
        mid_re = re.compile(r"M(\d+)")
        rpm_re = re.compile(r"RPM:([^\s|]+)")
        amp_re = re.compile(r"A:([^\s|]+)")
        pos_re = re.compile(r"POS:([^\s|]+)")

        while self.running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if not line:
                        continue

                    self.root.after(0, self.append_log, line)
                    now = time.time()

                    for token in line.split("|"):
                        token = token.strip()
                        if not token.startswith("M"):
                            continue

                        mm = mid_re.search(token)
                        rm = rpm_re.search(token)
                        am = amp_re.search(token)
                        pm = pos_re.search(token)
                        if not mm:
                            continue

                        mid = int(mm.group(1))
                        rpm = try_float(rm.group(1)) if rm else None
                        amp = try_float(am.group(1)) if am else None
                        pos = try_float(pm.group(1)) if pm else None
                        if rpm is None:
                            continue
                        if amp is None:
                            amp = 0.0

                        with self.telemetry_lock:
                            self.telemetry[mid].append((now, rpm, amp, pos))
                            self.csv_rows.append((now, mid, rpm, amp, pos))
                time.sleep(0.005)
            except Exception as e:
                self.root.after(0, self.append_log, f"[ERR] serial loop: {e}")
                break

        self.root.after(0, self.disconnect_serial)

    def _compute_step_metrics(self, points):
        if not self.step_active or not points:
            return "Rise: --  Overshoot: --  Settling: --  SSE: --"
        if self.step_motor != int(self.var_motor.get()):
            return "Rise: --  Overshoot: --  Settling: --  SSE: --"

        t0 = self.step_start_ts
        target = self.step_target
        if abs(target) < 1e-6:
            return "Rise: --  Overshoot: --  Settling: --  SSE: --"

        post = [(t - t0, rpm) for (t, rpm, _a, _p) in points if t >= t0]
        if len(post) < 4:
            return "Rise: --  Overshoot: --  Settling: --  SSE: --"

        times = [p[0] for p in post]
        vals = [p[1] for p in post]
        abs_target = abs(target)
        norm = [math.copysign(v, target) for v in vals]

        y10 = 0.1 * abs_target
        y90 = 0.9 * abs_target
        t10 = None
        t90 = None
        for t, y in zip(times, norm):
            if t10 is None and y >= y10:
                t10 = t
            if t90 is None and y >= y90:
                t90 = t
                break
        rise = (t90 - t10) if (t10 is not None and t90 is not None and t90 >= t10) else None

        peak = max(norm) if target >= 0 else max([-v for v in norm])
        overshoot = max(0.0, (peak - abs_target) / abs_target * 100.0)

        band = 0.05 * abs_target
        settling = None
        for i in range(len(norm)):
            if all(abs(abs_target - y) <= band for y in norm[i:]):
                settling = times[i]
                break

        recent = [abs_target - y for (t, y) in zip(times, norm) if times[-1] - t <= 1.0]
        sse = (sum(recent) / len(recent)) if recent else None

        def fmt(v, unit="s"):
            if v is None:
                return "--"
            return f"{v:.3f}{unit}"

        return (
            f"Rise: {fmt(rise)}  "
            f"Overshoot: {overshoot:.1f}%  "
            f"Settling: {fmt(settling)}  "
            f"SSE: {fmt(sse, ' rpm')}"
        )

    def _update_plot(self):
        try:
            mid = int(self.var_motor.get())
        except Exception:
            mid = 1

        with self.telemetry_lock:
            points = list(self.telemetry.get(mid, []))

        if points:
            t_ref = points[-1][0]
            x = [p[0] - t_ref for p in points]
            rpm = [p[1] for p in points]
            amp = [p[2] for p in points]
            target_line = [self.step_target if self.step_active and self.step_motor == mid else 0.0 for _ in x]

            self.line_rpm.set_data(x, rpm)
            self.line_cur.set_data(x, amp)
            self.line_target.set_data(x, target_line)
            self.ax_rpm.set_xlim(min(x), 0.0)

            y_all = rpm + target_line
            y_min = min(y_all) if y_all else -1
            y_max = max(y_all) if y_all else 1
            span = max(20.0, abs(y_max - y_min))
            self.ax_rpm.set_ylim(y_min - 0.15 * span, y_max + 0.15 * span)

            c_min = min(amp) if amp else -1
            c_max = max(amp) if amp else 1
            c_span = max(1.0, c_max - c_min)
            self.ax_cur.set_ylim(c_min - 0.15 * c_span, c_max + 0.15 * c_span)

            self.lbl_metrics.config(text=self._compute_step_metrics(points))
        else:
            self.line_rpm.set_data([], [])
            self.line_cur.set_data([], [])
            self.line_target.set_data([], [])
            self.lbl_metrics.config(text="Rise: --  Overshoot: --  Settling: --  SSE: --")

        self.canvas.draw_idle()
        if self.running:
            self.root.after(120, self._update_plot)

    def save_csv(self):
        if not self.csv_rows:
            messagebox.showwarning("No Data", "No telemetry data to save yet.")
            return

        path = filedialog.asksaveasfilename(
            title="Save Speed PID Log",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return

        try:
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["timestamp", "motor_id", "rpm", "current_a", "pos_deg"])
                with self.telemetry_lock:
                    for row in self.csv_rows:
                        w.writerow(row)
            self.append_log(f"[INFO] Saved CSV: {path}")
        except Exception as e:
            messagebox.showerror("Save Error", str(e))

    def on_close(self):
        self.running = False
        self.stop_sweep_flag = True
        try:
            self.send_command("S")
        except Exception:
            pass
        self.disconnect_serial()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = SpeedPIDTunerApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
