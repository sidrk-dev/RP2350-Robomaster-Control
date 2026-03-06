import tkinter as tk
from tkinter import ttk, messagebox
import json
import os
import serial
import serial.tools.list_ports
import time
import threading
import re
from tkinter import scrolledtext

CONFIG_FILE = "robot_config.json"
BAUD_RATE = 115200

# Default configs
DEFAULT_CONFIG = {
    "motors": {
        "1": {"type": "M3508", "gear_ratio": 94.23076923076923, "kp": 5.0, "ki": 1.0, "kd": 0.0, "aol": 500.0, "skp": 0.3, "ski": 0.0, "skd": 0.0, "sol": 20.0, "min_angle": 0.0, "max_angle": 360.0, "zero_offset": 0.0},
        "2": {"type": "M2006", "gear_ratio": 94.23076923076923, "kp": 5.0, "ki": 1.0, "kd": 0.0, "aol": 500.0, "skp": 0.3, "ski": 0.0, "skd": 0.0, "sol": 20.0, "min_angle": 0.0, "max_angle": 360.0, "zero_offset": 0.0},
        "3": {"type": "M2006", "gear_ratio": 94.23076923076923, "kp": 5.0, "ki": 1.0, "kd": 0.0, "aol": 500.0, "skp": 0.3, "ski": 0.0, "skd": 0.0, "sol": 20.0, "min_angle": 0.0, "max_angle": 360.0, "zero_offset": 0.0},
        "4": {"type": "M2006", "gear_ratio": 1.0, "kp": 5.0, "ki": 1.0, "kd": 0.0, "aol": 500.0, "skp": 0.3, "ski": 0.0, "skd": 0.0, "sol": 20.0, "min_angle": 0.0, "max_angle": 360.0, "zero_offset": 0.0},
        "5": {"type": "M2006", "gear_ratio": 1.0, "kp": 5.0, "ki": 1.0, "kd": 0.0, "aol": 500.0, "skp": 0.3, "ski": 0.0, "skd": 0.0, "sol": 20.0, "min_angle": 0.0, "max_angle": 360.0, "zero_offset": 0.0},
        "6": {"type": "M2006", "gear_ratio": 1.0, "kp": 5.0, "ki": 1.0, "kd": 0.0, "aol": 500.0, "skp": 0.3, "ski": 0.0, "skd": 0.0, "sol": 20.0, "min_angle": 0.0, "max_angle": 360.0, "zero_offset": 0.0}
    }
}

class MotorTab:
    def __init__(self, parent, motor_id, config_data, send_cmd_func):
        self.motor_id = motor_id
        self.config = config_data
        
        # Ensure new defaults exist
        if "skp" not in self.config: self.config["skp"] = 0.3
        if "ski" not in self.config: self.config["ski"] = 0.0
        if "skd" not in self.config: self.config["skd"] = 0.0
        if "aol" not in self.config: self.config["aol"] = 500.0
        if "sol" not in self.config: self.config["sol"] = 20.0
        if "min_angle"   not in self.config: self.config["min_angle"]   = 0.0
        if "max_angle"   not in self.config: self.config["max_angle"]   = 360.0
        if "zero_offset" not in self.config: self.config["zero_offset"] = 0.0
            
        self.send_cmd = send_cmd_func
        
        self.frame = ttk.Frame(parent, padding=10)
        
        # --- Real-Time Telemetry ---
        lf_telemetry = ttk.LabelFrame(self.frame, text="Real-Time Telemetry", padding=10)
        lf_telemetry.pack(fill="x", pady=5)
        
        ttk.Label(lf_telemetry, text="Current Angle:", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="e", padx=5)
        self.lbl_current_angle = ttk.Label(lf_telemetry, text="---", foreground="blue", font=("Arial", 10, "bold"))
        self.lbl_current_angle.grid(row=0, column=1, sticky="w")
        
        ttk.Label(lf_telemetry, text="Target Angle:", font=("Arial", 10, "bold")).grid(row=0, column=2, sticky="e", padx=(20, 5))
        self.lbl_target_angle = ttk.Label(lf_telemetry, text="---", font=("Arial", 10))
        self.lbl_target_angle.grid(row=0, column=3, sticky="w")

        ttk.Label(lf_telemetry, text="Velocity:", font=("Arial", 10, "bold")).grid(row=1, column=0, sticky="e", padx=5)
        self.lbl_current_rpm = ttk.Label(lf_telemetry, text="--- RPM", foreground="green", font=("Arial", 10, "bold"))
        self.lbl_current_rpm.grid(row=1, column=1, sticky="w")
        
        ttk.Label(lf_telemetry, text="Abs. Encoder:", font=("Arial", 10, "bold")).grid(row=1, column=2, sticky="e", padx=(20, 5))
        self.lbl_abs_angle = ttk.Label(lf_telemetry, text="---", foreground="#8B4513", font=("Arial", 10, "bold"))
        self.lbl_abs_angle.grid(row=1, column=3, sticky="w")

        btn_zero = ttk.Button(lf_telemetry, text="Set Zero Position (CAL)", command=self.set_zero)
        btn_zero.grid(row=0, column=4, padx=(20, 0))

        # --- Motor Specs ---
        lf_specs = ttk.LabelFrame(self.frame, text="Motor Specifications", padding=10)
        lf_specs.pack(fill="x", pady=5)
        
        ttk.Label(lf_specs, text="Motor Type:").grid(row=0, column=0, sticky="w", pady=2)
        self.var_type = tk.StringVar(value=self.config["type"])
        cb_type = ttk.Combobox(lf_specs, textvariable=self.var_type, values=["M2006", "M3508"], state="readonly", width=10)
        cb_type.grid(row=0, column=1, sticky="w", pady=2, padx=5)
        cb_type.bind("<<ComboboxSelected>>", self.on_change)
        
        ttk.Label(lf_specs, text="External Gear Ratio:").grid(row=1, column=0, sticky="w", pady=2)
        self.var_gear = tk.StringVar(value=str(self.config["gear_ratio"]))
        en_gear = ttk.Entry(lf_specs, textvariable=self.var_gear, width=20)
        en_gear.grid(row=1, column=1, sticky="w", pady=2, padx=5)
        en_gear.bind("<KeyRelease>", self.on_change)
        
        btn_apply_gear = ttk.Button(lf_specs, text="Apply Gear to Arm", command=self.apply_gear)
        btn_apply_gear.grid(row=1, column=2, padx=10)
        
        # --- PID Frames Container ---
        pid_container = ttk.Frame(self.frame)
        pid_container.pack(fill="x", pady=5)
        
        # --- Angle PID ---
        lf_pid = ttk.LabelFrame(pid_container, text="Angle PID Tuning", padding=10)
        lf_pid.pack(side="left", fill="x", expand=True, padx=(0, 5))
        
        self.var_kp = tk.DoubleVar(value=self.config["kp"])
        self.var_ki = tk.DoubleVar(value=self.config["ki"])
        self.var_kd = tk.DoubleVar(value=self.config["kd"])
        self.var_aol = tk.DoubleVar(value=self.config["aol"])
        
        self.create_entry(lf_pid, "Kp:", self.var_kp, row=0)
        self.create_entry(lf_pid, "Ki:", self.var_ki, row=1)
        self.create_entry(lf_pid, "Kd:", self.var_kd, row=2)
        self.create_entry(lf_pid, "Max Speed (RPM):", self.var_aol, row=3)
        
        ttk.Button(lf_pid, text="Apply Angle PID", command=self.apply_angle_pid).grid(row=4, column=0, columnspan=2, pady=10)
        
        # --- Speed PID ---
        lf_spid = ttk.LabelFrame(pid_container, text="Speed PID Tuning", padding=10)
        lf_spid.pack(side="right", fill="x", expand=True, padx=(5, 0))
        
        self.var_skp = tk.DoubleVar(value=self.config["skp"])
        self.var_ski = tk.DoubleVar(value=self.config["ski"])
        self.var_skd = tk.DoubleVar(value=self.config["skd"])
        self.var_sol = tk.DoubleVar(value=self.config["sol"])
        
        self.create_entry(lf_spid, "Kp:", self.var_skp, row=0)
        self.create_entry(lf_spid, "Ki:", self.var_ski, row=1)
        self.create_entry(lf_spid, "Kd:", self.var_skd, row=2)
        self.create_entry(lf_spid, "Max Current (A):", self.var_sol, row=3)
        
        ttk.Button(lf_spid, text="Apply Speed PID", command=self.apply_speed_pid).grid(row=4, column=0, columnspan=2, pady=10)
        
        # --- Test Controls ---
        lf_test = ttk.LabelFrame(self.frame, text="Testing Controls", padding=10)
        lf_test.pack(fill="x", pady=5)
        
        ttk.Button(lf_test, text="Move to 0°",   command=lambda: self.test_move(0)).pack(side="left", padx=5)
        ttk.Button(lf_test, text="Move to 90°",  command=lambda: self.test_move(90)).pack(side="left", padx=5)
        ttk.Button(lf_test, text="Move to 180°", command=lambda: self.test_move(180)).pack(side="left", padx=5)
        ttk.Button(lf_test, text="Move to 270°", command=lambda: self.test_move(270)).pack(side="left", padx=5)
        ttk.Button(lf_test, text="STOP", command=lambda: self.send_cmd("S")).pack(side="right", padx=5)

        # --- Jog Controls ---
        lf_jog = ttk.LabelFrame(self.frame, text="Jog & Zero", padding=10)
        lf_jog.pack(fill="x", pady=5)

        ttk.Button(lf_jog, text="-10°", command=lambda: self.jog(-10)).pack(side="left", padx=4)
        ttk.Button(lf_jog, text="-1°",  command=lambda: self.jog(-1)).pack(side="left", padx=4)
        ttk.Button(lf_jog, text="+1°",  command=lambda: self.jog(+1)).pack(side="left", padx=4)
        ttk.Button(lf_jog, text="+10°", command=lambda: self.jog(+10)).pack(side="left", padx=4)
        ttk.Separator(lf_jog, orient="vertical").pack(side="left", fill="y", padx=8)
        ttk.Button(lf_jog, text="Set Zero Here (CAL)", command=self.set_zero).pack(side="left", padx=4)

        # --- Joint Limits & Zero Offset ---
        lf_limits = ttk.LabelFrame(self.frame, text="Joint Limits & Zero Offset", padding=10)
        lf_limits.pack(fill="x", pady=5)

        self.var_min   = tk.DoubleVar(value=self.config["min_angle"])
        self.var_max   = tk.DoubleVar(value=self.config["max_angle"])
        self.var_zoff  = tk.DoubleVar(value=self.config["zero_offset"])

        ttk.Label(lf_limits, text="Min Angle (°):").grid(row=0, column=0, sticky="e", padx=5, pady=3)
        ttk.Entry(lf_limits, textvariable=self.var_min,  width=10).grid(row=0, column=1, sticky="w", padx=5, pady=3)
        ttk.Label(lf_limits, text="Max Angle (°):").grid(row=0, column=2, sticky="e", padx=5, pady=3)
        ttk.Entry(lf_limits, textvariable=self.var_max,  width=10).grid(row=0, column=3, sticky="w", padx=5, pady=3)
        ttk.Label(lf_limits, text="Zero Offset (°):", font=("Arial", 9)).grid(row=1, column=0, sticky="e", padx=5, pady=3)
        ttk.Entry(lf_limits, textvariable=self.var_zoff, width=10).grid(row=1, column=1, sticky="w", padx=5, pady=3)
        ttk.Label(lf_limits, text="Motor deg when IK=0°", font=("Arial", 8), foreground="gray").grid(row=1, column=2, columnspan=2, sticky="w")
        ttk.Button(lf_limits, text="Apply Limits", command=self.apply_limits).grid(row=0, column=4, rowspan=2, padx=10)

        # --- Visual Angle Gauge ---
        lf_vis = ttk.LabelFrame(self.frame, text="Joint Range Visualization", padding=5)
        lf_vis.pack(fill="x", pady=5)

        self._gauge_size = 130
        self._gauge_canvas = tk.Canvas(lf_vis, width=self._gauge_size, height=self._gauge_size,
                                        bg="white", highlightthickness=0)
        self._gauge_canvas.pack(side="left", padx=10)
        self._gauge_current_deg = 0.0  # updated by telemetry

        # Legend next to gauge
        legend = ttk.Frame(lf_vis)
        legend.pack(side="left", padx=10, anchor="n")
        ttk.Label(legend, text="● Gray = full 360°",   foreground="gray",   font=("Arial", 8)).pack(anchor="w")
        ttk.Label(legend, text="● Green = allowed",     foreground="#2d8a2d", font=("Arial", 8)).pack(anchor="w")
        ttk.Label(legend, text="● Blue = current pos",  foreground="blue",   font=("Arial", 8)).pack(anchor="w")
        ttk.Label(legend, text="● Red = zero (CAL 0°)", foreground="red",    font=("Arial", 8)).pack(anchor="w")

        self._draw_gauge()  # initial draw
        # Redraw periodically
        self._gauge_canvas.after(200, self._gauge_tick)


    def create_entry(self, parent, label, variable, row):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="e", padx=5, pady=5)
        
        def entry_changed(*args):
            try:
                variable.get()  # will throw TclError if empty
                self.on_change()
            except Exception:
                pass  # ignore empty field / invalid input during typing
                
        # To avoid infinite loop, use trace sparingly or bind
        entry = ttk.Entry(parent, textvariable=variable, width=15)
        entry.grid(row=row, column=1, padx=5, pady=5, sticky="w")
        entry.bind("<KeyRelease>", entry_changed)

    def on_change(self, *args):
        self.config["type"] = self.var_type.get()
        try:
            self.config["gear_ratio"]  = float(self.var_gear.get())
            self.config["kp"]          = self.var_kp.get()
            self.config["ki"]          = self.var_ki.get()
            self.config["kd"]          = self.var_kd.get()
            self.config["aol"]         = self.var_aol.get()
            self.config["skp"]         = self.var_skp.get()
            self.config["ski"]         = self.var_ski.get()
            self.config["skd"]         = self.var_skd.get()
            self.config["sol"]         = self.var_sol.get()
            self.config["min_angle"]   = self.var_min.get()
            self.config["max_angle"]   = self.var_max.get()
            self.config["zero_offset"] = self.var_zoff.get()
        except ValueError:
            pass

    def apply_gear(self):
        try:
            gr = float(self.var_gear.get())
            self.send_cmd(f"GEAR {self.motor_id} {gr}")
        except ValueError:
            messagebox.showerror("Error", "Invalid Gear Ratio")

    def apply_angle_pid(self):
        kp = self.var_kp.get()
        ki = self.var_ki.get()
        kd = self.var_kd.get()
        aol = self.var_aol.get()
        mid = self.motor_id
        batch = f"KP {mid} {kp:.4f}; KI {mid} {ki:.4f}; KD {mid} {kd:.4f}; AOL {mid} {aol:.2f}"
        self.send_cmd(batch)

    def apply_speed_pid(self):
        skp = self.var_skp.get()
        ski = self.var_ski.get()
        skd = self.var_skd.get()
        sol = self.var_sol.get()
        mid = self.motor_id
        batch = f"SKP {mid} {skp:.4f}; SKI {mid} {ski:.4f}; SKD {mid} {skd:.4f}; SOL {mid} {sol:.2f}"
        self.send_cmd(batch)

    def test_move(self, angle):
        self.send_cmd(f"P {self.motor_id} {angle}")
        self.lbl_target_angle.config(text=f"{angle}°")
        
    def jog(self, delta):
        """Jog motor by delta degrees using relative PR command."""
        self.send_cmd(f"PR {self.motor_id} {delta}")

    def set_zero(self):
        self.send_cmd(f"CAL {self.motor_id} 0")
        self.lbl_target_angle.config(text="0°")

    def apply_limits(self):
        mid = self.motor_id
        mn  = self.var_min.get()
        mx  = self.var_max.get()
        self.on_change()  # persist to config
        self.send_cmd(f"JLIM {mid} {mn:.1f} {mx:.1f}")

    def _draw_gauge(self):
        import math
        c = self._gauge_canvas
        s = self._gauge_size
        c.delete("all")
        cx, cy = s / 2, s / 2
        r = s / 2 - 10

        try:
            mn = self.var_min.get()
            mx = self.var_max.get()
            zoff = self.var_zoff.get()
        except Exception:
            mn, mx, zoff = 0, 360, 0

        # Gray circle (full 360°)
        c.create_oval(cx - r, cy - r, cx + r, cy + r, outline="#ddd", width=8)

        # Green arc for allowed range (tkinter arcs go counter-clockwise from 3-o'clock)
        # Convert our 0=top, CW convention to tkinter's 0=right, CCW convention
        start_tk = 90 - mn  # min angle mapped
        extent_tk = -(mx - mn)  # negative = clockwise in tkinter
        if abs(extent_tk) > 0.1:
            c.create_arc(cx - r, cy - r, cx + r, cy + r,
                         start=start_tk, extent=extent_tk,
                         style="arc", outline="#2d8a2d", width=8)

        # Tick marks at 0, 90, 180, 270
        for deg in [0, 90, 180, 270]:
            rad = math.radians(deg - 90)  # -90 so 0=top
            x1 = cx + (r - 6) * math.cos(rad)
            y1 = cy + (r - 6) * math.sin(rad)
            x2 = cx + (r + 3) * math.cos(rad)
            y2 = cy + (r + 3) * math.sin(rad)
            c.create_line(x1, y1, x2, y2, fill="#999", width=1)
            lx = cx + (r + 9) * math.cos(rad)
            ly = cy + (r + 9) * math.sin(rad)
            c.create_text(lx, ly, text=str(deg), font=("Arial", 6), fill="#999")

        # Red tick for zero offset position
        rad_z = math.radians(zoff - 90)
        zx1 = cx + (r - 10) * math.cos(rad_z)
        zy1 = cy + (r - 10) * math.sin(rad_z)
        zx2 = cx + (r + 2) * math.cos(rad_z)
        zy2 = cy + (r + 2) * math.sin(rad_z)
        c.create_line(zx1, zy1, zx2, zy2, fill="red", width=2)

        # Blue needle for current position
        cur = self._gauge_current_deg
        rad_c = math.radians(cur - 90)  # -90 so 0=top
        nx = cx + (r - 14) * math.cos(rad_c)
        ny = cy + (r - 14) * math.sin(rad_c)
        c.create_line(cx, cy, nx, ny, fill="blue", width=2, arrow="last")

        # Center dot
        c.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill="blue")

    def _gauge_tick(self):
        try:
            self._draw_gauge()
        except Exception:
            pass
        self._gauge_canvas.after(250, self._gauge_tick)



class PIDTunerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("RoboMaster PID & Configuration Tuner")
        self.root.geometry("800x650")
        
        self.config = self.load_config()
        self.ser = None
        self.tabs = {}
        self.running = True
        
        self.create_widgets()
        
    def load_config(self):
        if os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE, 'r') as f:
                    return json.load(f)
            except Exception as e:
                messagebox.showwarning("Warning", f"Failed to load config: {e}\nUsing defaults.")
        return DEFAULT_CONFIG.copy()

    def save_config(self):
        try:
            with open(CONFIG_FILE, 'w') as f:
                json.dump(self.config, f, indent=4)
            messagebox.showinfo("Success", "Configuration saved successfully!")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save config: {e}")

    def create_widgets(self):
        sw = self.root.winfo_screenwidth()
        sh = self.root.winfo_screenheight()
        w = max(900, int(sw * 0.80))
        h = max(650, int(sh * 0.85))
        self.root.geometry(f"{w}x{h}")
        self.root.minsize(820, 400)
        self.root.resizable(True, True)

        # --- Fixed bottom area (always visible) ---
        self.status_bar = ttk.Label(self.root,
            text="Ready. Note: Close 'ik_arm_control.py' before connecting via USB.",
            relief="sunken", anchor="w")
        self.status_bar.pack(fill="x", side="bottom", pady=(2, 0))

        bottom_frame = ttk.Frame(self.root, padding=(10, 5))
        bottom_frame.pack(fill="x", side="bottom")
        ttk.Button(bottom_frame, text="Save to File",             command=self.save_config).pack(side="left",  padx=5)
        ttk.Button(bottom_frame, text="Apply ALL Settings to Arm", command=self.apply_all).pack(side="right", padx=5)

        # --- Fixed top bar (always visible) ---
        top_frame = ttk.Frame(self.root, padding=10)
        top_frame.pack(fill="x", side="top")
        ttk.Label(top_frame, text="6-DoF Arm Configuration", font=("Arial", 16, "bold")).pack(side="left")
        self.btn_connect = ttk.Button(top_frame, text="Connect to Arm", command=self.toggle_serial)
        self.btn_connect.pack(side="right")
        self.lbl_status = ttk.Label(top_frame, text="Disconnected", foreground="red")
        self.lbl_status.pack(side="right", padx=10)

        # --- Scrollable canvas fills remaining space ---
        scroll_container = ttk.Frame(self.root)
        scroll_container.pack(fill="both", expand=True)

        vsb = ttk.Scrollbar(scroll_container, orient="vertical")
        vsb.pack(side="right", fill="y")

        self._canvas = tk.Canvas(scroll_container, yscrollcommand=vsb.set,
                                 bd=0, highlightthickness=0)
        self._canvas.pack(side="left", fill="both", expand=True)
        vsb.configure(command=self._canvas.yview)

        # Content frame inside canvas
        content = ttk.Frame(self._canvas)
        content_id = self._canvas.create_window((0, 0), window=content, anchor="nw")

        def _on_frame_cfg(e):
            self._canvas.configure(scrollregion=self._canvas.bbox("all"))
        content.bind("<Configure>", _on_frame_cfg)

        def _on_canvas_cfg(e):
            self._canvas.itemconfig(content_id, width=e.width)
        self._canvas.bind("<Configure>", _on_canvas_cfg)

        def _on_mousewheel(e):
            # Don't steal scroll from inner text widgets (e.g., log box)
            if isinstance(e.widget, tk.Text):
                return
            self._canvas.yview_scroll(int(-1 * (e.delta / 120)), "units")
        self._canvas.bind_all("<MouseWheel>", _on_mousewheel)

        # --- Notebook (full natural height — no expand) ---
        self.notebook = ttk.Notebook(content)
        self.notebook.pack(fill="x", padx=10, pady=5)

        for i in range(1, 7):
            mid = str(i)
            if mid not in self.config["motors"]:
                self.config["motors"][mid] = DEFAULT_CONFIG["motors"][mid].copy()
            tab = MotorTab(self.notebook, mid, self.config["motors"][mid], self.send_command)
            self.tabs[mid] = tab
            self.notebook.add(tab.frame, text=f"Motor {mid}")

        # --- Serial log (also scrollable with the page) ---
        lf_log = ttk.LabelFrame(content, text="Serial Log", padding=5)
        lf_log.pack(fill="x", padx=10, pady=(0, 10))

        log_top = ttk.Frame(lf_log)
        log_top.pack(fill="x")
        ttk.Label(log_top, text="Raw output from XIAO:", font=("Arial", 8)).pack(side="left")
        ttk.Button(log_top, text="Clear", command=self.clear_log).pack(side="right")

        self.log_box = scrolledtext.ScrolledText(
            lf_log, height=8, state="disabled", wrap="word",
            font=("Courier", 8), bg="#1e1e1e", fg="#d4d4d4",
            insertbackground="white"
        )
        self.log_box.pack(fill="x", pady=(4, 0))
        self.log_box.tag_config("telemetry", foreground="#6db3f2")
        self.log_box.tag_config("error",     foreground="#f44747")
        self.log_box.tag_config("info",      foreground="#b5cea8")
        self.log_box.tag_config("sent",      foreground="#dca54e")
        self._log_line_count = 0
    def find_serial_port(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if "CH340" in port.description or "Arduino" in port.description or "Serial" in port.description:
                return port.device
        return ports[0].device if ports else None

    def toggle_serial(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None
            self.btn_connect.config(text="Connect to Arm")
            self.lbl_status.config(text="Disconnected", foreground="red")
            self.status_bar.config(text="Disconnected from serial.")
        else:
            port = self.find_serial_port()
            if not port:
                messagebox.showerror("No Device", "Could not find a connected Arduino device.")
                return
                
            try:
                self.ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
                time.sleep(2)
                self.btn_connect.config(text="Disconnect")
                self.lbl_status.config(text=f"Connected ({port})", foreground="green")
                self.status_bar.config(text=f"Connected successfully to {port} @ {BAUD_RATE} baud.")
                
                # Start background serial read thread first
                threading.Thread(target=self.serial_read_loop, daemon=True).start()
                time.sleep(0.15)  # Give thread a moment to start
                
                # TON is idempotent (unlike T toggle) — safe to call on every connect
                self.send_command("TON")
                
                # Auto-apply saved configuration so Arduino matches the saved PID/limits
                time.sleep(0.1)
                self.apply_all()
            except Exception as e:
                messagebox.showerror("Connection Error", f"Could not connect to {port}:\n{e}")

    def serial_read_loop(self):
        while self.running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue
                    
                    # Append raw line to the log panel (thread-safe)
                    self.root.after(0, self.log_append, line)
                        
                    # Parse status "M1 POS:0.0 Tot:0.0 RPM:0.0 A:0.00 |"
                    # We can use regex or simple splits
                    for token in line.split("|"):
                        token = token.strip()
                        if token.startswith("M"):
                            match = re.search(r"M(\d+)\s+POS:([-\d.]+).*?RPM:([-\d.]+)", token)
                            if match:
                                mid = match.group(1)
                                pos = match.group(2)
                                rpm = match.group(3)
                                # Capture optional APOS (absolute encoder)
                                apos_match = re.search(r"APOS:([-\d.]+)", token)
                                apos = apos_match.group(1) if apos_match else None
                                if mid in self.tabs:
                                    self.root.after(0, self.update_telemetry_ui, mid, pos, rpm, apos)
                                    
                time.sleep(0.01)
            except Exception:
                break
                
    def update_telemetry_ui(self, mid, pos, rpm="---", apos=None):
        if mid in self.tabs:
            tab = self.tabs[mid]
            tab.lbl_current_angle.config(text=f"{pos}°")
            tab.lbl_current_rpm.config(text=f"{rpm} RPM")
            if apos is not None:
                tab.lbl_abs_angle.config(text=f"{apos}°")
            else:
                tab.lbl_abs_angle.config(text="N/A")
            try:
                tab._gauge_current_deg = float(pos)
            except ValueError:
                pass

    def clear_log(self):
        self.log_box.config(state="normal")
        self.log_box.delete("1.0", "end")
        self.log_box.config(state="disabled")
        self._log_line_count = 0

    def log_append(self, line):
        MAX_LINES = 200
        if line.startswith("M") and "POS:" in line:
            tag = "telemetry"
        elif "Error" in line or "error" in line or "blocked" in line:
            tag = "error"
        else:
            tag = "info"
        self.log_box.config(state="normal")
        self.log_box.insert("end", line + "\n", tag)
        self._log_line_count += 1
        if self._log_line_count > MAX_LINES:
            self.log_box.delete("1.0", "2.0")
            self._log_line_count -= 1
        self.log_box.see("end")
        self.log_box.config(state="disabled")

    def send_command(self, cmd):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((cmd + '\n').encode('utf-8'))
                self.status_bar.config(text=f"Sent: {cmd}")
                self.root.after(0, self._log_sent, cmd)
            except Exception as e:
                messagebox.showerror("Serial Error", f"Failed to send data:\n{e}")
                self.toggle_serial()
        else:
            self.status_bar.config(text=f"Not connected! (Would have sent: {cmd})")

    def _log_sent(self, cmd):
        self.log_box.config(state="normal")
        self.log_box.insert("end", f">>> SENT: {cmd}\n", "sent")
        self._log_line_count += 1
        if self._log_line_count > 200:
            self.log_box.delete("1.0", "2.0")
            self._log_line_count -= 1
        self.log_box.see("end")
        self.log_box.config(state="disabled")

    def apply_all(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("Not Connected", "Please connect to the arm first to apply settings alive.")
            return
            
        for mid, tab in self.tabs.items():
            # Batch all commands for this motor into one semicolon-separated string
            gr   = tab.config.get("gear_ratio", 1.0)
            mn   = tab.var_min.get(); mx = tab.var_max.get()
            kp   = tab.var_kp.get();  ki  = tab.var_ki.get();  kd  = tab.var_kd.get();  aol = tab.var_aol.get()
            skp  = tab.var_skp.get(); ski = tab.var_ski.get(); skd = tab.var_skd.get(); sol = tab.var_sol.get()
            batch = (
                f"GEAR {mid} {gr}"
                f"; JLIM {mid} {mn:.1f} {mx:.1f}"
                f"; KP {mid} {kp:.4f}; KI {mid} {ki:.4f}; KD {mid} {kd:.4f}; AOL {mid} {aol:.2f}"
                f"; SKP {mid} {skp:.4f}; SKI {mid} {ski:.4f}; SKD {mid} {skd:.4f}; SOL {mid} {sol:.2f}"
            )
            self.send_command(batch)
            time.sleep(0.05)
            
        self.status_bar.config(text="All Gear Ratios and PIDs applied to Arm!")

    def on_close(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.root.destroy()

if __name__ == "__main__":
    if not os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, 'w') as f:
            json.dump(DEFAULT_CONFIG, f, indent=4)
            
    root = tk.Tk()
    app = PIDTunerApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    
    root.lift()
    root.attributes('-topmost',True)
    root.after_idle(root.attributes,'-topmost',False)
    
    root.mainloop()
