import tkinter as tk
from tkinter import ttk, messagebox
import json
import os
import serial
import serial.tools.list_ports
import time
import threading
import re

CONFIG_FILE = "robot_config.json"
BAUD_RATE = 115200

# Default configs
DEFAULT_CONFIG = {
    "motors": {
        "1": {"type": "M3508", "gear_ratio": 94.23076923076923, "kp": 5.0, "ki": 1.0, "kd": 0.0, "skp": 0.3, "ski": 0.0, "skd": 0.0},
        "2": {"type": "M2006", "gear_ratio": 94.23076923076923, "kp": 5.0, "ki": 1.0, "kd": 0.0, "skp": 0.3, "ski": 0.0, "skd": 0.0},
        "3": {"type": "M2006", "gear_ratio": 94.23076923076923, "kp": 5.0, "ki": 1.0, "kd": 0.0, "skp": 0.3, "ski": 0.0, "skd": 0.0},
        "4": {"type": "M2006", "gear_ratio": 1.0, "kp": 5.0, "ki": 1.0, "kd": 0.0, "skp": 0.3, "ski": 0.0, "skd": 0.0},
        "5": {"type": "M2006", "gear_ratio": 1.0, "kp": 5.0, "ki": 1.0, "kd": 0.0, "skp": 0.3, "ski": 0.0, "skd": 0.0},
        "6": {"type": "M2006", "gear_ratio": 1.0, "kp": 5.0, "ki": 1.0, "kd": 0.0, "skp": 0.3, "ski": 0.0, "skd": 0.0}
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
        
        self.create_entry(lf_pid, "Kp:", self.var_kp, row=0)
        self.create_entry(lf_pid, "Ki:", self.var_ki, row=1)
        self.create_entry(lf_pid, "Kd:", self.var_kd, row=2)
        
        ttk.Button(lf_pid, text="Apply Angle PID", command=self.apply_angle_pid).grid(row=3, column=0, columnspan=2, pady=10)
        
        # --- Speed PID ---
        lf_spid = ttk.LabelFrame(pid_container, text="Speed PID Tuning", padding=10)
        lf_spid.pack(side="right", fill="x", expand=True, padx=(5, 0))
        
        self.var_skp = tk.DoubleVar(value=self.config["skp"])
        self.var_ski = tk.DoubleVar(value=self.config["ski"])
        self.var_skd = tk.DoubleVar(value=self.config["skd"])
        
        self.create_entry(lf_spid, "Kp:", self.var_skp, row=0)
        self.create_entry(lf_spid, "Ki:", self.var_ski, row=1)
        self.create_entry(lf_spid, "Kd:", self.var_skd, row=2)
        
        ttk.Button(lf_spid, text="Apply Speed PID", command=self.apply_speed_pid).grid(row=3, column=0, columnspan=2, pady=10)
        
        # --- Test Controls ---
        lf_test = ttk.LabelFrame(self.frame, text="Testing Controls", padding=10)
        lf_test.pack(fill="x", pady=5)
        
        ttk.Button(lf_test, text="Move to 0°", command=lambda: self.test_move(0)).pack(side="left", padx=5)
        ttk.Button(lf_test, text="Move to 90°", command=lambda: self.test_move(90)).pack(side="left", padx=5)
        ttk.Button(lf_test, text="Move to 180°", command=lambda: self.test_move(180)).pack(side="left", padx=5)
        ttk.Button(lf_test, text="Move to 270°", command=lambda: self.test_move(270)).pack(side="left", padx=5)
        ttk.Button(lf_test, text="STOP", command=lambda: self.send_cmd("S")).pack(side="right", padx=5)

    def create_entry(self, parent, label, variable, row):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="e", padx=5, pady=5)
        
        def entry_changed(*args):
            try:
                # Just to validate it's a number, but we don't clamp it
                float(variable.get())
                self.on_change()
            except ValueError:
                pass
                
        # To avoid infinite loop, use trace sparingly or bind
        entry = ttk.Entry(parent, textvariable=variable, width=15)
        entry.grid(row=row, column=1, padx=5, pady=5, sticky="w")
        entry.bind("<KeyRelease>", entry_changed)

    def on_change(self, *args):
        self.config["type"] = self.var_type.get()
        try:
            self.config["gear_ratio"] = float(self.var_gear.get())
            self.config["kp"] = self.var_kp.get()
            self.config["ki"] = self.var_ki.get()
            self.config["kd"] = self.var_kd.get()
            self.config["skp"] = self.var_skp.get()
            self.config["ski"] = self.var_ski.get()
            self.config["skd"] = self.var_skd.get()
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
        self.send_cmd(f"AP {self.motor_id} {kp:.2f} {ki:.2f} {kd:.2f}")

    def apply_speed_pid(self):
        skp = self.var_skp.get()
        ski = self.var_ski.get()
        skd = self.var_skd.get()
        self.send_cmd(f"SP {self.motor_id} {skp:.2f} {ski:.2f} {skd:.2f}")

    def test_move(self, angle):
        self.send_cmd(f"P {self.motor_id} {angle}")
        self.lbl_target_angle.config(text=f"{angle}°")
        
    def set_zero(self):
        self.send_cmd(f"CAL {self.motor_id} 0")
        self.lbl_target_angle.config(text="0°")


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
        top_frame = ttk.Frame(self.root, padding=10)
        top_frame.pack(fill="x")
        
        lbl_title = ttk.Label(top_frame, text="6-DoF Arm Configuration", font=("Arial", 16, "bold"))
        lbl_title.pack(side="left")
        
        self.btn_connect = ttk.Button(top_frame, text="Connect to Arm", command=self.toggle_serial)
        self.btn_connect.pack(side="right")
        
        self.lbl_status = ttk.Label(top_frame, text="Disconnected", foreground="red")
        self.lbl_status.pack(side="right", padx=10)
        
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill="both", expand=True, padx=10, pady=5)
        
        for i in range(1, 7):
            mid = str(i)
            if mid not in self.config["motors"]:
                self.config["motors"][mid] = DEFAULT_CONFIG["motors"][mid].copy()
                
            tab = MotorTab(self.notebook, mid, self.config["motors"][mid], self.send_command)
            self.tabs[mid] = tab
            self.notebook.add(tab.frame, text=f"Motor {mid}")
            
        bottom_frame = ttk.Frame(self.root, padding=10)
        bottom_frame.pack(fill="x")
        
        ttk.Button(bottom_frame, text="Save to File", command=self.save_config).pack(side="left", padx=5)
        ttk.Button(bottom_frame, text="Apply ALL Settings to Arm", command=self.apply_all).pack(side="right", padx=5)
        
        self.status_bar = ttk.Label(self.root, text="Ready. Note: Close 'ik_arm_control.py' before connecting via USB.", relief="sunken", anchor="w")
        self.status_bar.pack(fill="x", side="bottom", pady=(5,0))

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
                
                # Start background serial read thread
                threading.Thread(target=self.serial_read_loop, daemon=True).start()
            except Exception as e:
                messagebox.showerror("Connection Error", f"Could not connect to {port}:\n{e}")

    def serial_read_loop(self):
        while self.running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue
                        
                    # Parse status "M1 POS:0.0 Tot:0.0 RPM:0.0 A:0.00 |"
                    # We can use regex or simple splits
                    for token in line.split("|"):
                        token = token.strip()
                        if token.startswith("M"):
                            # Parse "M1 POS:90.0 ..."
                            match = re.search(r"M(\d+)\s+POS:([-\d.]+)", token)
                            if match:
                                mid = match.group(1)
                                pos = match.group(2)
                                if mid in self.tabs:
                                    # Update UI thread-safely
                                    self.root.after(0, self.update_telemetry_ui, mid, pos)
                                    
                time.sleep(0.01)
            except Exception:
                break
                
    def update_telemetry_ui(self, mid, pos):
        if mid in self.tabs:
            self.tabs[mid].lbl_current_angle.config(text=f"{pos}°")

    def send_command(self, cmd):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((cmd + '\n').encode('utf-8'))
                self.status_bar.config(text=f"Sent: {cmd}")
            except Exception as e:
                messagebox.showerror("Serial Error", f"Failed to send data:\n{e}")
                self.toggle_serial()
        else:
            self.status_bar.config(text=f"Not connected! (Would have sent: {cmd})")

    def apply_all(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("Not Connected", "Please connect to the arm first to apply settings alive.")
            return
            
        for mid, tab in self.tabs.items():
            tab.apply_gear()
            time.sleep(0.05)
            tab.apply_angle_pid()
            time.sleep(0.05)
            tab.apply_speed_pid()
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
