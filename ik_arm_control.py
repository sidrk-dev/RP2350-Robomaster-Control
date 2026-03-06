import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, CheckButtons
import serial
import serial.tools.list_ports
import time
import threading
import sys
import traceback
import tempfile
import os
import json

CONFIG_FILE = "robot_config.json"
robot_config = {}

try:
    from ikpy.chain import Chain
except ImportError:
    print("Error: ikpy or its dependencies not found.")
    print("Please install requirements by running:")
    print("pip install ikpy pyserial matplotlib numpy scipy")
    sys.exit(1)

try:
    from pynput import keyboard
except ImportError:
    print("Error: pynput library not found.")
    print("Please install it running: pip install pynput")
    sys.exit(1)

# --- Configuration ---
BAUD_RATE = 115200
URDF_FILE = "URDF/URDF.urdf"

# Disable default matplotlib key bindings to prevent interference with WASD Controls
for key in list(plt.rcParams.keys()):
    if key.startswith('keymap.'):
        plt.rcParams[key] = []

# --- Globals ---
ser = None
my_chain = None
current_target_xyz = [0.2, 0.0, 0.3]  # Default safe position
current_target_rpy = [0.0, 0.0, 0.0]  # Roll, Pitch, Yaw
current_joint_angles = [0.0]*6
actual_joint_angles = None
is_previewing = False
preview_start_time = 0
PREVIEW_DURATION = 2.5 # seconds for preview animation
live_update = False
last_send_time = 0
cam_elev = 20.0
cam_azim = -60.0
arm_line = None
ghost_line = None
target_point = None
joint_texts = []
arm_line_top = None
ghost_line_top = None
target_point_top = None
arm_line_side = None
ghost_line_side = None
target_point_side = None
ax_top = None
ax_side = None

# --- Serial Functions ---
def find_serial_port():
    ports = serial.tools.list_ports.comports()
    if not ports:
        return None
    for port in ports:
        if "CH340" in port.description or "Arduino" in port.description or "Serial" in port.description:
            return port.device
    return ports[0].device

def connect_serial():
    global ser, robot_config
    print("Looking for Arduino serial port...")
    port_name = find_serial_port()
    if not port_name:
        print("No serial ports found! Please plug in your device.")
        return False

    print(f"Connecting to {port_name}...")
    try:
        ser = serial.Serial(port_name, BAUD_RATE, timeout=0.1)
        time.sleep(2) # Wait for reset
        print("Connected successfully!")
        
        # Start a thread to read logs
        read_thread = threading.Thread(target=read_from_port, daemon=True)
        read_thread.start()
        
        # Load and apply Robot Config
        try:
            if os.path.exists(CONFIG_FILE):
                with open(CONFIG_FILE, 'r') as f:
                    data = json.load(f)
                    if "motors" in data:
                        robot_config = data["motors"]
            
            print("Applying robot_config.json settings to Arm hardware...")
            for i in range(1, 7):
                mid = str(i)
                if mid in robot_config:
                    gr  = robot_config[mid].get("gear_ratio", 1.0)
                    kp  = robot_config[mid].get("kp",  5.0)
                    ki  = robot_config[mid].get("ki",  1.0)
                    kd  = robot_config[mid].get("kd",  0.0)
                    aol = robot_config[mid].get("aol", 500.0)
                    skp = robot_config[mid].get("skp", 0.3)
                    ski = robot_config[mid].get("ski", 0.0)
                    skd = robot_config[mid].get("skd", 0.0)
                    sol = robot_config[mid].get("sol", 20.0)
                    mn  = robot_config[mid].get("min_angle",  0.0)
                    mx  = robot_config[mid].get("max_angle",  360.0)

                    batch = (
                        f"GEAR {i} {gr}"
                        f"; JLIM {i} {mn:.1f} {mx:.1f}"
                        f"; KP {i} {kp:.4f}; KI {i} {ki:.4f}; KD {i} {kd:.4f}; AOL {i} {aol:.2f}"
                        f"; SKP {i} {skp:.4f}; SKI {i} {ski:.4f}; SKD {i} {skd:.4f}; SOL {i} {sol:.2f}"
                    )
                    send_command(batch)
                    time.sleep(0.05)
            
            # Enable telemetry for all motors so live position display works
            send_command("T")
        except Exception as e:
            print(f"Warning: Failed to load and apply robot_config.json: {e}")
            
        return True
    except Exception as e:
        print(f"Failed to connect to {port_name}: {e}")
        return False

def read_from_port():
    global actual_joint_angles
    import re
    while ser and ser.is_open:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"[Arduino] {line}")
                    # Parse telemetry to update actual_joint_angles for visualization
                    for token in line.split("|"):
                        token = token.strip()
                        if token.startswith("M"):
                            match = re.search(r"M(\d+)\s+POS:([-\d.]+)", token)
                            if match:
                                mid = int(match.group(1))
                                pos = float(match.group(2))
                                # Prefer APOS (absolute encoder) if present
                                apos_match = re.search(r"APOS:([-\d.]+)", token)
                                if apos_match:
                                    pos = float(apos_match.group(1))
                                if actual_joint_angles is not None and mid < len(actual_joint_angles):
                                    actual_joint_angles[mid] = np.radians(pos)
            time.sleep(0.01)
        except Exception:
            break

def send_command(cmd):
    if ser and ser.is_open:
        try:
            ser.write((cmd + '\n').encode('utf-8'))
        except Exception as e:
            print(f"Error sending command: {e}")

def send_joints_to_arm(joint_angles_rad):
    """Send IK joint angles to motors, applying zero_offset and clamping to limits."""
    cmds = []
    for i in range(1, min(7, len(joint_angles_rad))):
        mid = str(i)
        zero_off = robot_config.get(mid, {}).get("zero_offset", 0.0)
        min_a    = robot_config.get(mid, {}).get("min_angle",  0.0)
        max_a    = robot_config.get(mid, {}).get("max_angle",  360.0)

        deg = np.degrees(joint_angles_rad[i])
        # Apply zero offset: motor_angle = IK_angle + zero_offset
        deg = (deg + zero_off) % 360.0
        # Clamp to physical limits
        deg = max(min_a, min(max_a, deg))
        cmds.append(f"P {i} {deg:.2f}")
    if cmds:
        send_command(";".join(cmds))

# --- UI and Visualization ---
# Key states
keys_pressed = {
    'w': False, 's': False,
    'a': False, 'd': False,
    'q': False, 'e': False,
    'up': False, 'down': False,
    'left': False, 'right': False,
    'shift': False  # Modifier
}

def on_press(key):
    try:
        if hasattr(key, 'char') and key.char:
            char = key.char.lower()
            if char in keys_pressed:
                keys_pressed[char] = True
        elif key == keyboard.Key.up:
            keys_pressed['up'] = True
        elif key == keyboard.Key.down:
            keys_pressed['down'] = True
        elif key == keyboard.Key.left:
            keys_pressed['left'] = True
        elif key == keyboard.Key.right:
            keys_pressed['right'] = True
        elif key == keyboard.Key.shift or key == keyboard.Key.shift_r:
            keys_pressed['shift'] = True
    except AttributeError:
        pass

def on_release(key):
    try:
        if hasattr(key, 'char') and key.char:
            char = key.char.lower()
            if char in keys_pressed:
                keys_pressed[char] = False
        elif key == keyboard.Key.up:
            keys_pressed['up'] = False
        elif key == keyboard.Key.down:
            keys_pressed['down'] = False
        elif key == keyboard.Key.left:
            keys_pressed['left'] = False
        elif key == keyboard.Key.right:
            keys_pressed['right'] = False
        elif key == keyboard.Key.shift or key == keyboard.Key.shift_r:
            keys_pressed['shift'] = False
    except AttributeError:
        pass

def finish_preview():
    global is_previewing
    is_previewing = False

def extract_xyz(positions):
    x = [p[0,3] for p in positions]
    y = [p[1,3] for p in positions]
    z = [p[2,3] for p in positions]
    return x, y, z

def init_plot_elements():
    global arm_line, ghost_line, target_point, joint_texts
    global arm_line_top, ghost_line_top, target_point_top
    global arm_line_side, ghost_line_side, target_point_side
    global ax, ax_top, ax_side

    for a in [ax, ax_top, ax_side]:
        if a is not None:
            a.cla()
            a.set_xlim(-0.6, 0.6)
            a.set_ylim(-0.6, 0.6)
            a.set_zlim(0, 0.8)
            a.set_xlabel('X')
            a.set_ylabel('Y')
            a.set_zlabel('Z')

    ax.set_title("Main 3D View")
    ax_top.set_title("Top View (XY)")
    ax_side.set_title("Side View (XZ)")

    ghost_line, = ax.plot([], [], [], '-o', color='darkorange', alpha=0.4, linewidth=4, markersize=8)
    arm_line, = ax.plot([], [], [], '-o', color='tab:blue', linewidth=5, markersize=10)
    target_point, = ax.plot([], [], [], 'x', color='red', markersize=12, markeredgewidth=2)
    
    ghost_line_top, = ax_top.plot([], [], [], '-o', color='darkorange', alpha=0.4, linewidth=4, markersize=8)
    arm_line_top, = ax_top.plot([], [], [], '-o', color='tab:blue', linewidth=5, markersize=10)
    target_point_top, = ax_top.plot([], [], [], 'x', color='red', markersize=12, markeredgewidth=2)
    
    ghost_line_side, = ax_side.plot([], [], [], '-o', color='darkorange', alpha=0.4, linewidth=4, markersize=8)
    arm_line_side, = ax_side.plot([], [], [], '-o', color='tab:blue', linewidth=5, markersize=10)
    target_point_side, = ax_side.plot([], [], [], 'x', color='red', markersize=12, markeredgewidth=2)
    
    joint_texts = [ax.text(0, 0, 0, f"J{i}", color='darkred', fontsize=11, fontweight='bold', visible=False) for i in range(1, 7)]

def render_plot():
    global actual_joint_angles, arm_line, ghost_line, target_point, joint_texts
    global arm_line_top, ghost_line_top, target_point_top
    global arm_line_side, ghost_line_side, target_point_side
    global ax, ax_top, ax_side

    if my_chain is None:
        return
        
    if actual_joint_angles is None:
        # By default, start with the current virtual position
        actual_joint_angles = list(current_joint_angles)

    if ghost_line is None:
        init_plot_elements()

    # compute ghost and arm poses
    ghost_pos = my_chain.forward_kinematics(actual_joint_angles, full_kinematics=True)
    gx, gy, gz = extract_xyz(ghost_pos)
    for gl in [ghost_line, ghost_line_top, ghost_line_side]:
        gl.set_data(gx, gy)
        gl.set_3d_properties(gz)

    angles_to_draw = current_joint_angles
    if is_previewing:
        t = (time.time() - preview_start_time) / PREVIEW_DURATION
        if t >= 1.0:
            finish_preview()
        else:
            angles_to_draw = [(1-t)*a + t*b for a, b in zip(actual_joint_angles, current_joint_angles)]
    
    arm_pos = my_chain.forward_kinematics(angles_to_draw, full_kinematics=True)
    x, y, z = extract_xyz(arm_pos)
    for al in [arm_line, arm_line_top, arm_line_side]:
        al.set_data(x, y)
        al.set_3d_properties(z)

    # labels
    for i in range(1, min(7, len(arm_pos))):
        mx, my, mz = x[i], y[i], z[i]
        joint_texts[i-1].set_position_3d((mx, my, mz + 0.04))
        joint_texts[i-1].set_visible(True)

    # marker
    for tp in [target_point, target_point_top, target_point_side]:
        tp.set_data([current_target_xyz[0]], [current_target_xyz[1]])
        tp.set_3d_properties([current_target_xyz[2]])

    ax.view_init(elev=cam_elev, azim=cam_azim)
    ax_top.view_init(elev=90, azim=-90)
    ax_side.view_init(elev=0, azim=-90)

    if txt_angles:
        active_joints = current_joint_angles[1:] if len(current_joint_angles) > 1 else []
        angle_strs = [f"J{i+1}: {np.degrees(a)%360:.1f}°" for i, a in enumerate(active_joints[:6])]
        txt_angles.set_text("Target Joints:\n" + "\n".join(angle_strs))
        
    fig.canvas.draw_idle()

def keyboard_update_loop():
    global current_target_xyz, current_target_rpy, cam_elev, cam_azim
    step = slider_step.val # meters per tick
    rot_step = 2.0 # degrees per tick for wrist
    cam_step = 2.0 # degrees per tick for camera
    
    d1, d2, d3 = 0, 0, 0 # generic deltas
    moved_cam = False

    if keys_pressed['w']: d2 += 1 # forward 
    if keys_pressed['s']: d2 -= 1 # backward
    if keys_pressed['a']: d1 -= 1 # left
    if keys_pressed['d']: d1 += 1 # right
    if keys_pressed['e']: d3 += 1 # up
    if keys_pressed['q']: d3 -= 1 # down
    
    if keys_pressed['up']:
        cam_elev += cam_step
        moved_cam = True
    if keys_pressed['down']:
        cam_elev -= cam_step
        moved_cam = True
    if keys_pressed['left']:
        cam_azim -= cam_step
        moved_cam = True
    if keys_pressed['right']:
        cam_azim += cam_step
        moved_cam = True

    cam_elev = max(-90, min(90, cam_elev))
    
    if d1 != 0 or d2 != 0 or d3 != 0:
        if keys_pressed['shift']:
            # Adjust RPY
            new_r = max(slider_r.valmin, min(slider_r.valmax, current_target_rpy[0] + d1*rot_step))
            new_p = max(slider_p.valmin, min(slider_p.valmax, current_target_rpy[1] + d2*rot_step))
            new_y = max(slider_y.valmin, min(slider_y.valmax, current_target_rpy[2] + d3*rot_step))
            current_target_rpy = [new_r, new_p, new_y]
        else:
            # Adjust XYZ
            new_x = max(slider_x.valmin, min(slider_x.valmax, current_target_xyz[0] + d1*step))
            new_y = max(slider_y.valmin, min(slider_y.valmax, current_target_xyz[1] + d2*step))
            new_z = max(slider_z.valmin, min(slider_z.valmax, current_target_xyz[2] + d3*step))
            current_target_xyz = [new_x, new_y, new_z]
        
        update_plot(val=None)
    elif is_previewing or moved_cam:
        render_plot()

def update_joint(val, idx):
    global current_target_xyz, current_joint_angles
    if is_previewing:
        return
    
    if idx < len(current_joint_angles):
        current_joint_angles[idx] = np.radians(val)
    
    # Re-run IK using the manually adjusted joint angle as the new 'seed'
    # This forces the IK solver to find a solution keeping the SAME XYZ/RPY target, 
    # but conforming around your chosen joint angle if possible.
    target_orientation = [-1, 0, 0] # Simple placeholder target constraint to try and align wrist
    new_angles = my_chain.inverse_kinematics(current_target_xyz, initial_position=current_joint_angles)
    
    # Snap to the closest valid mathematical pose
    current_joint_angles = list(new_angles)
    
    # Update Joint sliders silently to reflect the solver's snapped state
    if 'joint_sliders' in globals():
        for i, js in enumerate(joint_sliders):
            if i+1 < len(current_joint_angles):
                js.eventson = False
                js.set_val(np.degrees(current_joint_angles[i+1]))
                js.eventson = True
        
    render_plot()

def update_plot(val=None):
    global current_target_xyz, current_target_rpy, current_joint_angles, last_send_time
    global is_previewing
    
    if is_previewing:
        # Cancel preview if user manually moves target
        is_previewing = False

    if val is not None:
        # Came from slider
        x = slider_x.val
        y = slider_y.val
        z = slider_z.val
        r = slider_r.val
        p = slider_p.val
        yw = slider_y_rot.val
        
        current_target_xyz = [x, y, z]
        current_target_rpy = [r, p, yw]
    else:
        # Came from keyboard, update slider silently
        slider_x.eventson = False
        slider_y.eventson = False
        slider_z.eventson = False
        slider_r.eventson = False
        slider_p.eventson = False
        slider_y_rot.eventson = False
        
        slider_x.set_val(current_target_xyz[0])
        slider_y.set_val(current_target_xyz[1])
        slider_z.set_val(current_target_xyz[2])
        slider_r.set_val(current_target_rpy[0])
        slider_p.set_val(current_target_rpy[1])
        slider_y_rot.set_val(current_target_rpy[2])
        
        slider_x.eventson = True
        slider_y.eventson = True
        slider_z.eventson = True
        slider_r.eventson = True
        slider_p.eventson = True
        slider_y_rot.eventson = True

    # Convert RPY directly into ikpy target constraints using a simple translation trick. 
    # We configure a 'target_orientation' direction vector 
    dir_vector = [
        np.cos(np.radians(current_target_rpy[2])) * np.cos(np.radians(current_target_rpy[1])),
        np.sin(np.radians(current_target_rpy[2])) * np.cos(np.radians(current_target_rpy[1])),
        np.sin(np.radians(current_target_rpy[1]))
    ]
    
    # Compute IK
    # Orientation is partially supported by providing a target Z direction. 
    current_joint_angles = my_chain.inverse_kinematics(current_target_xyz, 
                                                     target_orientation=dir_vector,
                                                     orientation_mode="Z",
                                                     initial_position=current_joint_angles)
    
    # Update Joint sliders silently
    if 'joint_sliders' in globals():
        for i, js in enumerate(joint_sliders):
            if i+1 < len(current_joint_angles):
                js.eventson = False
                js.set_val(np.degrees(current_joint_angles[i+1]))
                js.eventson = True

    render_plot()

    # Live update mechanism (throttle updates to 10Hz)
    current_time = time.time()
    if live_update and (current_time - last_send_time > 0.1):
        execute_button_callback(None)

def find_alternative_ik(event):
    global current_joint_angles
    print("Searching for an alternative IK solution...")
    for _ in range(50): # try up to 50 times
        seed = [0.0] * len(current_joint_angles)
        for i in range(1, len(seed)):
            seed[i] = np.random.uniform(-np.pi, np.pi)
        
        test_angles = my_chain.inverse_kinematics(current_target_xyz, initial_position=seed)
        fk = my_chain.forward_kinematics(test_angles)
        
        # Check if the generated solution actually hits the target
        dist = np.linalg.norm(fk[:3, 3] - current_target_xyz)
        if dist < 0.05: # within 5cm accuracy
            # Check if it's sufficiently different from the current pose 
            diff = np.linalg.norm(np.array(test_angles[1:]) - np.array(current_joint_angles[1:]))
            if diff > 0.5:
                current_joint_angles = list(test_angles)
                print(f"Found alternative solution! (Diff: {diff:.2f} rad, Error: {dist:.4f}m)")
                
                # Update Joint sliders silently
                if 'joint_sliders' in globals():
                    for i, js in enumerate(joint_sliders):
                        if i+1 < len(current_joint_angles):
                            js.eventson = False
                            js.set_val(np.degrees(current_joint_angles[i+1]))
                            js.eventson = True
                            
                render_plot()
                return
                
    print("Could not find a significantly different Alternative IK solution for this coordinate.")

def preview_button_callback(event):
    global is_previewing, preview_start_time
    print("Previewing Plan (Ghost -> Target)...")
    is_previewing = True
    preview_start_time = time.time()
    render_plot()

def execute_button_callback(event):
    global actual_joint_angles, last_send_time
    if event is not None:
        print("Executing: Sending joint angles to Arm!")
    actual_joint_angles = list(current_joint_angles)
    send_joints_to_arm(current_joint_angles)
    last_send_time = time.time()
    render_plot()

def toggle_live_update(label):
    global live_update
    live_update = not live_update
    if live_update:
        print("Live update ENABLED.")
    else:
        print("Live update DISABLED.")

def emergency_stop_callback(event):
    print("EMERGENCY STOP TRIGGERED!")
    send_command("S")

def main():
    global my_chain, fig, ax, ax_top, ax_side
    global slider_x, slider_y, slider_z, slider_step, txt_angles
    global slider_r, slider_p, slider_y_rot
    global joint_sliders, slider_prev
    
    print("Loading URDF file:", URDF_FILE)
    try:
        # ikpy doesn't support 'continuous' joints, so we patch it in memory
        with open(URDF_FILE, 'r') as f:
            urdf_data = f.read()
        urdf_data = urdf_data.replace('type="continuous"', 'type="revolute"')
        
        with tempfile.NamedTemporaryFile(delete=False, mode='w', suffix='.urdf') as temp_urdf:
            temp_urdf.write(urdf_data)
            temp_path = temp_urdf.name

        my_chain = Chain.from_urdf_file(temp_path)
        
        # Patch joint limits into ikpy from robot_config.json
        # Links 1..N correspond to joints 1..N (link 0 is the base).
        # Bounds are in radians, relative to the URDF/IK zero which is offset
        # from the physical motor zero by zero_offset degrees.
        active_joint_idx = 0
        for link_idx, link in enumerate(my_chain.links):
            if link_idx == 0:
                continue  # skip base
            active_joint_idx += 1
            mid = str(active_joint_idx)
            if mid in robot_config:
                mn   = robot_config[mid].get("min_angle",  0.0)
                mx   = robot_config[mid].get("max_angle",  360.0)
                zoff = robot_config[mid].get("zero_offset", 0.0)
                # Convert physical degree limits to IK-frame radians
                lower = math.radians(mn - zoff)
                upper = math.radians(mx - zoff)
                if hasattr(link, 'bounds'):
                    link.bounds = (lower, upper)
            if active_joint_idx >= 6:
                break

        # Determine number of joints in the chain, ignoring the Base link (which is usually index 0)
        # We need this to safely map the real arm joints dynamically:
        global current_joint_angles
        num_joints = len(my_chain.links)
        current_joint_angles = [0.0] * num_joints
        
        os.remove(temp_path)
        
    except Exception as e:
        print(f"Failed to load URDF: {e}")
        traceback.print_exc()
        print(f"Make sure {URDF_FILE} is placed in the right directory.")
        sys.exit(1)

    connect_serial()

    # --- Setup Matplotlib Figure ---
    fig = plt.figure(figsize=(14, 8))
    fig.canvas.manager.set_window_title("6DoF Robot Arm IK Visualization")
    
    # 3D Axes for visualization
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    ax_top = fig.add_subplot(2, 2, 2, projection='3d')
    ax_side = fig.add_subplot(2, 2, 4, projection='3d')
    plt.subplots_adjust(left=0.01, bottom=0.05, right=0.6, top=0.95, wspace=0.1, hspace=0.1) 
    
    # Slider configurations
    slider_width = 0.22
    slider_left = 0.72
    dh = 0.033 # vertical step
    current_y = 0.90
    
    # Target Joints text box
    txt_angles = fig.text(0.65, 0.90, "", fontsize=10, va='top', ha='left', 
                          bbox=dict(facecolor='aliceblue', alpha=0.8, edgecolor='black', boxstyle='round,pad=0.5'))

    axcolor = 'lightgoldenrodyellow'
    current_y = 0.86
    ax_x = plt.axes([slider_left, current_y, slider_width, 0.02], facecolor=axcolor); current_y -= dh
    ax_y = plt.axes([slider_left, current_y, slider_width, 0.02], facecolor=axcolor); current_y -= dh
    ax_z = plt.axes([slider_left, current_y, slider_width, 0.02], facecolor=axcolor); current_y -= dh
    
    current_y -= 0.01
    ax_r = plt.axes([slider_left, current_y, slider_width, 0.02], facecolor='lightgreen'); current_y -= dh
    ax_p = plt.axes([slider_left, current_y, slider_width, 0.02], facecolor='lightgreen'); current_y -= dh
    ax_yw= plt.axes([slider_left, current_y, slider_width, 0.02], facecolor='lightgreen'); current_y -= dh
    
    current_y -= 0.01
    ax_step = plt.axes([slider_left, current_y, slider_width, 0.02], facecolor=axcolor); current_y -= dh
    
    current_y -= 0.02
    ax_j1 = plt.axes([slider_left, current_y, slider_width, 0.02], facecolor='lightblue'); current_y -= dh
    ax_j2 = plt.axes([slider_left, current_y, slider_width, 0.02], facecolor='lightblue'); current_y -= dh
    ax_j3 = plt.axes([slider_left, current_y, slider_width, 0.02], facecolor='lightblue'); current_y -= dh
    ax_j4 = plt.axes([slider_left, current_y, slider_width, 0.02], facecolor='lightblue'); current_y -= dh
    ax_j5 = plt.axes([slider_left, current_y, slider_width, 0.02], facecolor='lightblue'); current_y -= dh
    ax_j6 = plt.axes([slider_left, current_y, slider_width, 0.02], facecolor='lightblue'); current_y -= dh
    
    current_y -= 0.01
    ax_prev_t = plt.axes([slider_left, current_y, slider_width, 0.02], facecolor=axcolor); current_y -= dh

    # Sliders mapping
    slider_x = Slider(ax_x, 'X Target', -1.0, 1.0, valinit=current_target_xyz[0])
    slider_y = Slider(ax_y, 'Y Target', -1.0, 1.0, valinit=current_target_xyz[1])
    slider_z = Slider(ax_z, 'Z Target', 0.0, 1.5, valinit=current_target_xyz[2])
    
    slider_r = Slider(ax_r, 'Roll', -180.0, 180.0, valinit=current_target_rpy[0])
    slider_p = Slider(ax_p, 'Pitch', -180.0, 180.0, valinit=current_target_rpy[1])
    slider_y_rot = Slider(ax_yw, 'Yaw', -180.0, 180.0, valinit=current_target_rpy[2])
    
    slider_step = Slider(ax_step, 'Key Step', 0.001, 0.05, valinit=0.01)

    slider_x.on_changed(update_plot)
    slider_y.on_changed(update_plot)
    slider_z.on_changed(update_plot)
    slider_r.on_changed(update_plot)
    slider_p.on_changed(update_plot)
    slider_y_rot.on_changed(update_plot)
    
    joint_sliders = []
    num_active = min(6, len(current_joint_angles)-1)
    if num_active >= 1: joint_sliders.append(Slider(ax_j1, 'J1 (deg)', -360, 360, valinit=np.degrees(current_joint_angles[1])))
    if num_active >= 2: joint_sliders.append(Slider(ax_j2, 'J2 (deg)', -360, 360, valinit=np.degrees(current_joint_angles[2])))
    if num_active >= 3: joint_sliders.append(Slider(ax_j3, 'J3 (deg)', -360, 360, valinit=np.degrees(current_joint_angles[3])))
    if num_active >= 4: joint_sliders.append(Slider(ax_j4, 'J4 (deg)', -360, 360, valinit=np.degrees(current_joint_angles[4])))
    if num_active >= 5: joint_sliders.append(Slider(ax_j5, 'J5 (deg)', -360, 360, valinit=np.degrees(current_joint_angles[5])))
    if num_active >= 6: joint_sliders.append(Slider(ax_j6, 'J6 (deg)', -360, 360, valinit=np.degrees(current_joint_angles[6])))
    
    for i, js in enumerate(joint_sliders):
        def make_update(idx):
            return lambda val: update_joint(val, idx)
        js.on_changed(make_update(i+1))

    slider_prev = Slider(ax_prev_t, 'Prev Time(s)', 0.1, 5.0, valinit=PREVIEW_DURATION)
    def update_prev_time(val):
        global PREVIEW_DURATION
        PREVIEW_DURATION = val
    slider_prev.on_changed(update_prev_time)

    # Buttons Row
    current_y -= 0.05
    ax_alt = plt.axes([0.65, current_y, 0.13, 0.04])
    btn_alt = Button(ax_alt, 'Alternative IK', color='thistle', hovercolor='0.9')
    btn_alt.on_clicked(find_alternative_ik)

    ax_prev_btn = plt.axes([0.80, current_y, 0.13, 0.04])
    btn_prev = Button(ax_prev_btn, 'Preview (Plan)', color='lightblue', hovercolor='0.9')
    btn_prev.on_clicked(preview_button_callback)

    current_y -= 0.06
    ax_exec = plt.axes([0.65, current_y, 0.13, 0.04])
    btn_exec = Button(ax_exec, 'Execute', color='lightgreen', hovercolor='0.9')
    btn_exec.on_clicked(execute_button_callback)

    ax_live = plt.axes([0.80, current_y, 0.13, 0.04])
    chk_live = CheckButtons(ax_live, ['Live Execute'], [live_update])
    chk_live.on_clicked(toggle_live_update)

    current_y -= 0.06
    ax_stop = plt.axes([0.65, current_y, 0.28, 0.05])
    btn_stop = Button(ax_stop, 'EMERGENCY STOP', color='salmon', hovercolor='red')
    btn_stop.label.set_fontweight('bold')
    btn_stop.on_clicked(emergency_stop_callback)

    print("\n--- Controls ---")
    print("Use sliders or keys to modify the GOAL target (X, Y, Z).")
    print("Click 'Preview (Plan)' to simulate the arm moving from current state to the goal state.")
    print("Click 'Execute' to send the goal angles to the hardware and update the current state.")
    print("Click 'EMERGENCY STOP' to immediately stop all motors.")
    print("Tick 'Live Execute' to automatically execute immediately when moving.")
    print("Close the window to exit.")
    
    # Start pynput keyboard listener
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    print("\n--- Additional Controls ---")
    print("Use WASD to move XYZ (Forward/Back/Left/Right). Use Q/E to move Z (Down/Up).")
    print("Hold SHIFT + WASD/QE to adjust the Roll/Pitch/Yaw instead.")
    print("Use ARROW KEYS to rotate the Main Camera View.")

    # Start matplotlib timer for smooth keyboard polling
    timer = fig.canvas.new_timer(interval=10)
    timer.add_callback(keyboard_update_loop)
    timer.start()

    # Initial Plot
    update_plot()
    
    # Show Window (blocks until closed)
    plt.show()

    listener.stop()

    if ser and ser.is_open:
        # Stop everything before closing
        send_command("S")
        time.sleep(0.1)
        ser.close()
    print("Exited.")

if __name__ == "__main__":
    main()
