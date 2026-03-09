import os
import sys
import time
import tempfile
import traceback
import json

import numpy as np
from PySide6.QtCore import QTimer, Qt
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QLabel,
    QMainWindow,
    QPushButton,
    QSplitter,
    QHeaderView,
    QScrollArea,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import ik_arm_control as backend


KEYMAP_FILE = os.path.join(os.path.dirname(__file__), "ik_dashboard_keys.json")
KEY_OPTIONS = [
    "None",
    "W", "A", "S", "D", "Q", "E",
    "I", "J", "K", "L", "U", "O",
    "T", "F", "G", "H", "R", "Y",
    "P", "X", "C", "V", "B", "N", "M",
    "Up", "Down", "Left", "Right", "Space",
]
DEFAULT_KEYMAP = {
    "move_x_neg": "A",
    "move_x_pos": "D",
    "move_y_pos": "W",
    "move_y_neg": "S",
    "move_z_neg": "Q",
    "move_z_pos": "E",
    "pitch_pos": "I",
    "pitch_neg": "K",
    "roll_neg": "J",
    "roll_pos": "L",
    "yaw_neg": "U",
    "yaw_pos": "O",
    "cam_left": "Left",
    "cam_right": "Right",
    "cam_up": "Up",
    "cam_down": "Down",
    "execute": "Space",
    "estop": "X",
    "preview": "P",
    "sync_target": "C",
}
ACTION_LABELS = {
    "move_x_neg": "Move X-",
    "move_x_pos": "Move X+",
    "move_y_pos": "Move Y+",
    "move_y_neg": "Move Y-",
    "move_z_neg": "Move Z-",
    "move_z_pos": "Move Z+",
    "pitch_pos": "Pitch +",
    "pitch_neg": "Pitch -",
    "roll_neg": "Roll -",
    "roll_pos": "Roll +",
    "yaw_neg": "Yaw -",
    "yaw_pos": "Yaw +",
    "cam_left": "Cam Left",
    "cam_right": "Cam Right",
    "cam_up": "Cam Up",
    "cam_down": "Cam Down",
    "execute": "Execute",
    "estop": "E-Stop",
    "preview": "Preview",
    "sync_target": "Target Sync",
}


def _fmt_deg(value):
    return "---.-°" if value is None else f"{float(value):7.1f}°"


def _fmt_rpm(value):
    return "---.-" if value is None else f"{float(value):7.1f}"


def get_wrist_dashboard_snapshot():
    if not backend.wrist_diff_enabled():
        return None
    cfg = backend.get_wrist_diff_config()
    pitch_joint_id, roll_joint_id = backend.wrist_diff_joint_ids()
    motor_a_id, motor_b_id = backend.wrist_diff_motor_ids()
    pitch_actual = backend.get_wrist_output_logical_deg("pitch")
    roll_actual = backend.get_wrist_output_logical_deg("roll")
    pitch_target = backend.wrist_diff_target_logical_deg.get("pitch")
    roll_target = backend.wrist_diff_target_logical_deg.get("roll")
    last_cmd_a, last_cmd_b = backend.wrist_diff_last_motor_cmd
    return {
        "pitch_joint_id": pitch_joint_id,
        "roll_joint_id": roll_joint_id,
        "pitch_actual": pitch_actual,
        "roll_actual": roll_actual,
        "pitch_target": pitch_target,
        "roll_target": roll_target,
        "motor_a_id": motor_a_id,
        "motor_b_id": motor_b_id,
        "motor_a_cmd": last_cmd_a,
        "motor_b_cmd": last_cmd_b,
        "pitch_encoder_motor_id": int(cfg.get("pitch_encoder_motor_id", 5)),
        "roll_encoder_motor_id": int(cfg.get("roll_encoder_motor_id", 6)),
    }


class ArmCanvas(FigureCanvas):
    def __init__(self, parent=None):
        self.figure = Figure(figsize=(9.4, 8.8))
        super().__init__(self.figure)
        self.setParent(parent)
        gs = self.figure.add_gridspec(2, 2, height_ratios=[3.3, 1.4], hspace=0.08, wspace=0.05)
        self.ax = self.figure.add_subplot(gs[0, 0], projection="3d")
        self.ax_side = self.figure.add_subplot(gs[0, 1], projection="3d")
        self.ax_top = None
        self.ax_status = self.figure.add_subplot(gs[1, :])
        self.figure.subplots_adjust(left=0.03, right=0.99, top=0.98, bottom=0.04)

    def draw_arm(self):
        self.ax.cla()
        self.ax_side.cla()
        self.ax_status.cla()
        for axis in (self.ax, self.ax_side):
            axis.set_xlim(-0.9, 0.9)
            axis.set_ylim(-0.9, 0.9)
            axis.set_zlim(-0.05, 1.0)
            axis.set_xlabel("X")
            axis.set_ylabel("Y")
            axis.set_zlabel("Z")
        self.ax.set_title("3D View")
        self.ax_side.set_title("Side View")
        self.ax.view_init(elev=backend.cam_elev, azim=backend.cam_azim)
        self.ax_side.view_init(elev=0, azim=-90)

        if backend.my_chain is None:
            self.draw_idle()
            return

        if backend.actual_joint_angles is None:
            backend.actual_joint_angles = list(backend.current_joint_angles)

        if backend.sync_model_to_telemetry:
            tele_angles = backend.get_current_real_joint_angles()
            if tele_angles is not None:
                backend.actual_joint_angles = list(tele_angles)

        ghost_pos = backend.my_chain.forward_kinematics(backend.actual_joint_angles, full_kinematics=True)
        gx, gy, gz = backend.extract_xyz(ghost_pos)
        arm_pos = backend.my_chain.forward_kinematics(backend.current_joint_angles, full_kinematics=True)
        x, y, z = backend.extract_xyz(arm_pos)
        target_x = -backend.current_target_xyz[0] if backend.DISPLAY_MIRROR_X else backend.current_target_xyz[0]

        for axis in (self.ax, self.ax_side):
            axis.plot(gx, gy, gz, "-o", color="darkorange", alpha=0.35, linewidth=4, markersize=8)
            axis.plot(x, y, z, "-o", color="tab:blue", linewidth=5, markersize=9)
            axis.plot([target_x], [backend.current_target_xyz[1]], [backend.current_target_xyz[2]], "x", color="red", markersize=12)

        for joint_idx in range(1, min(7, len(x))):
            self.ax.text(x[joint_idx], y[joint_idx], z[joint_idx] + 0.03, f"J{joint_idx}", color="darkred", fontsize=9)

        self.ax_status.axis("off")
        lines = []
        for joint_idx in range(1, 7):
            target_deg = float(np.degrees(backend.current_joint_angles[joint_idx])) if joint_idx < len(backend.current_joint_angles) else 0.0
            current_deg = backend.get_current_ik_deg_from_apos(joint_idx)
            if current_deg is None:
                lines.append(f"J{joint_idx}: tgt {target_deg:7.1f}°   cur   ---.-°")
            else:
                lines.append(f"J{joint_idx}: tgt {target_deg:7.1f}°   cur {current_deg:7.1f}°")
        wrist = get_wrist_dashboard_snapshot()
        if wrist is not None:
            lines.append("")
            lines.append(
                f"Wrist Pitch(J{wrist['pitch_joint_id']}): tgt {_fmt_deg(wrist['pitch_target']).strip()}   cur {_fmt_deg(wrist['pitch_actual']).strip()}"
            )
            lines.append(
                f"Wrist Roll (J{wrist['roll_joint_id']}): tgt {_fmt_deg(wrist['roll_target']).strip()}   cur {_fmt_deg(wrist['roll_actual']).strip()}"
            )
            lines.append(
                f"Mix Cmd M{wrist['motor_a_id']}/{wrist['motor_b_id']}: {_fmt_rpm(wrist['motor_a_cmd']).strip()} / {_fmt_rpm(wrist['motor_b_cmd']).strip()} RPM"
            )
        self.ax_status.text(
            0.02,
            0.98,
            "Arm Status\n\n" + "\n".join(lines),
            va="top",
            ha="left",
            fontsize=10,
            family="monospace",
            bbox=dict(facecolor="#f4f4f4", edgecolor="black", boxstyle="round,pad=0.5"),
        )
        self.draw_idle()


class DashboardWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IK Dashboard (PySide6)")
        self.resize(1280, 900)
        self._log_lines = []
        self._building_controls = False
        self._last_status_snapshot = ""
        self._pressed_keys = set()
        self._key_action_fired = set()
        self.keymap = self._load_keymap()
        self.keymap_inputs = {}
        self.key_indicator_labels = {}
        self._build_ui()
        self._init_backend()
        self._start_timers()
        self.refresh_all()
        self.setFocusPolicy(Qt.StrongFocus)

    def _init_backend(self):
        backend.load_robot_config()
        try:
            with open(backend.URDF_FILE, "r", encoding="utf-8") as f:
                urdf_data = f.read().replace('type="continuous"', 'type="revolute"')
            with tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".urdf", encoding="utf-8") as temp_urdf:
                temp_urdf.write(urdf_data)
                temp_path = temp_urdf.name
            backend.my_chain = backend.Chain.from_urdf_file(temp_path)
            os.remove(temp_path)
            joint_count = len(backend.my_chain.links)
            backend.current_joint_angles = backend.clamp_seed_angles_to_bounds([0.0] * joint_count)
            backend.refresh_locked_joint_rads(backend.current_joint_angles)
            backend.apply_chain_bounds_from_config()
            if not self._connect_serial_passive():
                self._append_log("Serial connection not established at startup.")
            if not backend.sync_solver_seed_from_telemetry(update_target=True):
                backend.refresh_locked_joint_rads(backend.current_joint_angles)
                backend.apply_chain_bounds_from_config()
                backend.sync_target_from_joint_angles(backend.current_joint_angles)
            else:
                self._append_log("Synchronized dashboard pose from telemetry.")
            backend.planar_target_y = float(backend.current_target_xyz[1])
        except Exception as exc:
            traceback.print_exc()
            raise RuntimeError(f"Failed to initialize IK backend: {exc}")

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(8, 8, 8, 8)
        root_layout.setSpacing(6)

        splitter = QSplitter(Qt.Horizontal)
        root_layout.addWidget(splitter, 1)

        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        splitter.addWidget(left_scroll)

        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(6)
        left_panel.setMaximumWidth(330)
        left_scroll.setWidget(left_panel)

        self.canvas = ArmCanvas(self)
        splitter.addWidget(self.canvas)
        splitter.setSizes([310, 970])
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)

        title = QLabel("Robot Arm IK Dashboard")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 18px; font-weight: bold; padding: 4px;")
        root_layout.insertWidget(0, title)

        controls_group = QGroupBox("Controls")
        controls_layout = QGridLayout(controls_group)
        controls_layout.setHorizontalSpacing(4)
        controls_layout.setVerticalSpacing(4)
        left_layout.addWidget(controls_group)

        self.btn_connect = QPushButton()
        self.btn_connect.clicked.connect(self.toggle_connection)
        controls_layout.addWidget(self.btn_connect, 0, 0, 1, 2)

        self.btn_execute = QPushButton("Execute")
        self.btn_execute.clicked.connect(lambda: self.execute_now())
        controls_layout.addWidget(self.btn_execute, 1, 0)

        self.btn_estop = QPushButton("E-Stop")
        self.btn_estop.clicked.connect(lambda: self.estop())
        controls_layout.addWidget(self.btn_estop, 1, 1)

        self.btn_alt = QPushButton("Alternative IK")
        self.btn_alt.clicked.connect(lambda: self.find_alternative())
        controls_layout.addWidget(self.btn_alt, 2, 0)

        self.btn_preview = QPushButton("Preview")
        self.btn_preview.clicked.connect(lambda: self.preview_plan())
        controls_layout.addWidget(self.btn_preview, 2, 1)

        self.btn_zero = QPushButton("Set Current Pose as Zero")
        self.btn_zero.clicked.connect(lambda: self.capture_zero())
        controls_layout.addWidget(self.btn_zero, 3, 0, 1, 2)

        self.btn_move_zero = QPushButton("Move Configured Joints to Zero")
        self.btn_move_zero.clicked.connect(lambda: self.move_to_zero())
        controls_layout.addWidget(self.btn_move_zero, 4, 0, 1, 2)

        self.chk_live = QCheckBox("Live Execute")
        self.chk_live.setChecked(backend.live_update)
        self.chk_live.toggled.connect(self.toggle_live_execute)
        controls_layout.addWidget(self.chk_live, 5, 0)

        self.chk_sync = QCheckBox("Sync Ghost to Telemetry")
        self.chk_sync.setChecked(backend.sync_model_to_telemetry)
        self.chk_sync.toggled.connect(self.toggle_sync)
        controls_layout.addWidget(self.chk_sync, 5, 1)

        self.chk_mirror = QCheckBox("Mirror Display X")
        self.chk_mirror.setChecked(backend.DISPLAY_MIRROR_X)
        self.chk_mirror.toggled.connect(self.toggle_mirror)
        controls_layout.addWidget(self.chk_mirror, 6, 0)

        self.btn_sync_target = QPushButton("Target From Telemetry")
        self.btn_sync_target.clicked.connect(lambda: self.sync_target_from_telemetry())
        controls_layout.addWidget(self.btn_sync_target, 6, 1)

        camera_group = QGroupBox("Camera")
        camera_layout = QGridLayout(camera_group)
        camera_layout.setHorizontalSpacing(4)
        camera_layout.setVerticalSpacing(4)
        left_layout.addWidget(camera_group)

        self.btn_cam_left = QPushButton("Left")
        self.btn_cam_left.clicked.connect(lambda: self.adjust_camera(0.0, -10.0))
        camera_layout.addWidget(self.btn_cam_left, 0, 0)

        self.btn_cam_right = QPushButton("Right")
        self.btn_cam_right.clicked.connect(lambda: self.adjust_camera(0.0, 10.0))
        camera_layout.addWidget(self.btn_cam_right, 0, 1)

        self.btn_cam_up = QPushButton("Up")
        self.btn_cam_up.clicked.connect(lambda: self.adjust_camera(10.0, 0.0))
        camera_layout.addWidget(self.btn_cam_up, 1, 0)

        self.btn_cam_down = QPushButton("Down")
        self.btn_cam_down.clicked.connect(lambda: self.adjust_camera(-10.0, 0.0))
        camera_layout.addWidget(self.btn_cam_down, 1, 1)

        self.btn_cam_reset = QPushButton("Reset")
        self.btn_cam_reset.clicked.connect(lambda: self.reset_camera())
        camera_layout.addWidget(self.btn_cam_reset, 2, 0, 1, 2)

        target_group = QGroupBox("Target")
        target_layout = QGridLayout(target_group)
        target_layout.setHorizontalSpacing(4)
        target_layout.setVerticalSpacing(4)
        left_layout.addWidget(target_group)

        self.target_inputs = {}
        for row, key in enumerate(("x", "y", "z", "roll", "pitch", "yaw")):
            label = QLabel(key.upper())
            spin = QDoubleSpinBox()
            spin.setDecimals(3)
            spin.setSingleStep(0.01)
            if key == "z":
                spin.setRange(0.0, 1.5)
            elif key in ("roll", "pitch", "yaw"):
                spin.setDecimals(1)
                spin.setSingleStep(1.0)
                spin.setRange(-180.0, 180.0)
            else:
                spin.setRange(-1.0, 1.0)
            spin.valueChanged.connect(self.on_target_changed)
            target_layout.addWidget(label, row, 0)
            target_layout.addWidget(spin, row, 1)
            self.target_inputs[key] = spin

        self.step_input = QDoubleSpinBox()
        self.step_input.setDecimals(3)
        self.step_input.setRange(0.001, 0.05)
        self.step_input.setSingleStep(0.001)
        self.step_input.setValue(0.01)
        target_layout.addWidget(QLabel("Key Step"), 6, 0)
        target_layout.addWidget(self.step_input, 6, 1)

        joint_group = QGroupBox("Joint Targets (deg)")
        joint_layout = QGridLayout(joint_group)
        joint_layout.setHorizontalSpacing(4)
        joint_layout.setVerticalSpacing(4)
        left_layout.addWidget(joint_group)

        self.joint_inputs = {}
        for row, joint_idx in enumerate(range(1, 7)):
            label = QLabel(f"J{joint_idx}")
            spin = QDoubleSpinBox()
            spin.setDecimals(2)
            spin.setRange(-360.0, 360.0)
            spin.setSingleStep(1.0)
            spin.valueChanged.connect(lambda value, idx=joint_idx: self.on_joint_changed(idx, value))
            joint_layout.addWidget(label, row, 0)
            joint_layout.addWidget(spin, row, 1)
            self.joint_inputs[joint_idx] = spin

        status_group = QGroupBox("Status")
        status_layout = QVBoxLayout(status_group)
        status_layout.setContentsMargins(6, 6, 6, 6)
        status_layout.setSpacing(4)
        left_layout.addWidget(status_group)

        self.status_label = QLabel()
        self.status_label.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self.status_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        status_layout.addWidget(self.status_label)

        self.joint_table = QTableWidget(6, 4)
        self.joint_table.setHorizontalHeaderLabels(["Joint", "Target", "Current", "Error"])
        self.joint_table.verticalHeader().setVisible(False)
        self.joint_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        status_layout.addWidget(self.joint_table)

        tabs = QTabWidget()
        left_layout.addWidget(tabs, 1)

        log_tab = QWidget()
        log_layout = QVBoxLayout(log_tab)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        log_layout.addWidget(self.log_view)
        tabs.addTab(log_tab, "Event Log")

        tele_tab = QWidget()
        tele_layout = QVBoxLayout(tele_tab)
        self.telemetry_view = QTextEdit()
        self.telemetry_view.setReadOnly(True)
        tele_layout.addWidget(self.telemetry_view)
        tabs.addTab(tele_tab, "Telemetry")

        help_tab = QWidget()
        help_layout = QVBoxLayout(help_tab)
        self.help_view = QTextEdit()
        self.help_view.setReadOnly(True)
        self.help_view.setPlainText(
            "Controls\n\n"
            "- Connect: open/close serial link\n"
            "- Execute: send current joint target\n"
            "- Preview: copy target to actual preview pose\n"
            "- E-Stop: send stop command\n"
            "- Set Current Pose as Zero: save abs zero offsets\n"
            "- Move Configured Joints to Zero: send configured zero command\n"
            "- Target From Telemetry: align IK target with current real pose\n"
            "- Mirror Display X: flip the rendered view only\n"
            "- Key Bindings: remap controls and save them to ik_dashboard_keys.json\n"
            "- Startup Config: applies stored robot_config settings and telemetry bring-up on connect\n"
        )
        help_layout.addWidget(self.help_view)
        tabs.addTab(help_tab, "Help")

        keys_group = QGroupBox("Keyboard")
        keys_layout = QVBoxLayout(keys_group)
        keys_layout.setContentsMargins(6, 6, 6, 6)
        keys_layout.setSpacing(4)
        left_layout.addWidget(keys_group)

        self.keys_help = QLabel(
            "Keyboard mappings are editable below and saved to disk.\n"
            "Click the dashboard first so it has focus."
        )
        self.keys_help.setWordWrap(True)
        keys_layout.addWidget(self.keys_help)

        self.keys_status = QLabel("Pressed: <none>")
        self.keys_status.setWordWrap(True)
        self.keys_status.setStyleSheet("font-family: monospace; background: #f4f4f4; padding: 4px; border-radius: 4px;")
        keys_layout.addWidget(self.keys_status)

        keymap_group = QGroupBox("Key Bindings")
        keymap_layout = QGridLayout(keymap_group)
        keymap_layout.setHorizontalSpacing(4)
        keymap_layout.setVerticalSpacing(4)
        left_layout.addWidget(keymap_group)

        action_order = [
            "move_x_neg", "move_x_pos", "move_y_pos", "move_y_neg", "move_z_neg", "move_z_pos",
            "roll_neg", "roll_pos", "pitch_pos", "pitch_neg", "yaw_neg", "yaw_pos",
            "cam_left", "cam_right", "cam_up", "cam_down",
            "execute", "preview", "sync_target", "estop",
        ]
        for row, action in enumerate(action_order):
            label = QLabel(ACTION_LABELS[action])
            combo = QComboBox()
            combo.addItems(KEY_OPTIONS)
            combo.setCurrentText(self.keymap.get(action, "None"))
            combo.currentTextChanged.connect(lambda value, act=action: self.on_keymap_changed(act, value))
            indicator = QLabel()
            indicator.setAlignment(Qt.AlignCenter)
            indicator.setMinimumHeight(24)
            indicator.setStyleSheet("background-color: #e9ecef; color: #222; border: 1px solid #c5c9ce; border-radius: 4px; font-weight: bold;")
            keymap_layout.addWidget(label, row, 0)
            keymap_layout.addWidget(combo, row, 1)
            keymap_layout.addWidget(indicator, row, 2)
            self.keymap_inputs[action] = combo
            self.key_indicator_labels[action] = indicator

        self.btn_save_keys = QPushButton("Save Key Bindings")
        self.btn_save_keys.clicked.connect(self.save_keymap)
        keymap_layout.addWidget(self.btn_save_keys, len(action_order), 0, 1, 2)

        self.btn_reset_keys = QPushButton("Reset Defaults")
        self.btn_reset_keys.clicked.connect(self.reset_keymap_defaults)
        keymap_layout.addWidget(self.btn_reset_keys, len(action_order), 2)

        left_layout.addStretch(1)

    def _load_keymap(self):
        if not os.path.exists(KEYMAP_FILE):
            return dict(DEFAULT_KEYMAP)
        try:
            with open(KEYMAP_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
            if not isinstance(data, dict):
                return dict(DEFAULT_KEYMAP)
            out = dict(DEFAULT_KEYMAP)
            for action in DEFAULT_KEYMAP:
                value = str(data.get(action, out[action]))
                out[action] = value if value in KEY_OPTIONS else out[action]
            return out
        except Exception:
            return dict(DEFAULT_KEYMAP)

    def save_keymap(self):
        try:
            with open(KEYMAP_FILE, "w", encoding="utf-8") as f:
                json.dump(self.keymap, f, indent=2)
            self._append_log(f"Saved key bindings to {os.path.basename(KEYMAP_FILE)}.")
        except Exception as exc:
            self._append_log(f"Failed to save key bindings: {exc}")

    def reset_keymap_defaults(self):
        self.keymap = dict(DEFAULT_KEYMAP)
        for action, combo in self.keymap_inputs.items():
            combo.blockSignals(True)
            combo.setCurrentText(self.keymap.get(action, "None"))
            combo.blockSignals(False)
        self.save_keymap()
        self.refresh_all(skip_inputs=True)

    def on_keymap_changed(self, action, value):
        self.keymap[action] = value if value in KEY_OPTIONS else "None"
        self.refresh_all(skip_inputs=True)

    def _connect_serial_passive(self):
        old_auto_zero = getattr(backend, "AUTO_MOVE_TO_ZERO_ON_CONNECT", True)
        try:
            backend.AUTO_MOVE_TO_ZERO_ON_CONNECT = False
            ok = backend.connect_serial()
        finally:
            backend.AUTO_MOVE_TO_ZERO_ON_CONNECT = old_auto_zero
        if not ok:
            return False
        if backend.wait_for_telemetry(timeout_sec=1.5):
            self._append_log("Connected serial device; telemetry is live.")
        else:
            self._append_log("Connected serial device; waiting for telemetry.")
        return True

    def _start_timers(self):
        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh_all)
        self.refresh_timer.start(100)

        self.live_timer = QTimer(self)
        self.live_timer.timeout.connect(self._live_execute_tick)
        self.live_timer.start(100)

        self.keyboard_timer = QTimer(self)
        self.keyboard_timer.timeout.connect(self._keyboard_tick)
        self.keyboard_timer.start(40)

    def _append_log(self, message):
        stamp = time.strftime("%H:%M:%S")
        line = f"[{stamp}] {message}"
        self._log_lines.append(line)
        self._log_lines = self._log_lines[-200:]
        if hasattr(self, "log_view"):
            self.log_view.setPlainText("\n".join(self._log_lines))
            self.log_view.verticalScrollBar().setValue(self.log_view.verticalScrollBar().maximum())

    def _send_current_joint_target(self, smooth=False, repeat_count=1):
        backend.current_joint_angles = backend.clamp_seed_angles_to_bounds(
            backend.enforce_locked_joints(backend.current_joint_angles)
        )
        backend.actual_joint_angles = list(backend.current_joint_angles)
        backend.refresh_locked_joint_rads(backend.actual_joint_angles)
        backend.send_joints_to_arm(
            backend.current_joint_angles,
            smooth=smooth,
            max_ramp_steps=20 if smooth else 1,
            force_immediate=True,
            repeat_count=repeat_count,
        )
        backend.last_send_time = time.time()

    def toggle_connection(self):
        if backend.ser and backend.ser.is_open:
            backend.disconnect_serial()
            self._append_log("Disconnected serial device.")
        else:
            ok = self._connect_serial_passive()
            self._append_log("Serial connection failed." if not ok else "Passive startup complete: no motion commands sent.")
            if ok:
                if backend.sync_solver_seed_from_telemetry(update_target=True):
                    self._append_log("Synchronized dashboard pose from telemetry.")
        self.refresh_all()

    def execute_now(self):
        self._send_current_joint_target(smooth=False, repeat_count=3)
        self._append_log("Execute requested.")
        self.refresh_all()

    def preview_plan(self):
        backend.actual_joint_angles = list(backend.current_joint_angles)
        self._append_log("Preview pose updated to current target.")
        self.refresh_all()

    def estop(self):
        backend.emergency_stop_callback(None)
        self._append_log("Emergency stop sent.")
        self.refresh_all()

    def find_alternative(self):
        backend.find_alternative_ik(None)
        self._append_log("Alternative IK search requested.")
        self.refresh_all()

    def capture_zero(self):
        backend.set_current_pose_as_motor_and_ik_zero(None)
        self._append_log("Captured current pose as zero.")
        self.refresh_all()

    def move_to_zero(self):
        moved = backend.move_configured_joints_to_zero_on_connect()
        self._append_log("Move-to-zero requested." if moved else "Move-to-zero skipped.")
        self.refresh_all()

    def toggle_live_execute(self, checked):
        backend.live_update = bool(checked)
        self._append_log(f"Live Execute {'enabled' if checked else 'disabled'}.")

    def toggle_sync(self, checked):
        backend.sync_model_to_telemetry = bool(checked)
        self._append_log(f"Sync to telemetry {'enabled' if checked else 'disabled'}.")
        self.refresh_all()

    def toggle_mirror(self, checked):
        backend.DISPLAY_MIRROR_X = bool(checked)
        self._append_log(f"Display mirror {'enabled' if checked else 'disabled' }.")
        self.refresh_all(skip_inputs=True)

    def sync_target_from_telemetry(self):
        if backend.sync_solver_seed_from_telemetry(update_target=True):
            self._append_log("Target synchronized from telemetry pose.")
        else:
            self._append_log("Target sync from telemetry skipped: no telemetry.")
        self.refresh_all()

    def adjust_camera(self, elev_delta, azim_delta):
        backend.cam_elev = max(-90.0, min(90.0, backend.cam_elev + elev_delta))
        backend.cam_azim += azim_delta
        self.refresh_all(skip_inputs=True)

    def reset_camera(self):
        backend.cam_elev = 20.0
        backend.cam_azim = -60.0
        self.refresh_all(skip_inputs=True)

    def _sorted_pressed_keys(self):
        return sorted(self._pressed_keys, key=lambda v: (len(v), v))

    def _pressed_actions(self):
        actions = set()
        for action, key_name in self.keymap.items():
            if key_name and key_name != "None" and key_name.lower() in self._pressed_keys:
                actions.add(action)
        return actions

    def _event_to_key_name(self, event):
        key = event.key()
        text = event.text().upper() if event.text() else ""
        if key == Qt.Key_Left:
            return "Left"
        if key == Qt.Key_Right:
            return "Right"
        if key == Qt.Key_Up:
            return "Up"
        if key == Qt.Key_Down:
            return "Down"
        if key == Qt.Key_Space:
            return "Space"
        if text in {k for k in KEY_OPTIONS if len(k) == 1 and k.isalpha()}:
            return text
        return None

    def _keyboard_tick(self):
        if not self._pressed_keys:
            self._key_action_fired.clear()
            return

        move_step = float(self.step_input.value()) if hasattr(self, "step_input") else 0.01
        rot_step = 2.0
        cam_step = 3.0
        changed = False

        actions = self._pressed_actions()

        if "move_x_neg" in actions:
            backend.current_target_xyz[0] -= move_step
            changed = True
        if "move_x_pos" in actions:
            backend.current_target_xyz[0] += move_step
            changed = True
        if "move_y_pos" in actions:
            backend.planar_target_y += move_step
            changed = True
        if "move_y_neg" in actions:
            backend.planar_target_y -= move_step
            changed = True
        if "move_z_neg" in actions:
            backend.current_target_xyz[2] -= move_step
            changed = True
        if "move_z_pos" in actions:
            backend.current_target_xyz[2] += move_step
            changed = True
        if "roll_neg" in actions:
            backend.current_target_rpy[0] -= rot_step
            changed = True
        if "roll_pos" in actions:
            backend.current_target_rpy[0] += rot_step
            changed = True
        if "pitch_pos" in actions:
            backend.current_target_rpy[1] += rot_step
            changed = True
        if "pitch_neg" in actions:
            backend.current_target_rpy[1] -= rot_step
            changed = True
        if "yaw_neg" in actions:
            backend.current_target_rpy[2] -= rot_step
            changed = True
        if "yaw_pos" in actions:
            backend.current_target_rpy[2] += rot_step
            changed = True

        if "cam_left" in actions:
            backend.cam_azim -= cam_step
            changed = True
        if "cam_right" in actions:
            backend.cam_azim += cam_step
            changed = True
        if "cam_up" in actions:
            backend.cam_elev = max(-90.0, min(90.0, backend.cam_elev + cam_step))
            changed = True
        if "cam_down" in actions:
            backend.cam_elev = max(-90.0, min(90.0, backend.cam_elev - cam_step))
            changed = True

        edge_actions = {
            "execute": self.execute_now,
            "estop": self.estop,
            "preview": self.preview_plan,
            "sync_target": self.sync_target_from_telemetry,
        }
        for action, callback in edge_actions.items():
            if action in actions and action not in self._key_action_fired:
                callback()
                self._key_action_fired.add(action)
            elif action not in actions:
                self._key_action_fired.discard(action)

        if not changed:
            return

        backend.current_target_xyz[0] = max(-1.0, min(1.0, backend.current_target_xyz[0]))
        backend.planar_target_y = max(-1.0, min(1.0, backend.planar_target_y))
        backend.current_target_xyz[1] = backend.planar_target_y if backend.PLANAR_MODE else max(-1.0, min(1.0, backend.current_target_xyz[1]))
        backend.current_target_xyz[2] = max(0.0, min(1.5, backend.current_target_xyz[2]))
        backend.current_target_rpy = [
            max(-180.0, min(180.0, float(v)))
            for v in backend.current_target_rpy
        ]

        solved = backend.solve_ik_for_target(backend.current_joint_angles)
        if solved is not None:
            backend.current_joint_angles = list(solved)
            backend.refresh_locked_joint_rads(backend.current_joint_angles)
        self.refresh_all(skip_inputs=False)

    def keyPressEvent(self, event):
        mapped = self._event_to_key_name(event)
        if mapped is not None:
            self._pressed_keys.add(mapped.lower())
            self.refresh_all(skip_inputs=True)
            event.accept()
            return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        mapped = self._event_to_key_name(event)
        if mapped is not None and mapped.lower() in self._pressed_keys:
            self._pressed_keys.discard(mapped.lower())
            self.refresh_all(skip_inputs=True)
            event.accept()
            return
        super().keyReleaseEvent(event)

    def on_target_changed(self):
        if self._building_controls:
            return
        backend.current_target_xyz = [
            float(self.target_inputs["x"].value()),
            float(self.target_inputs["y"].value()),
            float(self.target_inputs["z"].value()),
        ]
        backend.current_target_rpy = [
            float(self.target_inputs["roll"].value()),
            float(self.target_inputs["pitch"].value()),
            float(self.target_inputs["yaw"].value()),
        ]
        backend.planar_target_y = float(self.target_inputs["y"].value())
        solved = backend.solve_ik_for_target(backend.current_joint_angles)
        if solved is not None:
            backend.current_joint_angles = list(solved)
            backend.refresh_locked_joint_rads(backend.current_joint_angles)
        self.refresh_all(skip_inputs=True)

    def on_joint_changed(self, joint_idx, value):
        if self._building_controls:
            return
        if joint_idx < len(backend.current_joint_angles):
            backend.current_joint_angles[joint_idx] = np.radians(float(value))
        backend.current_joint_angles = backend.enforce_locked_joints(backend.current_joint_angles)
        backend.refresh_locked_joint_rads(backend.current_joint_angles)
        solved = backend.solve_ik_for_target(backend.current_joint_angles)
        if solved is not None:
            backend.current_joint_angles = list(solved)
            backend.refresh_locked_joint_rads(backend.current_joint_angles)
            backend.sync_target_from_joint_angles(backend.current_joint_angles)
        self.refresh_all(skip_inputs=True)

    def _live_execute_tick(self):
        if backend.live_update:
            now = time.time()
            if now - backend.last_send_time > 0.1:
                self._send_current_joint_target(smooth=False, repeat_count=1)
                self.refresh_all(skip_inputs=True)

    def refresh_all(self, skip_inputs=False):
        self._building_controls = True
        try:
            if not skip_inputs:
                self.target_inputs["x"].setValue(float(backend.current_target_xyz[0]))
                self.target_inputs["y"].setValue(float(backend.planar_target_y if backend.PLANAR_MODE else backend.current_target_xyz[1]))
                self.target_inputs["z"].setValue(float(backend.current_target_xyz[2]))
                self.target_inputs["roll"].setValue(float(backend.current_target_rpy[0]))
                self.target_inputs["pitch"].setValue(float(backend.current_target_rpy[1]))
                self.target_inputs["yaw"].setValue(float(backend.current_target_rpy[2]))
                for joint_idx in range(1, 7):
                    if joint_idx < len(backend.current_joint_angles):
                        self.joint_inputs[joint_idx].setValue(float(np.degrees(backend.current_joint_angles[joint_idx])))

            connected = bool(backend.ser and backend.ser.is_open)
            if connected and backend.has_fresh_telemetry():
                self.btn_connect.setText("Connected")
                self.btn_connect.setStyleSheet("background-color: lightgreen;")
            elif connected:
                self.btn_connect.setText("Port Open")
                self.btn_connect.setStyleSheet("background-color: khaki;")
            else:
                self.btn_connect.setText("Disconnected")
                self.btn_connect.setStyleSheet("background-color: lightcoral;")

            self.btn_estop.setStyleSheet("background-color: salmon; font-weight: bold;")
            self.status_label.setText(backend.get_connection_status_text() + f"\nLast Telemetry: {backend.last_telemetry_line}")
            self.chk_live.setChecked(bool(backend.live_update))
            self.chk_sync.setChecked(bool(backend.sync_model_to_telemetry))
            self.chk_mirror.setChecked(bool(backend.DISPLAY_MIRROR_X))
            pressed = self._sorted_pressed_keys()
            self.keys_status.setText("Pressed: " + (", ".join(pressed) if pressed else "<none>"))
            active_actions = self._pressed_actions()
            for action, label in self.key_indicator_labels.items():
                key_name = self.keymap.get(action, "None")
                label.setText(key_name)
                if action in active_actions:
                    label.setStyleSheet("background-color: #6fda8c; color: #0b2b12; border: 1px solid #2d7a40; border-radius: 4px; font-weight: bold;")
                else:
                    label.setStyleSheet("background-color: #e9ecef; color: #222; border: 1px solid #c5c9ce; border-radius: 4px; font-weight: bold;")

            telemetry_rows = []
            for joint_idx in range(1, 7):
                pos = backend.latest_motor_pos_deg.get(joint_idx)
                apos = backend.latest_motor_apos_deg.get(joint_idx)
                line = f"J{joint_idx}: POS={pos if pos is not None else '---'}  APOS={apos if apos is not None else '---'}"
                telemetry_rows.append(line)
            wrist = get_wrist_dashboard_snapshot()
            if wrist is not None:
                telemetry_rows.append("")
                telemetry_rows.append("Wrist Differential:")
                telemetry_rows.append(
                    f"Pitch J{wrist['pitch_joint_id']}: target={_fmt_deg(wrist['pitch_target']).strip()} current={_fmt_deg(wrist['pitch_actual']).strip()} enc=M{wrist['pitch_encoder_motor_id']}"
                )
                telemetry_rows.append(
                    f"Roll  J{wrist['roll_joint_id']}: target={_fmt_deg(wrist['roll_target']).strip()} current={_fmt_deg(wrist['roll_actual']).strip()} enc=M{wrist['roll_encoder_motor_id']}"
                )
                telemetry_rows.append(
                    f"Mixed motor cmds: M{wrist['motor_a_id']}={_fmt_rpm(wrist['motor_a_cmd']).strip()} RPM  M{wrist['motor_b_id']}={_fmt_rpm(wrist['motor_b_cmd']).strip()} RPM"
                )
            telemetry_rows.append("")
            telemetry_rows.append("Raw:")
            telemetry_rows.append(backend.last_telemetry_line)
            self.telemetry_view.setPlainText("\n".join(telemetry_rows))

            for row, joint_idx in enumerate(range(1, 7)):
                tgt = float(np.degrees(backend.current_joint_angles[joint_idx])) if joint_idx < len(backend.current_joint_angles) else 0.0
                cur = backend.get_current_ik_deg_from_apos(joint_idx)
                err = None if cur is None else abs(cur - tgt)
                items = [
                    QTableWidgetItem(f"J{joint_idx}"),
                    QTableWidgetItem(f"{tgt:7.2f}°"),
                    QTableWidgetItem("---" if cur is None else f"{cur:7.2f}°"),
                    QTableWidgetItem("---" if err is None else f"{err:6.2f}°"),
                ]
                for col, item in enumerate(items):
                    item.setTextAlignment(Qt.AlignCenter)
                    self.joint_table.setItem(row, col, item)

            status_snapshot = backend.get_connection_status_text() + "\n" + backend.last_telemetry_line
            if status_snapshot != self._last_status_snapshot:
                self._append_log("Status updated.")
                self._last_status_snapshot = status_snapshot

            self.canvas.draw_arm()
        finally:
            self._building_controls = False

    def closeEvent(self, event):
        try:
            if backend.ser and backend.ser.is_open:
                backend.disconnect_serial()
        finally:
            super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    window = DashboardWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
