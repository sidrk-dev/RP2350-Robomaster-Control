import argparse
import json
import os
import sys
import threading
import time

import serial
import serial.tools.list_ports

try:
    from pynput import keyboard
except ImportError:
    print("Error: pynput library not found.")
    print("Please install: pip install pynput pyserial")
    sys.exit(1)


DEFAULT_BAUD_RATE = 115200
DEFAULT_SPEED = 50
DEFAULT_KEYMAP_FILE = "multi_motor_keymap.json"


def _norm_key_name(raw):
    if raw is None:
        return None
    s = str(raw).strip().lower()
    if not s:
        return None
    aliases = {
        "escape": "esc",
        "spacebar": "space",
        "return": "enter",
    }
    return aliases.get(s, s)


class MultiMotorKeyboardController:
    def __init__(self, config_path, keymap_path, baud_rate=DEFAULT_BAUD_RATE):
        self.config_path = config_path
        self.keymap_path = keymap_path
        self.baud_rate = int(baud_rate)

        self.controller_cfg = {}
        self.motor_to_controller = {}
        self.serial_by_controller = {}
        self.offline_controllers = set()
        self.write_locks = {}
        self.read_threads = {}
        self.running = True

        self.key_to_actions = {}
        self.motor_speed = {}
        self.motor_key_state = {}
        self.current_cmd_rpm = {}
        self.quit_key = "esc"
        self.stop_all_key = "space"
        self.blink_key = "b"

    def load_robot_config(self):
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"robot_config not found: {self.config_path}")
        with open(self.config_path, "r", encoding="utf-8") as f:
            cfg = json.load(f)

        raw_ctrl = cfg.get("controllers", {}) if isinstance(cfg, dict) else {}
        if not isinstance(raw_ctrl, dict) or not raw_ctrl:
            raise ValueError("robot_config.json missing 'controllers' block")

        self.controller_cfg = {}
        self.motor_to_controller = {}
        for name, c in raw_ctrl.items():
            if not isinstance(c, dict):
                continue
            port = c.get("port")
            motors = c.get("motors", [])
            if not isinstance(motors, list):
                motors = []
            motor_ids = []
            for mid in motors:
                try:
                    mid_i = int(mid)
                    motor_ids.append(mid_i)
                except Exception:
                    pass
            self.controller_cfg[name] = {"port": str(port) if port else None, "motors": motor_ids}
            for mid in motor_ids:
                self.motor_to_controller[mid] = name

        if not self.controller_cfg:
            raise ValueError("No valid controller entries found in robot_config.json")

    def _default_keymap(self):
        motors = sorted(self.motor_to_controller.keys())
        # forward keys then reverse keys, paired by index
        forward_pool = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "0"]
        reverse_pool = ["q", "w", "e", "r", "t", "y", "u", "i", "o", "p"]

        motor_map = {}
        for idx, mid in enumerate(motors):
            if idx < len(forward_pool):
                fwd = forward_pool[idx]
                rev = reverse_pool[idx]
            else:
                # fallback if >10 motors (unlikely here)
                fwd = f"f{mid}"
                rev = f"v{mid}"
            motor_map[str(mid)] = {"forward": fwd, "reverse": rev, "speed": DEFAULT_SPEED}

        return {
            "quit": "esc",
            "stop_all": "space",
            "blink": "b",
            "motors": motor_map,
        }

    def load_or_create_keymap(self):
        if not os.path.exists(self.keymap_path):
            default_map = self._default_keymap()
            with open(self.keymap_path, "w", encoding="utf-8") as f:
                json.dump(default_map, f, indent=2)
            print(f"Created default keymap: {self.keymap_path}")

        with open(self.keymap_path, "r", encoding="utf-8") as f:
            keymap = json.load(f)

        self.quit_key = _norm_key_name(keymap.get("quit", "esc")) or "esc"
        self.stop_all_key = _norm_key_name(keymap.get("stop_all", "space")) or "space"
        self.blink_key = _norm_key_name(keymap.get("blink", "b")) or "b"

        raw_motors = keymap.get("motors", {})
        if not isinstance(raw_motors, dict):
            raise ValueError("Keymap 'motors' must be an object")

        self.key_to_actions = {}
        self.motor_speed = {}
        self.motor_key_state = {}
        self.current_cmd_rpm = {}

        for mid, controller_name in sorted(self.motor_to_controller.items()):
            cfg = raw_motors.get(str(mid), {})
            if not isinstance(cfg, dict):
                cfg = {}
            fwd = _norm_key_name(cfg.get("forward"))
            rev = _norm_key_name(cfg.get("reverse"))
            if not fwd or not rev:
                raise ValueError(f"Keymap missing forward/reverse keys for motor {mid}")
            if fwd == rev:
                raise ValueError(f"Motor {mid} forward/reverse keys cannot be the same")

            try:
                speed = int(cfg.get("speed", DEFAULT_SPEED))
            except Exception:
                speed = DEFAULT_SPEED
            speed = max(1, min(2000, abs(speed)))

            self.motor_speed[mid] = speed
            self.motor_key_state[mid] = {"forward": False, "reverse": False}
            self.current_cmd_rpm[mid] = 0

            self.key_to_actions.setdefault(fwd, []).append((mid, +1))
            self.key_to_actions.setdefault(rev, []).append((mid, -1))

            if controller_name not in self.controller_cfg:
                raise ValueError(f"Motor {mid} maps to unknown controller '{controller_name}'")

    def _available_ports(self):
        return {p.device for p in serial.tools.list_ports.comports()}

    def connect_all(self):
        available = self._available_ports()
        self.offline_controllers = set()

        for name, cfg in self.controller_cfg.items():
            port = cfg.get("port")
            if not port:
                self.offline_controllers.add(name)
                print(f"[WARN] {name}: no COM port configured in robot_config.json")
                continue
            if port not in available:
                self.offline_controllers.add(name)
                print(f"[WARN] {name}:{port} not detected (controller offline/unplugged)")
                continue
            try:
                conn = serial.Serial(port, self.baud_rate, timeout=0.1)
                time.sleep(2.0)
                self.serial_by_controller[name] = conn
                self.write_locks[name] = threading.Lock()
                t = threading.Thread(target=self._read_loop, args=(name,), daemon=True)
                self.read_threads[name] = t
                t.start()
            except Exception as exc:
                self.offline_controllers.add(name)
                print(f"[WARN] Failed to open {name}:{port} -> {exc}")

        if not self.serial_by_controller:
            raise RuntimeError("No configured controller ports are currently connected/open.")

        self.broadcast("TON")
        print("Connected controllers:")
        for name, cfg in self.controller_cfg.items():
            if name in self.serial_by_controller:
                print(f"  - {name}: {cfg.get('port')} motors={cfg.get('motors', [])}")
        if self.offline_controllers:
            missing_text = ", ".join(
                f"{name}:{self.controller_cfg.get(name, {}).get('port') or '-'}"
                for name in sorted(self.offline_controllers)
            )
            print(f"[WARN] Not all configured controllers are connected: {missing_text}")

    def _read_loop(self, controller_name):
        conn = self.serial_by_controller.get(controller_name)
        while self.running and conn and conn.is_open:
            try:
                if conn.in_waiting > 0:
                    _ = conn.readline()
                time.sleep(0.01)
            except Exception:
                break

    def send_to_controller(self, controller_name, cmd):
        conn = self.serial_by_controller.get(controller_name)
        if not conn or not conn.is_open:
            return
        lock = self.write_locks.get(controller_name)
        if lock is None:
            return
        try:
            with lock:
                conn.write((cmd + "\n").encode("utf-8"))
        except Exception as exc:
            print(f"Write failed ({controller_name}): {exc}")

    def broadcast(self, cmd):
        for name in sorted(self.serial_by_controller.keys()):
            self.send_to_controller(name, cmd)

    def _key_name(self, key):
        try:
            if hasattr(key, "char") and key.char:
                return key.char.lower()
            if hasattr(key, "name") and key.name:
                return key.name.lower()
        except Exception:
            return None
        return None

    def _apply_motor_targets(self):
        for mid in sorted(self.motor_to_controller.keys()):
            state = self.motor_key_state[mid]
            rpm = 0
            if state["forward"] and not state["reverse"]:
                rpm = self.motor_speed[mid]
            elif state["reverse"] and not state["forward"]:
                rpm = -self.motor_speed[mid]

            if rpm == self.current_cmd_rpm[mid]:
                continue

            self.current_cmd_rpm[mid] = rpm
            controller_name = self.motor_to_controller[mid]
            cmd = f"M {mid} {rpm}"
            self.send_to_controller(controller_name, cmd)
            print(f"[{controller_name}] {cmd}")

    def on_press(self, key):
        name = self._key_name(key)
        if not name:
            return

        if name == self.quit_key:
            print("Quit key pressed.")
            return False
        if name == self.stop_all_key:
            print("STOP ALL")
            self.broadcast("S")
            for mid in self.current_cmd_rpm:
                self.current_cmd_rpm[mid] = 0
                self.motor_key_state[mid]["forward"] = False
                self.motor_key_state[mid]["reverse"] = False
            return
        if name == self.blink_key:
            self.broadcast("BLINK")
            return

        actions = self.key_to_actions.get(name, [])
        changed = False
        for mid, sign in actions:
            slot = "forward" if sign > 0 else "reverse"
            if not self.motor_key_state[mid][slot]:
                self.motor_key_state[mid][slot] = True
                changed = True
        if changed:
            self._apply_motor_targets()

    def on_release(self, key):
        name = self._key_name(key)
        if not name:
            return

        actions = self.key_to_actions.get(name, [])
        changed = False
        for mid, sign in actions:
            slot = "forward" if sign > 0 else "reverse"
            if self.motor_key_state[mid][slot]:
                self.motor_key_state[mid][slot] = False
                changed = True
        if changed:
            self._apply_motor_targets()

    def print_controls(self):
        print("\n=== Multi Motor Keyboard Velocity Control ===")
        print("How to use:")
        print("  1) Keep firmware running on every controller board listed in robot_config.json.")
        print("  2) Start this script, then press/hold each mapped key to command motor RPM.")
        print("  3) Release the key to stop that motor (sends M <id> 0).")
        print("  4) Use Stop-All if anything behaves unexpectedly.")
        print("  5) Press Quit to exit (script sends stop before closing ports).")
        print("")
        print("Runtime controls:")
        print(f"Quit: {self.quit_key} | Stop all: {self.stop_all_key} | Blink: {self.blink_key}")
        print("")
        print("Per-motor bindings:")
        for mid in sorted(self.motor_to_controller.keys()):
            controller_name = self.motor_to_controller[mid]
            speed = self.motor_speed[mid]
            ctrl_state = "ONLINE" if controller_name in self.serial_by_controller else "OFFLINE"
            # reverse lookup for display
            fwd_key = None
            rev_key = None
            for k, actions in self.key_to_actions.items():
                for am, sign in actions:
                    if am == mid and sign > 0:
                        fwd_key = k
                    if am == mid and sign < 0:
                        rev_key = k
            print(f"M{mid} ({controller_name},{ctrl_state}): forward={fwd_key} reverse={rev_key} speed={speed} rpm")
        print("")
        print(f"Keymap file: {self.keymap_path}")
        print("Edit that file to remap keys/speeds, then restart this script.")
        print("============================================\n")

    def close(self):
        self.running = False
        try:
            self.broadcast("S")
            time.sleep(0.1)
        except Exception:
            pass
        for conn in self.serial_by_controller.values():
            try:
                if conn and conn.is_open:
                    conn.close()
            except Exception:
                pass


def main():
    parser = argparse.ArgumentParser(description="Multi-controller keyboard velocity control for RoboMaster motors")
    parser.add_argument("--config", default="robot_config.json", help="Path to robot_config.json")
    parser.add_argument("--keymap", default=DEFAULT_KEYMAP_FILE, help="Path to keymap JSON (created if missing)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD_RATE, help="Serial baud rate")
    args = parser.parse_args()

    app = MultiMotorKeyboardController(args.config, args.keymap, args.baud)

    try:
        app.load_robot_config()
        app.load_or_create_keymap()
        app.connect_all()
        app.print_controls()

        with keyboard.Listener(on_press=app.on_press, on_release=app.on_release) as listener:
            listener.join()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Error: {exc}")
        sys.exit(1)
    finally:
        app.close()
        print("Exited.")


if __name__ == "__main__":
    main()
