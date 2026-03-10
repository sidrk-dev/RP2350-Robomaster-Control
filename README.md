# RoboMaster Motor Control (Pico + MCP2515)

Control up to 8 DJI RoboMaster motors (M3508/M2006) using a Raspberry Pi Pico and MCP2515 CAN Bus module. This project provides a serial command interface to control position, speed, PID tuning, calibration, and homing.

## Features

- **Position Control**: Move motors to absolute angles (0-360 degrees).
- **Speed Control**: Set continuous rotation speed (RPM).
- **PID Tuning**: Adjust Angle and Speed PID gains on the fly.
- **Calibration**: Set the current physical position to a specific angle.
- **Gear Ratios**: Configure external gear reductions.
- **Multi-Motor Support**: Control up to 8 motors on a single CAN bus.
- **Brushed DC Support**: Integrated support for Cytron MD10C motor drivers.
- **Configuration Management**: GUI utility to tune PID loops, set gear ratios, and save multi-motor parameters to JSON.
- **Inverse Kinematics UI**: Full 3D 6-DOF visualization and control panel for coordinate based movement.
- **Python Control Scripts**: Control the RoboMaster and Cytron motors directly from your laptop using keyboard inputs.

## Hardware Setup

### Components
- **Microcontroller**: Raspberry Pi Pico (RP2040)
- **CAN Controller**: MCP2515 Module (SPI)
- **Motors**: DJI RoboMaster M3508 or M2006 (with C620/C610 ESCs)
- **Power Supply**: 24V DC (for motors)

### Wiring (Pico <-> MCP2515)

| Pico Pin | MCP2515 Pin | Function |
| :------- | :---------- | :------- |
| GPIO 4   | SO (MISO)   | SPI RX   |
| GPIO 5   | CS          | Chip Select |
| GPIO 6   | SCK         | SPI Clock |
| GPIO 7   | SI (MOSI)   | SPI TX   |
| VBUS/VSYS| VCC         | 5V Power |
| GND      | GND         | Ground   |

*Note: The MCP2515 INT pin is not currently used in this code.*

> **IMPORTANT**: The Raspberry Pi Pico, MCP2515, and the Motor Controller (ESC) power supply **MUST share a common ground**. Failing to connect the grounds will result in communication failure.

### CAN Bus Wiring
- Connect generic CAN High and CAN Low from the MCP2515 to the CAN High and CAN Low on the motor ESCs.
- Ensure the CAN bus is terminated with 120Ω resistors at both ends.
- **Motor IDs**: Set motor IDs (1-8) on the ESCs according to the DJI documentation (usually by pressing the button on the ESC).

### Cytron MD10C Wiring (Optional)
If you are using a standard brushed DC motor via the Cytron MD10C:
| Pico Pin | MD10C Pin | Function |
| :------- | :-------- | :------- |
| GPIO 26  | PWM       | Motor Speed |
| GPIO 27  | DIR       | Motor Direction |
| GND      | GND       | Ground |

## Software Setup

### 1. Install Arduino IDE
Download and install the latest Arduino IDE from [arduino.cc](https://www.arduino.cc/en/software).

### 2. Install Raspberry Pi Pico Board Support
1. Open Arduino IDE.
2. Go to **File** > **Preferences**.
3. In "Additional Boards Manager URLs", add:
   `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`
4. Go to **Tools** > **Board** > **Boards Manager**.
5. Search for "Pico" and install **"Raspberry Pi Pico/RP2040" by Earle F. Philhower, III**.

### 3. Libraries
All required libraries are included in the `src` folder. No manual library installation is required.

- `autowp-mcp2515` (Modified for local include)
- `RPi_Pico_TimerInterrupt` (Modified for local include)


### 4. Upload Code
1. Open `goo.ino` in Arduino IDE.
2. Connect your Raspberry Pi Pico via USB while holding the BOOTSEL button (or just connect if already in flash mode/serial enabled).
3. Select your board: **Tools** > **Board** > **Raspberry Pi Pico**.
4. Select the Port: **Tools** > **Port**.
5. Click **Upload** (Right arrow icon).

## Code Configuration (Permanent Settings)

To change default settings permanently, edit `goo.ino`:

1.  **Pin Definitions**:
    ```cpp
    #define RX_PIN 4  // gpio4 (MISO)
    #define CS_PIN 5  // gpio5 (CS)
    #define SCK_PIN 6 // gpio6 (SCK)
    #define TX_PIN 7  // gpio7 (MOSI)
    ```
2.  **Motor Count**:
    ```cpp
    #define MOTOR_NUM 8 // Change if using fewer motors
    ```
3.  **Default Gear Ratios and PID**:
    Locate the `setMotorParam()` function.
    -   Modify `motor[i].gearRatio` for M3508 (approx 19.0) or M2006 (36.0).
    -   Modify default PID values in `motor[i].anglePid` and `motor[i].speedPid`.


## Usage (Serial Monitor)

Open the **Serial Monitor** (Tools > Serial Monitor) and set the baud rate to **115200**. Ensure "Both NL & CR" or "Newline" is selected.

### Basic Commands

| Command | Description | Example |
| :------ | :---------- | :------ |
| `H` | Show Help menu | `H` |
| `S` | **STOP ALL MOTORS** | `S` |

### Motor Control

| Command | Description | Example |
| :------ | :---------- | :------ |
| `P <id> <angle>` | Move Motor `<id>` to limits `<angle>` (0-360) | `P 1 90` (Motor 1 to 90°) |
| `M <id> <rpm>` | Run Motor `<id>` at speed `<rpm>` | `M 2 500` (Motor 2 at 500 RPM) |
| `CAL <id> <angle>` | Calibrate: Set current position to `<angle>` | `CAL 1 0` (Set current pos as 0°) |
| `GEAR <id> <ratio>` | Set external gear ratio | `GEAR 1 5` (5:1 reduction) |
| `C <speed>` | Run Cytron motor at speed `-255` to `255` | `C 127` (50% forward) |

### PID Tuning
Adjust the control loop behavior. Values are saved until restart.

| Command | Description | Example |
| :------ | :---------- | :------ |
| `AP <id> <kp> <ki> <kd>` | Set **Angle** PID | `AP 1 10.0 0.0 0.5` |
| `SP <id> <kp> <ki> <kd>` | Set **Speed** PID | `SP 1 2.0 0.1 0.0` |
| `SHOW <id>` | Show current PID values | `SHOW 1` |

## Python Control Scripts

This repository includes Python scripts and graphical interfaces to allow you to control the motors directly from your laptop.
First, make sure to install dependencies:
```bash
pip install ikpy pyserial matplotlib numpy scipy pynput
```

- **`ik_dashboard_qt.py`** (**Primary / Main arm dashboard**): Recommended day-to-day control UI for IK + runtime operation (reconnect handling, verify flow, execute/live controls).
- **`pid_tuner.py`** (**Primary tuning tool**): Dedicated Tkinter GUI for motor parameter maintenance (PID, limits, ENCMAP/ABSPID, and config persistence).
- **`ik_arm_control.py`**: Backend + legacy visualizer internals used by the Qt dashboard.
- **`keyboard_control.py`**: Hold `W` to spin RoboMaster Motor 1 forward, `S` to spin backward. `Q` to quit.
- **`differential_wrist_control.py`**: Uses WASD to control a differential wrist attached to Motors 1 and 2. `W/S` for Pitch, `A/D` for Roll.
- **`cytron_control.py`**: Hold `U` to spin the Cytron motor forward (PWM 255), `J` to spin backward. `Q` to quit.

## Recommended Daily Workflow (Important)

This project has a multi-controller serial architecture (`base`, `joint4`, `wrist`).
Most runtime issues are from startup order, stale telemetry, or encoder mapping/offset mistakes.

### 1) One app at a time
- Do **not** run `ik_dashboard_qt.py` and `pid_tuner.py` at the same time.
- Close one before connecting with the other, or USB ports will conflict.

### 2) Power and cable checks first
- Make sure all expected controller boards are powered and plugged in.
- If one board is unplugged, motion may remain blocked in IK for safety.

### 3) Apply settings before motion
- In PID Tuner, use **Apply ALL Settings to Arm** after loading/checking config.
- In IK Dashboard, after reconnect, click **Post-Reconnect Verify + Arm Motion** before Execute/Live.

### 4) Wait for telemetry
- Confirm live `POS` telemetry appears for joints before testing motion.
- If telemetry is stale/missing, IK motion will be blocked.

---

## IK Dashboard (`ik_dashboard_qt.py`) - How to Use

### Startup
1. Launch `python ik_dashboard_qt.py`
2. Click **Connect**.
3. Wait for connection + telemetry status to become healthy.
4. Click **Post-Reconnect Verify + Arm Motion**.
5. Use **Execute** for one-shot movement or **Live Update** for continuous updates.

### Safety/Behavior Notes
- The dashboard intentionally enforces runtime safety gates:
  - runtime apply must be confirmed,
  - required controllers must be online,
  - telemetry must be live,
  - motion must be armed after verify.
- If any of the above fails, Execute is blocked and a reason is logged.

### Joint 4 note
- If Joint 4 does not move from IK, check whether backend is in planar-lock mode.
- In planar mode, joint 4 can be intentionally locked out from IK command generation.

---

## PID Tuner (`pid_tuner.py`) - How to Use

### Typical sequence
1. Launch `python pid_tuner.py`
2. Connect to controllers.
3. Review motor config values loaded from `robot_config.json`.
4. Click **Apply Encoder Mappings** (or **Apply ALL Settings to Arm**).
5. Wait briefly for encoder detection settle.
6. Apply/enable ABS PID.
7. Test with small moves first (`P`/Jog), then larger movements.

### Encoder mapping and ABSPID sequencing quirk
- ABSPID enable can fail if sent too quickly after mapping.
- Correct order is:
  1. `ENCMAP`
  2. short wait for firmware detection
  3. `ABSPID`
- The tooling now retries ABSPID enables, but you should still wait for telemetry stability.

### Shared encoder index across boards
- Same encoder index value can be reused on different controller boards.
- Conflict only matters **within the same controller** (same board/firmware instance).

---

## Common Errors and Fixes

### 1) "Motion blocked" in IK Dashboard
**Symptoms**
- Execute does nothing
- Logs mention runtime apply/telemetry/controller health

**Fix**
1. Reconnect all boards.
2. Confirm telemetry is updating.
3. Run **Post-Reconnect Verify + Arm Motion**.
4. Retry Execute.

### 2) One controller unplugged / wrong COM port
**Symptoms**
- One joint group (often J4 or wrist) never responds.
- Status shows offline/missing controller.

**Fix**
1. Check USB cable and power for that board.
2. Confirm controller mapping in `robot_config.json` (`controllers.base/joint4/wrist.port`).
3. Reconnect app and verify again.

### 3) ABSPID skipped / not enabled
**Symptoms**
- Serial log includes `ABSPID skipped`.
- Motor stays on internal encoder PID source.

**Likely causes**
- No encoder mapped for that motor.
- Encoder not detected yet right after remap.

**Fix**
1. Set correct `abs_encoder_index` for that motor.
2. Apply encoder mappings.
3. Wait ~0.2-0.5s.
4. Enable ABSPID again (or use Apply ALL).

### 4) Motor moves opposite direction
**Symptoms**
- Position correction runs away or moves opposite expected command.

**Fix**
1. Flip `abs_encoder_reversed` for that motor.
2. Re-apply encoder mapping/direction.
3. Re-enable ABSPID and retest with tiny move.

### 5) Wrong zero / bad encoder offset
**Symptoms**
- Displayed pose is offset from real arm.
- Commanded target goes to unexpected physical angle.

**Fix**
1. Put arm in known reference pose.
2. Recalibrate with `CAL <id> <angle>` (or dashboard zero capture workflow).
3. Verify `abs_zero_offset` / `ik_zero_offset` in `robot_config.json` are correct.
4. Apply settings, then sync from telemetry before executing motion.

### 6) Jog spins continuously instead of stepping
**Symptoms**
- Jog behaves like continuous speed command.

**Fix**
- Use updated PID tuner jog path that sends bounded position targets (`P`) from current telemetry reference.
- Ensure live telemetry is present before jogging.

---

## Quick Command Reference (Firmware)

- `S` -> stop all motors
- `TON` / `TOFF` -> telemetry on/off
- `P <id> <angle>` -> absolute logical position target
- `PR <id> <delta>` -> relative logical position step
- `M <id> <rpm>` -> continuous speed command
- `ENCMAP <enc_idx> <motor_id|0>` -> map/unmap absolute encoder
- `ENCREVSET <enc_idx> <0|1> <motor_id>` -> deterministic encoder direction
- `ABSPID <id> <0|1>` -> toggle absolute-encoder PID source
- `CAL <id> <angle>` -> set logical zero reference at current physical position

## Documentation

For more detailed guides, see the included markdown files:
- [PID Tuning Guide](PID_TUNING_GUIDE.md)
- [Gear Ratio Guide](GEAR_RATIO_GUIDE.md)
- [Multi-Motor Reference](MULTI_MOTOR_REFERENCE.md)
