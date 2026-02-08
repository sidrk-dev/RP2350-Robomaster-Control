# RoboMaster Motor Control (Pico + MCP2515)

Control up to 8 DJI RoboMaster motors (M3508/M2006) using a Raspberry Pi Pico and MCP2515 CAN Bus module. This project provides a serial command interface to control position, speed, PID tuning, calibration, and homing.

## Features

- **Position Control**: Move motors to absolute angles (0-360 degrees).
- **Speed Control**: Set continuous rotation speed (RPM).
- **PID Tuning**: Adjust Angle and Speed PID gains on the fly.
- **Calibration**: Set the current physical position to a specific angle.
- **Homing**: Automatically find physical limits (e.g., endstops) using current sensing (stall detection).
- **Gear Ratios**: Configure external gear reductions.
- **Multi-Motor Support**: Control up to 8 motors on a single CAN bus.

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

> **⚠️ IMPORTANT**: The Raspberry Pi Pico, MCP2515, and the Motor Controller (ESC) power supply **MUST share a common ground**. Failing to connect the grounds will result in communication failure.

### CAN Bus Wiring
- Connect generic CAN High and CAN Low from the MCP2515 to the CAN High and CAN Low on the motor ESCs.
- Ensure the CAN bus is terminated with 120Ω resistors at both ends.
- **Motor IDs**: Set motor IDs (1-8) on the ESCs according to the DJI documentation (usually by pressing the button on the ESC).

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

### PID Tuning
Adjust the control loop behavior. Values are saved until restart.

| Command | Description | Example |
| :------ | :---------- | :------ |
| `AP <id> <kp> <ki> <kd>` | Set **Angle** PID | `AP 1 10.0 0.0 0.5` |
| `SP <id> <kp> <ki> <kd>` | Set **Speed** PID | `SP 1 2.0 0.1 0.0` |
| `SHOW <id>` | Show current PID values | `SHOW 1` |

### Homing (Endstop Detection)
Configure automatic homing based on stall detection (current limit).

1. **Configure Parameters** (Optional, applied to all motors):
   `HOMECFG <speed> <current_threshold> <time_ms>`
   - `speed`: Homing speed in RPM (negative for reverse). default: -5.0
   - `current_threshold`: Current in Amps to trigger stall. default: 1.75A
   - `time_ms`: Time in ms to hold stall before confirming. default: 300ms
   
   *Example*: `HOMECFG -10 2.5 500`

2. **Start Homing**:
   `HOME <id>`
   *Example*: `HOME 1` (Motor 1 runs until it hits a hard stop, then sets position to 0).

## Documentation

For more detailed guides, see the included markdown files:
- [Homing Guide](HOMING_GUIDE.md)
- [Homing Configuration](HOMECFG_GUIDE.md)
- [PID Tuning Guide](PID_TUNING_GUIDE.md)
- [Gear Ratio Guide](GEAR_RATIO_GUIDE.md)
- [Multi-Motor Reference](MULTI_MOTOR_REFERENCE.md)
