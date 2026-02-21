import serial
import serial.tools.list_ports
import time
import sys
import threading

try:
    from pynput import keyboard
except ImportError:
    print("Error: pynput library not found.")
    print("Please install it running: pip install pynput pyserial")
    sys.exit(1)

# Configurationuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuujjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuujjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuujjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjj
BAUD_RATE = 115200
CYTRON_SPEED = 255  # Max speed for MD10C (0-255)

ser = None
forward_pressed = False
reverse_pressed = False

def find_serial_port():
    ports = serial.tools.list_ports.comports()
    if not ports:
        return None
    for port in ports:
        if "CH340" in port.description or "Arduino" in port.description or "Serial" in port.description:
            return port.device
    return ports[0].device

def send_command(cmd):
    if ser and ser.is_open:
        try:
            ser.write((cmd + '\n').encode('utf-8'))
        except Exception as e:
            pass

def on_press(key):
    global forward_pressed, reverse_pressed
    try:
        if hasattr(key, 'char') and key.char:
            char = key.char.lower()
            if char == 'u' and not forward_pressed:
                forward_pressed = True
                print(f"Spinning Cytron Motor Forward ({CYTRON_SPEED})...")
                send_command(f"C {CYTRON_SPEED}")
            elif char == 'j' and not reverse_pressed:
                reverse_pressed = True
                print(f"Spinning Cytron Motor Reverse ({-CYTRON_SPEED})...")
                send_command(f"C {-CYTRON_SPEED}")
            elif char == 'q':
                print("Quitting...")
                return False # Stop listener
    except AttributeError:
        pass

def on_release(key):
    global forward_pressed, reverse_pressed
    try:
        if hasattr(key, 'char') and key.char:
            char = key.char.lower()
            if char == 'u':
                forward_pressed = False
                if not reverse_pressed:
                    print("Stopping Cytron Motor...")
                    send_command("C 0")
            elif char == 'j':
                reverse_pressed = False
                if not forward_pressed:
                    print("Stopping Cytron Motor...")
                    send_command("C 0")
    except AttributeError:
        pass

def read_from_port():
    while ser and ser.is_open:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                pass
            time.sleep(0.01)
        except Exception:
            break

def main():
    global ser
    print("Looking for Arduino serial port...")
    port_name = find_serial_port()
    
    if not port_name:
        print("No serial ports found! Please plug in your device.")
        return

    print(f"Connecting to {port_name}...")
    try:
        ser = serial.Serial(port_name, BAUD_RATE, timeout=0.1)
    except Exception as e:
        print(f"Failed to connect to {port_name}: {e}")
        return

    time.sleep(2) # Give Arduino some time to reset upon connection
    print("Connected successfully!")
    print("\n--- Cytron MD10C Controls ---")
    print("Hold 'U' to spin forward")
    print("Hold 'J' to spin backward")
    print("Press 'Q' to quit")
    print("-----------------------------\n")

    # Start a thread to read incoming messages from Arduino
    read_thread = threading.Thread(target=read_from_port, daemon=True)
    read_thread.start()

    # Start keyboard listener
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

    # Cleanup
    if ser and ser.is_open:
        send_command("S") # Stop everything before closing
        time.sleep(0.1)
        ser.close()
    print("Exited.")

if __name__ == "__main__":
    main()
