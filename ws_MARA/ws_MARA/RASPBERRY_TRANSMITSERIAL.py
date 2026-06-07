import serial
import time
import threading

PORT = '/dev/ttyUSB0'
BAUD = 115200

print(f"Connecting to ARDUINO on {PORT}...")
ser = serial.Serial(PORT, BAUD, timeout=0.1)

# 1. THE RESET FIX: Wait 2 seconds for the ESP32 to finish rebooting
time.sleep(2)
ser.reset_input_buffer() 
print("Connected! Type a target angle and press ENTER.\n")

# 2. THE BUFFER FIX: Read Arduino data continuously in the background
def read_serial():
    while True:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"[ARDUINO] {line}")
            except Exception:
                pass
        time.sleep(0.01)

# Start the background reading thread
listener_thread = threading.Thread(target=read_serial, daemon=True)
listener_thread.start()

# 3. Main loop strictly handles your terminal inputs
while True:
    try:
        command = input()
        if command.strip():
            ser.write((command + "\n").encode())
    except KeyboardInterrupt:
        print("\nExiting...")
        ser.close()
        break
