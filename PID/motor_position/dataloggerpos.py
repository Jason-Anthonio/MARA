import serial
import csv
import time
# ---------------- CONFIG ----------------
PORT = "/dev/ttyUSB0"        # CHANGE PORT 
BAUDRATE = 115200
OUTPUT_CSV = "motor_id_data_train_001.csv" # Nama file baru untuk data
TIMEOUT = 1.0
# ---------------------------------------

def main():
    ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)

    time.sleep(2) # Tunggu Arduino reset
    ser.write(b"f")
    ser.flush()

    try:
        with open(OUTPUT_CSV, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time_us", "pwm", "pos"])

            while True:
                raw = ser.readline()

                if not raw:
                    continue

                try:
                    line = raw.decode("utf-8", errors="ignore").strip()
                except Exception:
                    continue

                if not line:
                    continue

                print(line)

                parts = line.split(",")
                if len(parts) != 3:
                    continue

                try:
                    time_us = int(parts[0])
                    pwm = int(parts[1])
                    rpm = float(parts[2])
                except ValueError:
                    continue

                writer.writerow([time_us, pwm, rpm])
                f.flush()

    finally:
        try:
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
