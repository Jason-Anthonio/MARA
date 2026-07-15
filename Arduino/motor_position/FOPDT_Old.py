import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

CSV_FILE = "motor_id_data_valid_001.csv" 
ENCODER_CPR = 1133
PWM_STEP = 150.0  

try:
    data = pd.read_csv(CSV_FILE)
except FileNotFoundError:
    print(f"Error: {CSV_FILE} not found")
    exit()

data = data[pd.to_numeric(data['time_us'], errors='coerce').notnull()] #filters non-numerics
data = data.astype({'time_us': float, 'pwm': float, 'pos': float}) #sections the data by columns

#convert time to seconds and position to radians
time_sec = data["time_us"].values / 1e6
theta_rad = data["pos"].values * 2 * np.pi / ENCODER_CPR

# Calculate Delta Encoder Position
pos_ticks_delta = data["pos"].iloc[-1] - data["pos"].iloc[0]
pos_rad_delta = theta_rad[-1] - theta_rad[0]

velocity_raw = np.gradient(theta_rad, time_sec)

#filter the noisy numerical derivative
window_length = min(31, len(velocity_raw) - 1)
if window_length % 2 == 0: 
    window_length -= 1
velocity_smooth = savgol_filter(velocity_raw, window_length, 3)

#average the last 20% to round up the velocity
tail_index = int(len(velocity_smooth) * 0.8)
omega_ss = np.mean(velocity_smooth[tail_index:])

#find the gain K
K = omega_ss / PWM_STEP

#find the time constant tau where velocity reaches 63.2%
target_velocity = 0.632 * omega_ss
tau_idx = np.argmax(velocity_smooth >= target_velocity)
tau = time_sec[tau_idx]


print("\n========================================")
print("INPUT VARIABLES")
print("========================================")
print(f"pos   Delta Encoder position       {pos_ticks_delta:.2f} ticks")
print(f"                                   ({pos_rad_delta:.4f} rad)")
print("")
print(f"u(t)  PWM Step Input               {int(PWM_STEP)}")
print(f"cpr   Encoder CPR                  {ENCODER_CPR}")
print(f"wss   Steady State Angular Velocity  {omega_ss:.2f} rad/s")
print("========================================\n")

print("\n========================================")
print("SYSTEM CHARACTERISTICS")
print("========================================")
print(f"u(t) - Step Input            : {int(PWM_STEP)} PWM")
print(f"w(t) - Angular Velocity     : {omega_ss:.2f} rad/s (Steady-State)")
print(f"K    - Steady State Gain     : {K:.4f} (rad/s) / PWM")
print(f"tau  - Time Constant         : {tau:.4f} seconds")
print("========================================")


#makes Ki=0 to output a critically damped response
Kp_rad = 1.0 / (K * tau)
Kd_rad = 0.5 / K

print("========================================")
print("MOTOR PHYSICS (FOPDT MODEL)")
print("========================================")
print(f"Max Velocity (w_ss)  : {omega_ss:.2f} rad/s")
print(f"Steady-State Gain (K): {K:.4f} (rad/s) / PWM")
print(f"Time Constant (tau)  : {tau:.4f} sec")

print("\nVelocity Transfer Function:")
print(f"       {K:.4f}")
print(f"-------------------")
print(f" {tau:.4f}s + 1")

print("\nPosition Transfer Function:")
print(f"           {K:.4f}")
print(f"-----------------------")
print(f" s({tau:.4f}s + 1)")


#converts radian PID values to tick values
RAD_PER_TICK = (2 * np.pi) / ENCODER_CPR

Kp_ticks = Kp_rad * RAD_PER_TICK
Kd_ticks = Kd_rad * RAD_PER_TICK

print("\n========================================")
print("ARDUINO PID VALUES (SCALED FOR TICKS)")
print("========================================")
print(f"float kp = {Kp_ticks:.4f};")
print(f"float ki = 0.0000;")
print(f"float kd = {Kd_ticks:.4f};")


# Generate the FOPDT mathematical curve
velocity_fit = omega_ss * (1 - np.exp(-time_sec / tau))

plt.figure(figsize=(10, 6))
plt.plot(time_sec, velocity_raw, alpha=0.3, label="Raw Derivative")
plt.plot(time_sec, velocity_smooth, linewidth=2, label="Smoothed Motor Velocity")
plt.plot(time_sec, velocity_fit, 'r--', linewidth=2, label="Calculated Model Fit")

plt.axhline(target_velocity, color='g', linestyle=':', label="63.2% max velocity")
plt.axvline(tau, color='g', linestyle=':', label=f"Tau = {tau:.3f}s")

plt.title("Motor Step Response: Velocity Identification")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (rad/s)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()