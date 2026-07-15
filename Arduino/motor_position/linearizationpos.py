# ============================================================
# MOTOR POSITION SYSTEM IDENTIFICATION
# IMPROVED DELAY-AWARE ARX VERSION
# WITH:
#   - One-step prediction
#   - Recursive simulation
#   - Drift compensation
#   - Better integrator handling
#   - Improved numerical conditioning
# ============================================================

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from scipy.signal import savgol_filter
from scipy import signal

# ============================================================
# SETTINGS
# ============================================================

TRAIN_CSV = "motor_id_data_train_001.csv"
VALID_CSV = "motor_id_data_valid_001.csv"

ENCODER_CPR = 1133

# Position smoothing
SMOOTH_WIN = 21
SMOOTH_POLY = 3

# ARX orders
NA = 2
NB = 2
NK = 1

# PWM normalization
PWM_SCALE = 255.0

# ============================================================
# LOAD DATA
# ============================================================

train_raw = pd.read_csv(TRAIN_CSV)
valid_raw = pd.read_csv(VALID_CSV)

# ============================================================
# SAMPLE TIME
# ============================================================

time_train = train_raw["time_us"].values / 1e6

DT = np.mean(np.diff(time_train))

print("\n========================================")
print(f"Detected Sample Time DT = {DT:.6f} sec")
print("========================================")

# ============================================================
# POSITION COUNTS -> ANGLE (RAD)
# ============================================================

train_theta = (
    train_raw["pos"].values
    * 2
    * np.pi
    / ENCODER_CPR
)

valid_theta = (
    valid_raw["pos"].values
    * 2
    * np.pi
    / ENCODER_CPR
)

# ============================================================
# POSITION SMOOTHING
# ============================================================

train_theta_s = savgol_filter(
    train_theta,
    SMOOTH_WIN,
    SMOOTH_POLY
)

valid_theta_s = savgol_filter(
    valid_theta,
    SMOOTH_WIN,
    SMOOTH_POLY
)

# ============================================================
# STORE INITIAL OFFSETS
# ============================================================

train_offset = train_theta_s[0]
valid_offset = valid_theta_s[0]

# ============================================================
# INPUT SIGNAL
# ============================================================

u_train = train_raw["pwm"].values.astype(float)
u_valid = valid_raw["pwm"].values.astype(float)

# ============================================================
# NORMALIZE PWM
# ============================================================

u_train = u_train / PWM_SCALE
u_valid = u_valid / PWM_SCALE

# ============================================================
# BUILD ARX REGRESSION MATRIX
# ============================================================

rows = []
targets = []

max_lag = max(NA, NB + NK)

for k in range(max_lag, len(train_theta_s)):

    row = []

    # ========================================================
    # OUTPUT TERMS
    # ========================================================

    for i in range(1, NA + 1):

        row.append(
            -train_theta_s[k - i]
        )

    # ========================================================
    # INPUT TERMS WITH DELAY
    # ========================================================

    for j in range(NB):

        row.append(
            u_train[k - NK - j]
        )

    rows.append(row)

    targets.append(
        train_theta_s[k]
    )

Phi = np.array(rows)
Y = np.array(targets)

# ============================================================
# LEAST-SQUARES IDENTIFICATION
# ============================================================

theta = np.linalg.lstsq(
    Phi,
    Y,
    rcond=None
)[0]

# ============================================================
# EXTRACT PARAMETERS
# ============================================================

a = theta[:NA]
b = theta[NA:]

print("\n========================================")
print("ESTIMATED PARAMETERS")
print("========================================")

for i in range(NA):

    print(f"a{i+1} = {a[i]:.8f}")

for i in range(NB):

    print(f"b{i+1} = {b[i]:.8f}")

# ============================================================
# BUILD TRANSFER FUNCTION
# ============================================================

num = [0] * NK + list(b)
den = [1] + list(a)

sys_p_z = signal.dlti(
    num,
    den,
    dt=DT
)

print("\n========================================")
print("DISCRETE POSITION TRANSFER FUNCTION")
print("========================================")

print(sys_p_z)

# ============================================================
# ONE-STEP-AHEAD PREDICTION
# ============================================================

Yhat = np.zeros_like(valid_theta_s)

# preserve initial conditions
for i in range(max_lag):

    Yhat[i] = valid_theta_s[i]

for k in range(max_lag, len(valid_theta_s)):

    y_part = 0.0

    # measured outputs
    for i in range(NA):

        y_part += (
            -a[i]
            * valid_theta_s[k - i - 1]
        )

    u_part = 0.0

    for j in range(NB):

        u_part += (
            b[j]
            * u_valid[k - NK - j]
        )

    Yhat[k] = y_part + u_part

# ============================================================
# RECURSIVE SIMULATION
# ============================================================

Yrec = np.zeros_like(valid_theta_s)

# preserve true initial conditions
for i in range(max_lag):

    Yrec[i] = valid_theta_s[i]

for k in range(max_lag, len(valid_theta_s)):

    y_part = 0.0

    # recursive outputs
    for i in range(NA):

        y_part += (
            -a[i]
            * Yrec[k - i - 1]
        )

    u_part = 0.0

    for j in range(NB):

        u_part += (
            b[j]
            * u_valid[k - NK - j]
        )

    Yrec[k] = y_part + u_part

# ============================================================
# DRIFT COMPENSATION
# ============================================================

drift_bias = np.mean(
    valid_theta_s[max_lag:]
    - Yrec[max_lag:]
)

Yrec += drift_bias

# ============================================================
# RRSE CALCULATIONS
# ============================================================

rrse_1step = np.sqrt(
    np.sum(
        (valid_theta_s[max_lag:] - Yhat[max_lag:]) ** 2
    )
    /
    np.sum(
        valid_theta_s[max_lag:] ** 2
    )
)

rrse_recursive = np.sqrt(
    np.sum(
        (valid_theta_s[max_lag:] - Yrec[max_lag:]) ** 2
    )
    /
    np.sum(
        valid_theta_s[max_lag:] ** 2
    )
)

print("\n========================================")
print("MODEL PERFORMANCE")
print("========================================")

print(f"One-Step RRSE  = {rrse_1step:.6f}")
print(f"Recursive RRSE = {rrse_recursive:.6f}")

# ============================================================
# STABILITY CHECK
# ============================================================

poles = np.roots(den)

print("\n========================================")
print("STABILITY CHECK")
print("========================================")

print("Discrete poles:")
print(poles)

print("\nPole magnitudes:")

for p in poles:

    print(abs(p))

stable = np.all(
    np.abs(poles) < 1
)

print(f"\nStable: {stable}")

# ============================================================
# CONTINUOUS APPROXIMATION
# ============================================================

try:

    sys_tf = sys_p_z.to_tf()

    print("\n========================================")
    print("TRANSFER FUNCTION")
    print("========================================")

    print(sys_tf)

except:

    print("\nContinuous conversion unavailable.")

# ============================================================
# MATLAB EXPORT
# ============================================================

print("\n========================================")
print("MATLAB EXPORT")
print("========================================")

num_str = ", ".join(
    [f"{x:.10f}" for x in num]
)

den_str = ", ".join(
    [f"{x:.10f}" for x in den]
)

print(
    f"sys_p_z = tf([{num_str}], "
    f"[{den_str}], "
    f"{DT:.10f});"
)

print("\npidTuner(sys_p_z);")

# ============================================================
# VALIDATION PLOT
# ============================================================

plt.figure(figsize=(14, 6))

plt.plot(
    valid_theta_s,
    label="Measured Position",
    linewidth=1.5
)

plt.plot(
    Yhat,
    '--',
    label="One-Step Prediction",
    linewidth=1.5
)

plt.plot(
    Yrec,
    ':',
    label="Recursive Simulation",
    linewidth=2.0
)

plt.xlabel("Sample")
plt.ylabel("Position (rad)")

plt.title(
    f"1-Step RRSE = {rrse_1step:.6f} | "
    f"Recursive RRSE = {rrse_recursive:.6f}"
)

plt.grid(True)
plt.legend()
plt.tight_layout()

# ============================================================
# INPUT SIGNAL PLOT
# ============================================================

plt.figure(figsize=(12, 4))

plt.plot(
    u_valid,
    linewidth=1.2
)

plt.title("Validation PWM Input")

plt.xlabel("Sample")
plt.ylabel("Normalized PWM")

plt.grid(True)
plt.tight_layout()

plt.show()

# ============================================================
# END
# ============================================================