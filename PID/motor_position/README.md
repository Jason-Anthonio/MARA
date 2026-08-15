# Motor Position PID Tuning & System Identification

Welcome to the `PID` directory for the **MARA (Modular Adaptive Robot Arm)** project\! This folder contains all the firmware, data-logging scripts, experimental datasets, and math routines we used to model our DC motors and tune the joint position controllers.

---

## 📌 Project Overview

For robotic arms, precision and stability are everything. If a joint overshoots, the gripper could collide with obstacles or shake violently. To get accurate positioning without oscillations, we used an empirical **First-Order Plus Dead Time (FOPDT)** model combined with the **Chien-Hrones-Reswick (CHR)** tuning method.

---

## 📂 Folder Structure

PID/motor\_position/

├── csv\_datasets/

│   ├── motor\_id\_data\_train\_001.csv      \# Step response training log (PWM, time, ticks)

│   └── motor\_id\_data\_train\_002.csv      \# Validation dataset

├── data\_log\_position/

│   ├── serial\_writing\_motordata.ino     \# Arduino firmware: applies step PWM & streams encoder ticks

│   └── serial\_reading\_datatocsv.py      \# Python script: listens over UART and saves data to CSV

├── FOPDT\_Math.py                        \# System ID, Savitzky-Golay filtering, & CHR PID calculation

├── rootlocus.png                        \# Root locus plot showing closed-loop stability in LHP

├── stepresponse(before\&after).png       \# Step response comparison before vs. after CHR tuning

└── README.md                            \# This documentation file

---

## ⚙️ Hardware Specifications

* **Motor Model**: JGA25-370 DC Geared Motor with integrated Hall-effect encoder  
* **Encoder Resolution**: 1133 Counts Per Revolution (CPR)  
* **Microcontroller**: Arduino / ESP32  
* **Step Input Used**: `150 Digital PWM` (\~59% duty cycle, \~3.696 rad/s).  
  *Why 150?* It sits right in the linear operating zone, avoiding both the low-end static friction deadband and the high-end voltage saturation.

---

## 💡 Key Methods & "Why We Did It This Way"

### 1\. Velocity FOPDT Modeling (Instead of Direct Position Modeling)

* **Concept**: DC motors physically behave as 1st-order velocity systems (due to rotor inertia $J$, damping $b$, resistance $R$, and back-EMF $K\_e$).  
* **Why**: Fitting a 1st-order step curve to velocity ($\\omega(t)$) to find gain $K$ and mechanical time constant $\\tau$ is much simpler and more reliable than fitting a 2nd-order curve to raw position directly.  
* **Math**:  
  * **Velocity Transfer Function**: $$G\_v(s) \= \\frac{\\Omega(s)}{U(s)} \= \\frac{K}{\\tau s \+ 1} \= \\frac{0.0249}{0.02s \+ 1}$$  
  * **Position Transfer Function** (integrating velocity by dividing by $s$): $$G\_p(s) \= \\frac{\\Theta(s)}{U(s)} \= \\frac{K}{s(\\tau s \+ 1)} \= \\frac{0.0249}{0.02s^2 \+ s}$$

---

### 2\. Savitzky-Golay Filtering for Numerical Derivatives

* **Concept**: To get velocity from discrete encoder position ticks, we compute the numerical derivative ($\\frac{\\Delta \\text{pos}}{\\Delta t}$).  
* **Why it's unique**: Discrete encoder ticks create jagged, high-frequency quantization noise when differentiated. Standard low-pass or moving-average filters introduce unwanted phase lag that blurs the fast initial acceleration transient.  
* **Solution**: We applied a **Savitzky-Golay filter** (`scipy.signal.savgol_filter`, window length 31, polynomial order 3). It fits local polynomials through the data, eliminating quantization noise while preserving the true physical acceleration slope needed to extract $\\tau$.

---

### 3\. Chien-Hrones-Reswick (CHR) Tuning (0% Overshoot Setpoint Tracking)

* **Concept**: An analytical tuning method based on FOPDT parameters ($K$ and $\\tau$).  
* **Why not standard Ziegler-Nichols (ZN)?**: Standard ZN aims for a \~25% overshoot with aggressive quarter-amplitude decay. For industrial/assistive robot arms, 25% overshoot causes joint slamming, gearbox wear, and collision hazards.  
* **Solution**: We chose **CHR 0% Overshoot Tuning** to achieve a **critically damped** response. The joint moves swiftly to the target angle and stops cleanly with zero overshoot.

---

### 4\. PD Control Architecture ($K\_i \= 0$)

* **Concept**: We use a Proportional-Derivative (PD) controller rather than full PID.  
* **Why**: The position transfer function $G\_p(s) \= \\frac{K}{s(\\tau s \+ 1)}$ already contains a natural open-loop integrator ($\\frac{1}{s}$), making the plant a **Type 1 system**. Type 1 systems inherently achieve **zero steady-state error** for step position inputs. Adding an active integrator term ($K\_i$) would introduce unnecessary phase lag and risk integrator windup/overshoot.

---

## 📊 Experimental Results

| Parameter | Value | Unit |
| :---- | :---- | :---- |
| **Steady-State Angular Velocity ($\\omega\_{ss}$)** | `3.73` | rad/s |
| **Steady-State Gain ($K$)** | `0.0249` | (rad/s) / PWM |
| **Mechanical Time Constant ($\\tau$)** | `0.020` | seconds (time to reach 63.2% $\\omega\_{ss}$) |
| **Dead Time ($L$ / $\\theta\_p$)** | `~0.0` | seconds (negligible hardware delay) |

### Scaled PID Values (for Firmware in Encoder Ticks):

// Scaled for 1133 CPR encoder

float kp \= 0.0494;

float ki \= 0.0000;

float kd \= 0.0108;

* **Before Tuning**: P-only control showed notable steady-state error and sluggish positioning.  
* **After CHR Tuning**: Fast rise time, critically damped arrival, and clean target lock at $360^\\circ$ (1133 ticks) with 0 steady-state error and 0% overshoot.  
* **Root Locus Verification**: All closed-loop poles reside safely in the Left-Half Plane (LHP), confirming robust stability.

---

## 🚀 How to Replicate & Run

1. **Flash Data Logger**:  
   * Open `data_log_position/serial_writing_motordata.ino` in Arduino IDE and flash to your microcontroller.  
2. **Collect Step Response**:  
   * Run `python3 data_log_position/serial_reading_datatocsv.py`. Press `'f'` in the terminal to trigger the 1.5s step test and save the data to CSV.  
3. **Run Identification & Tuning**:  
   * Run `python3 FOPDT_Math.py` to smooth the data, extract $K$ and $\\tau$, print the transfer functions, and output the recommended $K\_p$ and $K\_d$ gains.  
4. **Deploy**:  
   * Paste the output $K\_p$ and $K\_d$ values into the MARA motor controller firmware.

---

*Developed by the MARA Control Systems Student Team (Gavin, Calvin, Jason, Meru).*  
