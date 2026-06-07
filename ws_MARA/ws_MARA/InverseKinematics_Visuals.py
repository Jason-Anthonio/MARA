import matplotlib.pyplot as plt
from numpy import *

# --- From RobotArmInverseKinematics.py ---

# Length of links in cm
a1 = 25.0  # shoulder
a2 = 19.0  # elbow
a3 = 6.0  # wrist/camera joint

# Desired Position of End effector
px = -14  # target x
py = 3  # target y

phi = 90  # End effector angle from horizontal
phi = deg2rad(phi)

# Equations for Inverse kinematics
wx = px - a3 * cos(phi) #wrist
wy = py - a3 * sin(phi)

delta = wx**2 + wy**2
c2 = (delta - a1**2 - a2**2) / (2 * a1 * a2) #cosine rule for angle at elbow

# Check if the position is reachable
if abs(c2) > 1:
    print("Error: Target position is not reachable.")
    exit()

s2 = -sqrt(1 - c2**2)  # elbow up configuration (sin2 + cos2 = 1)
# s2 = sqrt(1 - c2**2) # elbow down configuration
theta_2 = arctan2(s2, c2)

s1 = ((a1 + a2 * c2) * wy - a2 * s2 * wx) / delta
c1 = ((a1 + a2 * c2) * wx + a2 * s2 * wy) / delta
theta_1 = arctan2(s1, c1)
theta_3 = phi - theta_1 - theta_2

print('theta_1: ', rad2deg(theta_1))
print('theta_2: ', rad2deg(theta_2))
print('theta_3: ', rad2deg(theta_3))


# --- Visualization --- #NOTE:the results may be 180 degrees clockwise due to the way the starting angle in matplotlib is in the left horizontal direction.

# Joint positions
x0, y0 = 0, 0
x1 = a1 * cos(theta_1)
y1 = a1 * sin(theta_1)
x2 = x1 + a2 * cos(theta_1 + theta_2)
y2 = y1 + a2 * sin(theta_1 + theta_2)
x3 = x2 + a3 * cos(theta_1 + theta_2 + theta_3)
y3 = y2 + a3 * sin(theta_1 + theta_2 + theta_3)

# Plotting
plt.figure()
plt.plot([x0, x1], [y0, y1], 'r-o', linewidth=3, markersize=8, label='Link 1 (Shoulder)')
plt.plot([x1, x2], [y1, y2], 'g-o', linewidth=3, markersize=8, label='Link 2 (Elbow)')
plt.plot([x2, x3], [y2, y3], 'b-o', linewidth=3, markersize=8, label='Link 3 (Wrist)')

# Plot target point
plt.plot(px, py, 'kx', markersize=12, markeredgewidth=3, label='Target Position')

# Plot end effector orientation vector
plt.arrow(x3, y3, 5 * cos(phi), 5 * sin(phi), head_width=1, head_length=1.5, fc='k', ec='k')


# Setting plot limits and labels
max_reach = a1 + a2 + a3
plt.xlim(-max_reach, max_reach)
plt.ylim(-max_reach, max_reach)
plt.gca().set_aspect('equal', adjustable='box')
plt.title('Robot Arm Inverse Kinematics Simulation')
plt.xlabel('X-axis (cm)')
plt.ylabel('Y-axis (cm)')
plt.legend()
plt.grid(True)
plt.show()
