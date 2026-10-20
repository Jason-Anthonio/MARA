# Transformation Matrices - marav3best

Homogeneous transformation matrices between consecutive frames.
Convention: URDF RPY (XYZ extrinsic / ZYX intrinsic).

## Notation

### Frames

| Index | Link |
|-------|------|
| $L_{0}$ | base_link |
| $L_{1}$ | RG_shoulder_link |
| $L_{2}$ | RG_shoulder_arm |
| $L_{3}$ | RG_elbow_link |
| $L_{4}$ | RG_elbow_arm |
| $L_{5}$ | RG_wrist_link |
| $L_{6}$ | RG_wrist_arm |
| $L_{7}$ | RG_gripper_horn |
| $L_{8}$ | marav3best_gripper_2nd_link |
| $L_{9}$ | marav3best_gripper_2nd_link_2 |
| $L_{10}$ | marav3best_gripper |
| $L_{11}$ | marav3best_gripper_2 |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | J1_yaw_base_shoulderLink | continuous (rad) | $L_{0}$ | $L_{1}$ |
| $q_{2}$ | J2_pitch_shoulder | continuous (rad) | $L_{1}$ | $L_{2}$ |
| $q_{3}$ | J4_pitch_elbow | continuous (rad) | $L_{3}$ | $L_{4}$ |
| $q_{4}$ | J6_pitch_wrist | continuous (rad) | $L_{5}$ | $L_{6}$ |
| $q_{5}$ | J9_horn_drive | continuous (rad) | $L_{6}$ | $L_{7}$ |
| $q_{6}$ | J10_horn_link1 | continuous (rad) | $L_{7}$ | $L_{8}$ |
| $q_{7}$ | J11_horn_link2 | continuous (rad) | $L_{7}$ | $L_{9}$ |
| $q_{8}$ | J12_link1_tooth1 | continuous (rad) | $L_{8}$ | $L_{10}$ |
| $q_{9}$ | J13_link2_tooth2 | continuous (rad) | $L_{9}$ | $L_{11}$ |

Shorthand: $c_i = \cos(q_i)$, $s_i = \sin(q_i)$

### Kinematic Tree

```
L0: base_link
  +-- [continuous] J1_yaw_base_shoulderLink (q1)
      L1: RG_shoulder_link
        +-- [continuous] J2_pitch_shoulder (q2)
            L2: RG_shoulder_arm
              +-- [fixed] J3_rigid_shoulderArm_elbowLink
                  L3: RG_elbow_link
                    +-- [continuous] J4_pitch_elbow (q3)
                        L4: RG_elbow_arm
                          +-- [fixed] J5_rigid_elbowArm_wristLink
                              L5: RG_wrist_link
                                +-- [continuous] J6_pitch_wrist (q4)
                                    L6: RG_wrist_arm
                                      +-- [continuous] J9_horn_drive (q5)
                                          L7: RG_gripper_horn
                                            |-- [continuous] J10_horn_link1 (q6)
                                            |   L8: marav3best_gripper_2nd_link
                                            |     +-- [continuous] J12_link1_tooth1 (q8)
                                            |         L10: marav3best_gripper
                                            +-- [continuous] J11_horn_link2 (q7)
                                                L9: marav3best_gripper_2nd_link_2
                                                  +-- [continuous] J13_link2_tooth2 (q9)
                                                      L11: marav3best_gripper_2
```

## Transforms

## J1_yaw_base_shoulderLink

$L_{0}$ **base_link** -> $L_{1}$ **RG_shoulder_link** (continuous)
  Variable: $q_{1}$

- **origin xyz**: (0, 0, 0.1615) m
- **origin rpy**: (-3.141593, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{0}_{1}(q_{1}) = T_{fixed} \cdot R_{axis}(q_{1})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & -1 & 0 & 0 \\
0 & 0 & -1 & 0.1615 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{1}) = \begin{bmatrix}
c_{1} & -s_{1} & 0 & 0 \\
s_{1} & c_{1} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## J2_pitch_shoulder

$L_{1}$ **RG_shoulder_link** -> $L_{2}$ **RG_shoulder_arm** (continuous)
  Variable: $q_{2}$

- **origin xyz**: (-0.104, 0, -0.0655) m
- **origin rpy**: (1.570796, 0, -1.570796) rad
- **axis**: (0, 0, -1)

### Local Transform

$T^{1}_{2}(q_{2}) = T_{fixed} \cdot R_{axis}(q_{2})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0 & -1 & -0.104 \\
-1 & 0 & 0 & 0 \\
0 & 1 & 0 & -0.0655 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{2}) = \begin{bmatrix}
c_{2} & s_{2} & 0 & 0 \\
-s_{2} & c_{2} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## J3_rigid_shoulderArm_elbowLink

$L_{2}$ **RG_shoulder_arm** -> $L_{3}$ **RG_elbow_link** (fixed)

- **origin xyz**: (-0.00187, -0.139988, -0.149) m
- **origin rpy**: (-3.141593, 0, -0.01336) rad

### Local Transform

$$
T^{2}_{3} = \begin{bmatrix}
0.999911 & -0.013359 & 0 & -0.00187 \\
-0.013359 & -0.999911 & 0 & -0.139988 \\
0 & 0 & -1 & -0.149 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## J4_pitch_elbow

$L_{3}$ **RG_elbow_link** -> $L_{4}$ **RG_elbow_arm** (continuous)
  Variable: $q_{3}$

- **origin xyz**: (0, 0, 0.055) m
- **origin rpy**: (0, 0, -0.01336) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{3}_{4}(q_{3}) = T_{fixed} \cdot R_{axis}(q_{3})$ where:

$$
T_{fixed} = \begin{bmatrix}
0.999911 & 0.013359 & 0 & 0 \\
-0.013359 & 0.999911 & 0 & 0 \\
0 & 0 & 1 & 0.055 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{3}) = \begin{bmatrix}
c_{3} & -s_{3} & 0 & 0 \\
s_{3} & c_{3} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## J5_rigid_elbowArm_wristLink

$L_{4}$ **RG_elbow_arm** -> $L_{5}$ **RG_wrist_link** (fixed)

- **origin xyz**: (-0.001437, 0.139993, -0.15) m
- **origin rpy**: (-3.141593, 0, -3.131329) rad

### Local Transform

$$
T^{4}_{5} = \begin{bmatrix}
-0.999947 & -0.010264 & 0 & -0.001437 \\
-0.010264 & 0.999947 & 0 & 0.139993 \\
0 & 0 & -1 & -0.15 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## J6_pitch_wrist

$L_{5}$ **RG_wrist_link** -> $L_{6}$ **RG_wrist_arm** (continuous)
  Variable: $q_{4}$

- **origin xyz**: (0, 0, 0.03) m
- **origin rpy**: (0, 0, -3.131329) rad
- **axis**: (0, 0, -1)

### Local Transform

$T^{5}_{6}(q_{4}) = T_{fixed} \cdot R_{axis}(q_{4})$ where:

$$
T_{fixed} = \begin{bmatrix}
-0.999947 & 0.010264 & 0 & 0 \\
-0.010264 & -0.999947 & 0 & 0 \\
0 & 0 & 1 & 0.03 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{4}) = \begin{bmatrix}
c_{4} & s_{4} & 0 & 0 \\
-s_{4} & c_{4} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## J9_horn_drive

$L_{6}$ **RG_wrist_arm** -> $L_{7}$ **RG_gripper_horn** (continuous)
  Variable: $q_{5}$

- **origin xyz**: (-0.002376, -0.118976, -0.082) m
- **origin rpy**: (-1.550832, 1.570796, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{6}_{7}(q_{5}) = T_{fixed} \cdot R_{axis}(q_{5})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -0.999801 & 0.019963 & -0.002376 \\
0 & 0.019963 & 0.999801 & -0.118976 \\
-1 & 0 & 0 & -0.082 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{5}) = \begin{bmatrix}
c_{5} & -s_{5} & 0 & 0 \\
s_{5} & c_{5} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## J10_horn_link1

$L_{7}$ **RG_gripper_horn** -> $L_{8}$ **marav3best_gripper_2nd_link** (continuous)
  Variable: $q_{6}$

- **origin xyz**: (0.025999, -0.000167, -0.0108) m
- **origin rpy**: (-3.141593, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{7}_{8}(q_{6}) = T_{fixed} \cdot R_{axis}(q_{6})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & 0.025999 \\
0 & -1 & 0 & -0.000167 \\
0 & 0 & -1 & -0.0108 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{6}) = \begin{bmatrix}
c_{6} & -s_{6} & 0 & 0 \\
s_{6} & c_{6} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## J11_horn_link2

$L_{7}$ **RG_gripper_horn** -> $L_{9}$ **marav3best_gripper_2nd_link_2** (continuous)
  Variable: $q_{7}$

- **origin xyz**: (-0.025999, 0.000167, -0.0108) m
- **origin rpy**: (-3.141593, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{7}_{9}(q_{7}) = T_{fixed} \cdot R_{axis}(q_{7})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & -0.025999 \\
0 & -1 & 0 & 0.000167 \\
0 & 0 & -1 & -0.0108 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{7}) = \begin{bmatrix}
c_{7} & -s_{7} & 0 & 0 \\
s_{7} & c_{7} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## J12_link1_tooth1

$L_{8}$ **marav3best_gripper_2nd_link** -> $L_{10}$ **marav3best_gripper** (continuous)
  Variable: $q_{8}$

- **origin xyz**: (0.025997, -0.000367, -0.00355) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$$
T^{8}_{10}(q_{8}) = \begin{bmatrix}
c_{8} & -s_{8} & 0 & 0.025997 \\
s_{8} & c_{8} & 0 & -0.000367 \\
0 & 0 & 1 & -0.00355 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## J13_link2_tooth2

$L_{9}$ **marav3best_gripper_2nd_link_2** -> $L_{11}$ **marav3best_gripper_2** (continuous)
  Variable: $q_{9}$

- **origin xyz**: (-0.025997, 0.000367, -0.00355) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$$
T^{9}_{11}(q_{9}) = \begin{bmatrix}
c_{9} & -s_{9} & 0 & -0.025997 \\
s_{9} & c_{9} & 0 & 0.000367 \\
0 & 0 & 1 & -0.00355 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Global Transform Chains

Transform from root $L_0$ to any link, as product of local transforms along the kinematic chain.

$$T^{0}_{2} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2})\quad (L_0 \to L_{2}: \text{RG_shoulder_arm})$$

$$T^{0}_{3} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}\quad (L_0 \to L_{3}: \text{RG_elbow_link})$$

$$T^{0}_{4} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3} \cdot T^{3}_{4}(q_{3})\quad (L_0 \to L_{4}: \text{RG_elbow_arm})$$

$$T^{0}_{5} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3} \cdot T^{3}_{4}(q_{3}) \cdot T^{4}_{5}\quad (L_0 \to L_{5}: \text{RG_wrist_link})$$

$$T^{0}_{6} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3} \cdot T^{3}_{4}(q_{3}) \cdot T^{4}_{5} \cdot T^{5}_{6}(q_{4})\quad (L_0 \to L_{6}: \text{RG_wrist_arm})$$

$$T^{0}_{7} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3} \cdot T^{3}_{4}(q_{3}) \cdot T^{4}_{5} \cdot T^{5}_{6}(q_{4}) \cdot T^{6}_{7}(q_{5})\quad (L_0 \to L_{7}: \text{RG_gripper_horn})$$

$$T^{0}_{8} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3} \cdot T^{3}_{4}(q_{3}) \cdot T^{4}_{5} \cdot T^{5}_{6}(q_{4}) \cdot T^{6}_{7}(q_{5}) \cdot T^{7}_{8}(q_{6})\quad (L_0 \to L_{8}: \text{marav3best_gripper_2nd_link})$$

$$T^{0}_{9} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3} \cdot T^{3}_{4}(q_{3}) \cdot T^{4}_{5} \cdot T^{5}_{6}(q_{4}) \cdot T^{6}_{7}(q_{5}) \cdot T^{7}_{9}(q_{7})\quad (L_0 \to L_{9}: \text{marav3best_gripper_2nd_link_2})$$

$$T^{0}_{10} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3} \cdot T^{3}_{4}(q_{3}) \cdot T^{4}_{5} \cdot T^{5}_{6}(q_{4}) \cdot T^{6}_{7}(q_{5}) \cdot T^{7}_{8}(q_{6}) \cdot T^{8}_{10}(q_{8})\quad (L_0 \to L_{10}: \text{marav3best_gripper})$$

$$T^{0}_{11} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3} \cdot T^{3}_{4}(q_{3}) \cdot T^{4}_{5} \cdot T^{5}_{6}(q_{4}) \cdot T^{6}_{7}(q_{5}) \cdot T^{7}_{9}(q_{7}) \cdot T^{9}_{11}(q_{9})\quad (L_0 \to L_{11}: \text{marav3best_gripper_2})$$

