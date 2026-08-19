# Transformation Matrices - marav1

Homogeneous transformation matrices between consecutive frames.
Convention: URDF RPY (XYZ extrinsic / ZYX intrinsic).

## Notation

### Frames

| Index | Link |
|-------|------|
| $L_{0}$ | base_link |
| $L_{1}$ | 2nd_link |
| $L_{2}$ | 1st_encoder |
| $L_{3}$ | 3rd_link |
| $L_{4}$ | 2nd_encoder |
| $L_{5}$ | 4th_link |
| $L_{6}$ | wrist_base |
| $L_{7}$ | 5th_link |
| $L_{8}$ | gripper_motor |
| $L_{9}$ | camera_holder |
| $L_{10}$ | gripper_tooth_1 |
| $L_{11}$ | gripper_tooth_2 |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | Revolute_1 | continuous (rad) | $L_{0}$ | $L_{1}$ |
| $q_{2}$ | Revolute_2 | continuous (rad) | $L_{1}$ | $L_{3}$ |
| $q_{3}$ | Revolute_3 | continuous (rad) | $L_{3}$ | $L_{5}$ |
| $q_{4}$ | Revolute_4 | continuous (rad) | $L_{5}$ | $L_{7}$ |
| $q_{5}$ | Slider_7 | prismatic (m) | $L_{9}$ | $L_{10}$ |
| $q_{6}$ | Slider_12 | prismatic (m) | $L_{9}$ | $L_{11}$ |

Shorthand: $c_i = \cos(q_i)$, $s_i = \sin(q_i)$

### Kinematic Tree

```
L0: base_link
  +-- [continuous] Revolute_1 (q1)
      L1: 2nd_link
        |-- [fixed] Rigid_10
        |   L2: 1st_encoder
        +-- [continuous] Revolute_2 (q2)
            L3: 3rd_link
              |-- [fixed] Rigid_14
              |   L4: 2nd_encoder
              +-- [continuous] Revolute_3 (q3)
                  L5: 4th_link
                    |-- [fixed] Rigid_13
                    |   L6: wrist_base
                    +-- [continuous] Revolute_4 (q4)
                        L7: 5th_link
                          |-- [fixed] Rigid_15
                          |   L8: gripper_motor
                          +-- [fixed] Rigid_5
                              L9: camera_holder
                                |-- [prismatic] Slider_7 (q5)
                                |   L10: gripper_tooth_1
                                +-- [prismatic] Slider_12 (q6)
                                    L11: gripper_tooth_2
```

## Transforms

## Revolute_1

$L_{0}$ **base_link** -> $L_{1}$ **2nd_link** (continuous)
  Variable: $q_{1}$

- **origin xyz**: (0, 0, 0.322) m
- **origin rpy**: (3.141593, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{0}_{1}(q_{1}) = T_{fixed} \cdot R_{axis}(q_{1})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & -1 & 0 & 0 \\
0 & 0 & -1 & 0.322 \\
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

## Rigid_10

$L_{1}$ **2nd_link** -> $L_{2}$ **1st_encoder** (fixed)

- **origin xyz**: (0, 0, 0.322) m
- **origin rpy**: (3.141593, 0, 0) rad

### Local Transform

$$
T^{1}_{2} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & -1 & 0 & 0 \\
0 & 0 & -1 & 0.322 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_2

$L_{1}$ **2nd_link** -> $L_{3}$ **3rd_link** (continuous)
  Variable: $q_{2}$

- **origin xyz**: (0, 0.005, -0.124) m
- **origin rpy**: (-1.570796, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{1}_{3}(q_{2}) = T_{fixed} \cdot R_{axis}(q_{2})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 0 & 1 & 0.005 \\
0 & -1 & 0 & -0.124 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{2}) = \begin{bmatrix}
c_{2} & -s_{2} & 0 & 0 \\
s_{2} & c_{2} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_14

$L_{3}$ **3rd_link** -> $L_{4}$ **2nd_encoder** (fixed)

- **origin xyz**: (0, -0.306, 0.095169) m
- **origin rpy**: (1.570796, 0, 3.141593) rad

### Local Transform

$$
T^{3}_{4} = \begin{bmatrix}
-1 & 0 & 0 & 0 \\
0 & 0 & 1 & -0.306 \\
0 & 1 & 0 & 0.095169 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_3

$L_{3}$ **3rd_link** -> $L_{5}$ **4th_link** (continuous)
  Variable: $q_{3}$

- **origin xyz**: (0, 0.28, -0.11) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$$
T^{3}_{5}(q_{3}) = \begin{bmatrix}
c_{3} & -s_{3} & 0 & 0 \\
s_{3} & c_{3} & 0 & 0.28 \\
0 & 0 & 1 & -0.11 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_13

$L_{5}$ **4th_link** -> $L_{6}$ **wrist_base** (fixed)

- **origin xyz**: (0, -0.446, 0.009831) m
- **origin rpy**: (-1.570796, 0, 0) rad

### Local Transform

$$
T^{5}_{6} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 0 & 1 & -0.446 \\
0 & -1 & 0 & 0.009831 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_4

$L_{5}$ **4th_link** -> $L_{7}$ **5th_link** (continuous)
  Variable: $q_{4}$

- **origin xyz**: (0, -0.123, 0.16) m
- **origin rpy**: (3.141593, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{5}_{7}(q_{4}) = T_{fixed} \cdot R_{axis}(q_{4})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & -1 & 0 & -0.123 \\
0 & 0 & -1 & 0.16 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{4}) = \begin{bmatrix}
c_{4} & -s_{4} & 0 & 0 \\
s_{4} & c_{4} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_15

$L_{7}$ **5th_link** -> $L_{8}$ **gripper_motor** (fixed)

- **origin xyz**: (0, 0.603, 0.055) m
- **origin rpy**: (1.570796, 0, 0) rad

### Local Transform

$$
T^{7}_{8} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 0 & -1 & 0.603 \\
0 & 1 & 0 & 0.055 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_5

$L_{7}$ **5th_link** -> $L_{9}$ **camera_holder** (fixed)

- **origin xyz**: (0, 0.603, 0.055) m
- **origin rpy**: (1.570796, 0, 0) rad

### Local Transform

$$
T^{7}_{9} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 0 & -1 & 0.603 \\
0 & 1 & 0 & 0.055 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Slider_7

$L_{9}$ **camera_holder** -> $L_{10}$ **gripper_tooth_1** (prismatic)
  Variable: $q_{5}$

- **origin xyz**: (0, 0.055, 0.628) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, -1, 0)
- **limits**: [0, 0.04] m

### Local Transform

$$
T^{9}_{10}(q_{5}) = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0.055 - q_{5} \\
0 & 0 & 1 & 0.628 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Slider_12

$L_{9}$ **camera_holder** -> $L_{11}$ **gripper_tooth_2** (prismatic)
  Variable: $q_{6}$

- **origin xyz**: (0, -0.15, 0.628) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, 1, 0)
- **limits**: [0, 0.04] m

### Local Transform

$$
T^{9}_{11}(q_{6}) = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & -0.15 + q_{6} \\
0 & 0 & 1 & 0.628 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Global Transform Chains

Transform from root $L_0$ to any link, as product of local transforms along the kinematic chain.

$$T^{0}_{2} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}\quad (L_0 \to L_{2}: \text{1st_encoder})$$

$$T^{0}_{3} = T^{0}_{1}(q_{1}) \cdot T^{1}_{3}(q_{2})\quad (L_0 \to L_{3}: \text{3rd_link})$$

$$T^{0}_{4} = T^{0}_{1}(q_{1}) \cdot T^{1}_{3}(q_{2}) \cdot T^{3}_{4}\quad (L_0 \to L_{4}: \text{2nd_encoder})$$

$$T^{0}_{5} = T^{0}_{1}(q_{1}) \cdot T^{1}_{3}(q_{2}) \cdot T^{3}_{5}(q_{3})\quad (L_0 \to L_{5}: \text{4th_link})$$

$$T^{0}_{6} = T^{0}_{1}(q_{1}) \cdot T^{1}_{3}(q_{2}) \cdot T^{3}_{5}(q_{3}) \cdot T^{5}_{6}\quad (L_0 \to L_{6}: \text{wrist_base})$$

$$T^{0}_{7} = T^{0}_{1}(q_{1}) \cdot T^{1}_{3}(q_{2}) \cdot T^{3}_{5}(q_{3}) \cdot T^{5}_{7}(q_{4})\quad (L_0 \to L_{7}: \text{5th_link})$$

$$T^{0}_{8} = T^{0}_{1}(q_{1}) \cdot T^{1}_{3}(q_{2}) \cdot T^{3}_{5}(q_{3}) \cdot T^{5}_{7}(q_{4}) \cdot T^{7}_{8}\quad (L_0 \to L_{8}: \text{gripper_motor})$$

$$T^{0}_{9} = T^{0}_{1}(q_{1}) \cdot T^{1}_{3}(q_{2}) \cdot T^{3}_{5}(q_{3}) \cdot T^{5}_{7}(q_{4}) \cdot T^{7}_{9}\quad (L_0 \to L_{9}: \text{camera_holder})$$

$$T^{0}_{10} = T^{0}_{1}(q_{1}) \cdot T^{1}_{3}(q_{2}) \cdot T^{3}_{5}(q_{3}) \cdot T^{5}_{7}(q_{4}) \cdot T^{7}_{9} \cdot T^{9}_{10}(q_{5})\quad (L_0 \to L_{10}: \text{gripper_tooth_1})$$

$$T^{0}_{11} = T^{0}_{1}(q_{1}) \cdot T^{1}_{3}(q_{2}) \cdot T^{3}_{5}(q_{3}) \cdot T^{5}_{7}(q_{4}) \cdot T^{7}_{9} \cdot T^{9}_{11}(q_{6})\quad (L_0 \to L_{11}: \text{gripper_tooth_2})$$

