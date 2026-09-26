# Transformation Matrices - marav2ckptbest

Homogeneous transformation matrices between consecutive frames.
Convention: URDF RPY (XYZ extrinsic / ZYX intrinsic).

## Notation

### Frames

| Index | Link |
|-------|------|
| $L_{0}$ | base_link |
| $L_{1}$ | shoulder_link |
| $L_{2}$ | shoulder_arm |
| $L_{3}$ | elbow_link |
| $L_{4}$ | elbow_arm |
| $L_{5}$ | wrist_link |
| $L_{6}$ | wrist_arm |
| $L_{7}$ | gripper_teeth |
| $L_{8}$ | gripper_teeth2 |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | Revolute_1 | continuous (rad) | $L_{0}$ | $L_{1}$ |
| $q_{2}$ | Revolute_5 | continuous (rad) | $L_{1}$ | $L_{2}$ |
| $q_{3}$ | Revolute_7 | continuous (rad) | $L_{3}$ | $L_{4}$ |
| $q_{4}$ | Revolute_9 | continuous (rad) | $L_{5}$ | $L_{6}$ |
| $q_{5}$ | Slider_10 | prismatic (m) | $L_{6}$ | $L_{7}$ |
| $q_{6}$ | Slider_11 | prismatic (m) | $L_{6}$ | $L_{8}$ |

Shorthand: $c_i = \cos(q_i)$, $s_i = \sin(q_i)$

### Kinematic Tree

```
L0: base_link
  +-- [continuous] Revolute_1 (q1)
      L1: shoulder_link
        +-- [continuous] Revolute_5 (q2)
            L2: shoulder_arm
              +-- [fixed] Rigid_6
                  L3: elbow_link
                    +-- [continuous] Revolute_7 (q3)
                        L4: elbow_arm
                          +-- [fixed] Rigid_8
                              L5: wrist_link
                                +-- [continuous] Revolute_9 (q4)
                                    L6: wrist_arm
                                      |-- [prismatic] Slider_10 (q5)
                                      |   L7: gripper_teeth
                                      +-- [prismatic] Slider_11 (q6)
                                          L8: gripper_teeth2
```

## Transforms

## Revolute_1

$L_{0}$ **base_link** -> $L_{1}$ **shoulder_link** (continuous)
  Variable: $q_{1}$

- **origin xyz**: (0, 0, 0.166) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$$
T^{0}_{1}(q_{1}) = \begin{bmatrix}
c_{1} & -s_{1} & 0 & 0 \\
s_{1} & c_{1} & 0 & 0 \\
0 & 0 & 1 & 0.166 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_5

$L_{1}$ **shoulder_link** -> $L_{2}$ **shoulder_arm** (continuous)
  Variable: $q_{2}$

- **origin xyz**: (0.05, 0, 0.06) m
- **origin rpy**: (1.570796, 0, 1.570796) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{1}_{2}(q_{2}) = T_{fixed} \cdot R_{axis}(q_{2})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0 & 1 & 0.05 \\
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0.06 \\
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

## Rigid_6

$L_{2}$ **shoulder_arm** -> $L_{3}$ **elbow_link** (fixed)

- **origin xyz**: (0, -0.226, -0.05) m
- **origin rpy**: (-1.570796, -1.570796, 0) rad

### Local Transform

$$
T^{2}_{3} = \begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & 0 & 1 & -0.226 \\
1 & 0 & 0 & -0.05 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_7

$L_{3}$ **elbow_link** -> $L_{4}$ **elbow_arm** (continuous)
  Variable: $q_{3}$

- **origin xyz**: (-0.045, 0, 0.366) m
- **origin rpy**: (-1.570796, 0, 1.570796) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{3}_{4}(q_{3}) = T_{fixed} \cdot R_{axis}(q_{3})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0 & -1 & -0.045 \\
1 & 0 & 0 & 0 \\
0 & -1 & 0 & 0.366 \\
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

## Rigid_8

$L_{4}$ **elbow_arm** -> $L_{5}$ **wrist_link** (fixed)

- **origin xyz**: (0, 0.366, -0.045) m
- **origin rpy**: (1.570796, 1.570796, 0) rad

### Local Transform

$$
T^{4}_{5} = \begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & 0 & -1 & 0.366 \\
-1 & 0 & 0 & -0.045 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_9

$L_{5}$ **wrist_link** -> $L_{6}$ **wrist_arm** (continuous)
  Variable: $q_{4}$

- **origin xyz**: (0.05, 0, 0.506) m
- **origin rpy**: (1.570796, 0, 1.570796) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{5}_{6}(q_{4}) = T_{fixed} \cdot R_{axis}(q_{4})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0 & 1 & 0.05 \\
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0.506 \\
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

## Slider_10

$L_{6}$ **wrist_arm** -> $L_{7}$ **gripper_teeth** (prismatic)
  Variable: $q_{5}$

- **origin xyz**: (0, 0.125, -0.09) m
- **origin rpy**: (-1.570796, -1.570796, 0) rad
- **axis**: (1, 0, 0)
- **limits**: [0, 0.04] m

### Local Transform

$$
T^{6}_{7}(q_{5}) = \begin{bmatrix}
0 & 1 & 0 & q_{5} \\
0 & 0 & 1 & 0.125 \\
1 & 0 & 0 & -0.09 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Slider_11

$L_{6}$ **wrist_arm** -> $L_{8}$ **gripper_teeth2** (prismatic)
  Variable: $q_{6}$

- **origin xyz**: (0, 0.125, 0) m
- **origin rpy**: (-1.570796, -1.570796, 0) rad
- **axis**: (1, 0, 0)
- **limits**: [-0.04, 0] m

### Local Transform

$$
T^{6}_{8}(q_{6}) = \begin{bmatrix}
0 & 1 & 0 & q_{6} \\
0 & 0 & 1 & 0.125 \\
1 & 0 & 0 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Global Transform Chains

Transform from root $L_0$ to any link, as product of local transforms along the kinematic chain.

$$T^{0}_{2} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2})\quad (L_0 \to L_{2}: \text{shoulder_arm})$$

$$T^{0}_{3} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}\quad (L_0 \to L_{3}: \text{elbow_link})$$

$$T^{0}_{4} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3} \cdot T^{3}_{4}(q_{3})\quad (L_0 \to L_{4}: \text{elbow_arm})$$

$$T^{0}_{5} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3} \cdot T^{3}_{4}(q_{3}) \cdot T^{4}_{5}\quad (L_0 \to L_{5}: \text{wrist_link})$$

$$T^{0}_{6} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3} \cdot T^{3}_{4}(q_{3}) \cdot T^{4}_{5} \cdot T^{5}_{6}(q_{4})\quad (L_0 \to L_{6}: \text{wrist_arm})$$

$$T^{0}_{7} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3} \cdot T^{3}_{4}(q_{3}) \cdot T^{4}_{5} \cdot T^{5}_{6}(q_{4}) \cdot T^{6}_{7}(q_{5})\quad (L_0 \to L_{7}: \text{gripper_teeth})$$

$$T^{0}_{8} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3} \cdot T^{3}_{4}(q_{3}) \cdot T^{4}_{5} \cdot T^{5}_{6}(q_{4}) \cdot T^{6}_{8}(q_{6})\quad (L_0 \to L_{8}: \text{gripper_teeth2})$$

