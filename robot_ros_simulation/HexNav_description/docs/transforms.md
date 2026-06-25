# Transformation Matrices - HexNav

Homogeneous transformation matrices between consecutive frames.
Convention: URDF RPY (XYZ extrinsic / ZYX intrinsic).

## Notation

### Frames

| Index | Link |
|-------|------|
| $L_{0}$ | base_link |
| $L_{1}$ | leg_1_mx28_coxa |
| $L_{2}$ | leg_2_mx28_coxa |
| $L_{3}$ | leg_3_mx28_coxa |
| $L_{4}$ | leg_4_mx28_coxa |
| $L_{5}$ | leg_5_mx28_coxa |
| $L_{6}$ | leg_6_mx28_coxa |
| $L_{7}$ | leg_1_coxa |
| $L_{8}$ | leg_2_coxa |
| $L_{9}$ | leg_3_coxa |
| $L_{10}$ | leg_4_coxa |
| $L_{11}$ | leg_5_coxa |
| $L_{12}$ | bot |
| $L_{13}$ | leg_6_coxa |
| $L_{14}$ | leg_1_mx28_femur |
| $L_{15}$ | leg_2_mx28_femur |
| $L_{16}$ | leg_3_mx28_femur |
| $L_{17}$ | leg_4_mx28_femur |
| $L_{18}$ | leg_5_mx28_femur |
| $L_{19}$ | leg_6_mx28_femur |
| $L_{20}$ | leg_1_femur |
| $L_{21}$ | leg_2_femur |
| $L_{22}$ | leg_3_femur |
| $L_{23}$ | leg_4_femur |
| $L_{24}$ | leg_5_femur |
| $L_{25}$ | leg_6_femur |
| $L_{26}$ | leg_1_mx28_tibia |
| $L_{27}$ | leg_2_mx28_tibia |
| $L_{28}$ | leg_3_mx28_tibia |
| $L_{29}$ | leg_4_mx28_tibia |
| $L_{30}$ | leg_5_mx28_tibia |
| $L_{31}$ | leg_6_mx28_tibia |
| $L_{32}$ | leg_1_tibia |
| $L_{33}$ | leg_2_tibia |
| $L_{34}$ | leg_3_tibia |
| $L_{35}$ | leg_4_tibia |
| $L_{36}$ | leg_5_tibia |
| $L_{37}$ | leg_6_tibia |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | leg_1_coxa_joint | continuous (rad) | $L_{1}$ | $L_{7}$ |
| $q_{2}$ | leg_2_coxa_joint | continuous (rad) | $L_{2}$ | $L_{8}$ |
| $q_{3}$ | leg_3_coxa_joint | continuous (rad) | $L_{3}$ | $L_{9}$ |
| $q_{4}$ | leg_4_coxa_joint | continuous (rad) | $L_{4}$ | $L_{10}$ |
| $q_{5}$ | leg_5_coxa_joint | continuous (rad) | $L_{5}$ | $L_{11}$ |
| $q_{6}$ | leg_6_coxa_joint | continuous (rad) | $L_{6}$ | $L_{13}$ |
| $q_{7}$ | leg_1_femur_joint | continuous (rad) | $L_{7}$ | $L_{14}$ |
| $q_{8}$ | leg_2_femur_joint | continuous (rad) | $L_{8}$ | $L_{15}$ |
| $q_{9}$ | leg_3_femur_joint | continuous (rad) | $L_{9}$ | $L_{16}$ |
| $q_{10}$ | leg_4_femur_joint | continuous (rad) | $L_{10}$ | $L_{17}$ |
| $q_{11}$ | leg_5_femur_joint | continuous (rad) | $L_{11}$ | $L_{18}$ |
| $q_{12}$ | leg_6_femur_joint | continuous (rad) | $L_{13}$ | $L_{19}$ |
| $q_{13}$ | leg_1_tibia_joint | continuous (rad) | $L_{20}$ | $L_{26}$ |
| $q_{14}$ | leg_2_tibia_joint | continuous (rad) | $L_{21}$ | $L_{27}$ |
| $q_{15}$ | leg_3_tibia_joint | continuous (rad) | $L_{22}$ | $L_{28}$ |
| $q_{16}$ | leg_4_tibia_joint | continuous (rad) | $L_{23}$ | $L_{29}$ |
| $q_{17}$ | leg_5_tibia_joint | continuous (rad) | $L_{24}$ | $L_{30}$ |
| $q_{18}$ | leg_6_tibia_joint | continuous (rad) | $L_{25}$ | $L_{31}$ |

Shorthand: $c_i = \cos(q_i)$, $s_i = \sin(q_i)$

### Kinematic Tree

```
L0: base_link
  |-- [fixed] leg_1_coxa_mount
  |   L1: leg_1_mx28_coxa
  |     +-- [continuous] leg_1_coxa_joint (q1)
  |         L7: leg_1_coxa
  |           +-- [continuous] leg_1_femur_joint (q7)
  |               L14: leg_1_mx28_femur
  |                 +-- [fixed] leg_1_femur_mount
  |                     L20: leg_1_femur
  |                       +-- [continuous] leg_1_tibia_joint (q13)
  |                           L26: leg_1_mx28_tibia
  |                             +-- [fixed] leg_1_tibia_mount
  |                                 L32: leg_1_tibia
  |-- [fixed] leg_2_coxa_mount
  |   L2: leg_2_mx28_coxa
  |     +-- [continuous] leg_2_coxa_joint (q2)
  |         L8: leg_2_coxa
  |           +-- [continuous] leg_2_femur_joint (q8)
  |               L15: leg_2_mx28_femur
  |                 +-- [fixed] leg_2_femur_mount
  |                     L21: leg_2_femur
  |                       +-- [continuous] leg_2_tibia_joint (q14)
  |                           L27: leg_2_mx28_tibia
  |                             +-- [fixed] leg_2_tibia_mount
  |                                 L33: leg_2_tibia
  |-- [fixed] leg_3_coxa_mount
  |   L3: leg_3_mx28_coxa
  |     +-- [continuous] leg_3_coxa_joint (q3)
  |         L9: leg_3_coxa
  |           +-- [continuous] leg_3_femur_joint (q9)
  |               L16: leg_3_mx28_femur
  |                 +-- [fixed] leg_3_femur_mount
  |                     L22: leg_3_femur
  |                       +-- [continuous] leg_3_tibia_joint (q15)
  |                           L28: leg_3_mx28_tibia
  |                             +-- [fixed] leg_3_tibia_mount
  |                                 L34: leg_3_tibia
  |-- [fixed] leg_4_coxa_mount
  |   L4: leg_4_mx28_coxa
  |     +-- [continuous] leg_4_coxa_joint (q4)
  |         L10: leg_4_coxa
  |           +-- [continuous] leg_4_femur_joint (q10)
  |               L17: leg_4_mx28_femur
  |                 +-- [fixed] leg_4_femur_mount
  |                     L23: leg_4_femur
  |                       +-- [continuous] leg_4_tibia_joint (q16)
  |                           L29: leg_4_mx28_tibia
  |                             +-- [fixed] leg_4_tibia_mount
  |                                 L35: leg_4_tibia
  |-- [fixed] leg_5_coxa_mount
  |   L5: leg_5_mx28_coxa
  |     |-- [continuous] leg_5_coxa_joint (q5)
  |     |   L11: leg_5_coxa
  |     |     +-- [continuous] leg_5_femur_joint (q11)
  |     |         L18: leg_5_mx28_femur
  |     |           +-- [fixed] leg_5_femur_mount
  |     |               L24: leg_5_femur
  |     |                 +-- [continuous] leg_5_tibia_joint (q17)
  |     |                     L30: leg_5_mx28_tibia
  |     |                       +-- [fixed] leg_5_tibia_mount
  |     |                           L36: leg_5_tibia
  |     +-- [fixed] bot_mount
  |         L12: bot
  +-- [fixed] leg_6_coxa_mount
      L6: leg_6_mx28_coxa
        +-- [continuous] leg_6_coxa_joint (q6)
            L13: leg_6_coxa
              +-- [continuous] leg_6_femur_joint (q12)
                  L19: leg_6_mx28_femur
                    +-- [fixed] leg_6_femur_mount
                        L25: leg_6_femur
                          +-- [continuous] leg_6_tibia_joint (q18)
                              L31: leg_6_mx28_tibia
                                +-- [fixed] leg_6_tibia_mount
                                    L37: leg_6_tibia
```

## Transforms

## leg_1_coxa_mount

$L_{0}$ **base_link** -> $L_{1}$ **leg_1_mx28_coxa** (fixed)

- **origin xyz**: (0.010886, 0.010886, -0.0165) m
- **origin rpy**: (0, 0, 2.356194) rad

### Local Transform

$$
T^{0}_{1} = \begin{bmatrix}
-0.707107 & -0.707107 & 0 & 0.010886 \\
0.707107 & -0.707107 & 0 & 0.010886 \\
0 & 0 & 1 & -0.0165 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_2_coxa_mount

$L_{0}$ **base_link** -> $L_{2}$ **leg_2_mx28_coxa** (fixed)

- **origin xyz**: (0.14204, 0.010886, -0.0165) m
- **origin rpy**: (0, 0, -2.356194) rad

### Local Transform

$$
T^{0}_{2} = \begin{bmatrix}
-0.707107 & 0.707107 & 0 & 0.14204 \\
-0.707107 & -0.707107 & 0 & 0.010886 \\
0 & 0 & 1 & -0.0165 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_3_coxa_mount

$L_{0}$ **base_link** -> $L_{3}$ **leg_3_mx28_coxa** (fixed)

- **origin xyz**: (0.146243, 0.126463, -0.0165) m
- **origin rpy**: (0, 0, -1.570796) rad

### Local Transform

$$
T^{0}_{3} = \begin{bmatrix}
0 & 1 & 0 & 0.146243 \\
-1 & 0 & 0 & 0.126463 \\
0 & 0 & 1 & -0.0165 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_4_coxa_mount

$L_{0}$ **base_link** -> $L_{4}$ **leg_4_mx28_coxa** (fixed)

- **origin xyz**: (0.14204, 0.24204, -0.0165) m
- **origin rpy**: (0, 0, -0.785398) rad

### Local Transform

$$
T^{0}_{4} = \begin{bmatrix}
0.707107 & 0.707107 & 0 & 0.14204 \\
-0.707107 & 0.707107 & 0 & 0.24204 \\
0 & 0 & 1 & -0.0165 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_5_coxa_mount

$L_{0}$ **base_link** -> $L_{5}$ **leg_5_mx28_coxa** (fixed)

- **origin xyz**: (0.010886, 0.24204, -0.0165) m
- **origin rpy**: (0, 0, 0.785398) rad

### Local Transform

$$
T^{0}_{5} = \begin{bmatrix}
0.707107 & -0.707107 & 0 & 0.010886 \\
0.707107 & 0.707107 & 0 & 0.24204 \\
0 & 0 & 1 & -0.0165 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_6_coxa_mount

$L_{0}$ **base_link** -> $L_{6}$ **leg_6_mx28_coxa** (fixed)

- **origin xyz**: (0.006683, 0.126463, -0.0165) m
- **origin rpy**: (0, 0, 1.570796) rad

### Local Transform

$$
T^{0}_{6} = \begin{bmatrix}
0 & -1 & 0 & 0.006683 \\
1 & 0 & 0 & 0.126463 \\
0 & 0 & 1 & -0.0165 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_1_coxa_joint

$L_{1}$ **leg_1_mx28_coxa** -> $L_{7}$ **leg_1_coxa** (continuous)
  Variable: $q_{1}$

- **origin xyz**: (0, 0.052, 0) m
- **origin rpy**: (0, 0, 1.570796) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{1}_{7}(q_{1}) = T_{fixed} \cdot R_{axis}(q_{1})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -1 & 0 & 0 \\
1 & 0 & 0 & 0.052 \\
0 & 0 & 1 & 0 \\
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

## leg_2_coxa_joint

$L_{2}$ **leg_2_mx28_coxa** -> $L_{8}$ **leg_2_coxa** (continuous)
  Variable: $q_{2}$

- **origin xyz**: (0, 0.052, 0) m
- **origin rpy**: (0, 0, 1.570796) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{2}_{8}(q_{2}) = T_{fixed} \cdot R_{axis}(q_{2})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -1 & 0 & 0 \\
1 & 0 & 0 & 0.052 \\
0 & 0 & 1 & 0 \\
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

## leg_3_coxa_joint

$L_{3}$ **leg_3_mx28_coxa** -> $L_{9}$ **leg_3_coxa** (continuous)
  Variable: $q_{3}$

- **origin xyz**: (0, 0.052, 0) m
- **origin rpy**: (0, 0, 1.570796) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{3}_{9}(q_{3}) = T_{fixed} \cdot R_{axis}(q_{3})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -1 & 0 & 0 \\
1 & 0 & 0 & 0.052 \\
0 & 0 & 1 & 0 \\
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

## leg_4_coxa_joint

$L_{4}$ **leg_4_mx28_coxa** -> $L_{10}$ **leg_4_coxa** (continuous)
  Variable: $q_{4}$

- **origin xyz**: (0, 0.052, 0) m
- **origin rpy**: (0, 0, 1.570796) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{4}_{10}(q_{4}) = T_{fixed} \cdot R_{axis}(q_{4})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -1 & 0 & 0 \\
1 & 0 & 0 & 0.052 \\
0 & 0 & 1 & 0 \\
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

## leg_5_coxa_joint

$L_{5}$ **leg_5_mx28_coxa** -> $L_{11}$ **leg_5_coxa** (continuous)
  Variable: $q_{5}$

- **origin xyz**: (0, 0.052, 0) m
- **origin rpy**: (0, 0, 1.570796) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{5}_{11}(q_{5}) = T_{fixed} \cdot R_{axis}(q_{5})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -1 & 0 & 0 \\
1 & 0 & 0 & 0.052 \\
0 & 0 & 1 & 0 \\
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

## bot_mount

$L_{5}$ **leg_5_mx28_coxa** -> $L_{12}$ **bot** (fixed)

- **origin xyz**: (-0.070711, -0.271586, -0.015) m
- **origin rpy**: (3.141593, 0, 2.356194) rad

### Local Transform

$$
T^{5}_{12} = \begin{bmatrix}
-0.707107 & 0.707107 & 0 & -0.070711 \\
0.707107 & 0.707107 & 0 & -0.271586 \\
0 & 0 & -1 & -0.015 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_6_coxa_joint

$L_{6}$ **leg_6_mx28_coxa** -> $L_{13}$ **leg_6_coxa** (continuous)
  Variable: $q_{6}$

- **origin xyz**: (0, 0.052, 0) m
- **origin rpy**: (0, 0, 1.570796) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{6}_{13}(q_{6}) = T_{fixed} \cdot R_{axis}(q_{6})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -1 & 0 & 0 \\
1 & 0 & 0 & 0.052 \\
0 & 0 & 1 & 0 \\
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

## leg_1_femur_joint

$L_{7}$ **leg_1_coxa** -> $L_{14}$ **leg_1_mx28_femur** (continuous)
  Variable: $q_{7}$

- **origin xyz**: (0, 0, 0) m
- **origin rpy**: (-3.134106, -1.570796, -2.817751) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{7}_{14}(q_{7}) = T_{fixed} \cdot R_{axis}(q_{7})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -0.325299 & -0.945611 & 0 \\
0 & 0.945611 & -0.325299 & 0 \\
1 & 0 & 0 & 0 \\
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

## leg_2_femur_joint

$L_{8}$ **leg_2_coxa** -> $L_{15}$ **leg_2_mx28_femur** (continuous)
  Variable: $q_{8}$

- **origin xyz**: (0, 0, 0) m
- **origin rpy**: (2.562288, -1.570796, 2.333648) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{8}_{15}(q_{8}) = T_{fixed} \cdot R_{axis}(q_{8})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0.983202 & -0.182518 & 0 \\
0 & 0.182518 & 0.983202 & 0 \\
1 & 0 & 0 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{8}) = \begin{bmatrix}
c_{8} & -s_{8} & 0 & 0 \\
s_{8} & c_{8} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_3_femur_joint

$L_{9}$ **leg_3_coxa** -> $L_{16}$ **leg_3_mx28_femur** (continuous)
  Variable: $q_{9}$

- **origin xyz**: (0, 0, 0) m
- **origin rpy**: (2.58963, -1.570796, 2.312708) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{9}_{16}(q_{9}) = T_{fixed} \cdot R_{axis}(q_{9})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0.982014 & -0.188809 & 0 \\
0 & 0.188809 & 0.982014 & 0 \\
1 & 0 & 0 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{9}) = \begin{bmatrix}
c_{9} & -s_{9} & 0 & 0 \\
s_{9} & c_{9} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_4_femur_joint

$L_{10}$ **leg_4_coxa** -> $L_{17}$ **leg_4_mx28_femur** (continuous)
  Variable: $q_{10}$

- **origin xyz**: (0, 0, 0) m
- **origin rpy**: (2.510748, -1.570796, 2.399678) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{10}_{17}(q_{10}) = T_{fixed} \cdot R_{axis}(q_{10})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0.980455 & -0.196745 & 0 \\
0 & 0.196745 & 0.980455 & 0 \\
1 & 0 & 0 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{10}) = \begin{bmatrix}
c_{10} & -s_{10} & 0 & 0 \\
s_{10} & c_{10} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_5_femur_joint

$L_{11}$ **leg_5_coxa** -> $L_{18}$ **leg_5_mx28_femur** (continuous)
  Variable: $q_{11}$

- **origin xyz**: (0, 0, 0) m
- **origin rpy**: (2.811929, -1.570796, 2.142273) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{11}_{18}(q_{11}) = T_{fixed} \cdot R_{axis}(q_{11})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0.970905 & -0.239463 & 0 \\
0 & 0.239463 & 0.970905 & 0 \\
1 & 0 & 0 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{11}) = \begin{bmatrix}
c_{11} & -s_{11} & 0 & 0 \\
s_{11} & c_{11} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_6_femur_joint

$L_{13}$ **leg_6_coxa** -> $L_{19}$ **leg_6_mx28_femur** (continuous)
  Variable: $q_{12}$

- **origin xyz**: (0, 0, 0) m
- **origin rpy**: (2.312681, -1.570796, 2.559007) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{13}_{19}(q_{12}) = T_{fixed} \cdot R_{axis}(q_{12})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0.987339 & -0.158626 & 0 \\
0 & 0.158626 & 0.987339 & 0 \\
1 & 0 & 0 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{12}) = \begin{bmatrix}
c_{12} & -s_{12} & 0 & 0 \\
s_{12} & c_{12} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_1_femur_mount

$L_{14}$ **leg_1_mx28_femur** -> $L_{20}$ **leg_1_femur** (fixed)

- **origin xyz**: (0, -0.0358, 0) m
- **origin rpy**: (0, 0, -3.141593) rad

### Local Transform

$$
T^{14}_{20} = \begin{bmatrix}
-1 & 0 & 0 & 0 \\
0 & -1 & 0 & -0.0358 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_2_femur_mount

$L_{15}$ **leg_2_mx28_femur** -> $L_{21}$ **leg_2_femur** (fixed)

- **origin xyz**: (0, -0.0358, 0) m
- **origin rpy**: (0, 0, -3.141593) rad

### Local Transform

$$
T^{15}_{21} = \begin{bmatrix}
-1 & 0 & 0 & 0 \\
0 & -1 & 0 & -0.0358 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_3_femur_mount

$L_{16}$ **leg_3_mx28_femur** -> $L_{22}$ **leg_3_femur** (fixed)

- **origin xyz**: (0, -0.0358, 0) m
- **origin rpy**: (0, 0, -3.141593) rad

### Local Transform

$$
T^{16}_{22} = \begin{bmatrix}
-1 & 0 & 0 & 0 \\
0 & -1 & 0 & -0.0358 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_4_femur_mount

$L_{17}$ **leg_4_mx28_femur** -> $L_{23}$ **leg_4_femur** (fixed)

- **origin xyz**: (0, -0.0358, 0) m
- **origin rpy**: (0, 0, -3.141593) rad

### Local Transform

$$
T^{17}_{23} = \begin{bmatrix}
-1 & 0 & 0 & 0 \\
0 & -1 & 0 & -0.0358 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_5_femur_mount

$L_{18}$ **leg_5_mx28_femur** -> $L_{24}$ **leg_5_femur** (fixed)

- **origin xyz**: (0, -0.0358, 0) m
- **origin rpy**: (0, 0, -3.141593) rad

### Local Transform

$$
T^{18}_{24} = \begin{bmatrix}
-1 & 0 & 0 & 0 \\
0 & -1 & 0 & -0.0358 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_6_femur_mount

$L_{19}$ **leg_6_mx28_femur** -> $L_{25}$ **leg_6_femur** (fixed)

- **origin xyz**: (0, -0.0358, 0) m
- **origin rpy**: (0, 0, -3.141593) rad

### Local Transform

$$
T^{19}_{25} = \begin{bmatrix}
-1 & 0 & 0 & 0 \\
0 & -1 & 0 & -0.0358 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_1_tibia_joint

$L_{20}$ **leg_1_femur** -> $L_{26}$ **leg_1_mx28_tibia** (continuous)
  Variable: $q_{13}$

- **origin xyz**: (0.015, 0.029, 0) m
- **origin rpy**: (0, 0, 2.443461) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{20}_{26}(q_{13}) = T_{fixed} \cdot R_{axis}(q_{13})$ where:

$$
T_{fixed} = \begin{bmatrix}
-0.766044 & -0.642788 & 0 & 0.015 \\
0.642788 & -0.766044 & 0 & 0.029 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{13}) = \begin{bmatrix}
c_{13} & -s_{13} & 0 & 0 \\
s_{13} & c_{13} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_2_tibia_joint

$L_{21}$ **leg_2_femur** -> $L_{27}$ **leg_2_mx28_tibia** (continuous)
  Variable: $q_{14}$

- **origin xyz**: (0.015, 0.029, 0) m
- **origin rpy**: (0, 0, 2.443461) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{21}_{27}(q_{14}) = T_{fixed} \cdot R_{axis}(q_{14})$ where:

$$
T_{fixed} = \begin{bmatrix}
-0.766044 & -0.642788 & 0 & 0.015 \\
0.642788 & -0.766044 & 0 & 0.029 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{14}) = \begin{bmatrix}
c_{14} & -s_{14} & 0 & 0 \\
s_{14} & c_{14} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_3_tibia_joint

$L_{22}$ **leg_3_femur** -> $L_{28}$ **leg_3_mx28_tibia** (continuous)
  Variable: $q_{15}$

- **origin xyz**: (0.015, 0.029, 0) m
- **origin rpy**: (0, 0, 2.443461) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{22}_{28}(q_{15}) = T_{fixed} \cdot R_{axis}(q_{15})$ where:

$$
T_{fixed} = \begin{bmatrix}
-0.766044 & -0.642788 & 0 & 0.015 \\
0.642788 & -0.766044 & 0 & 0.029 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{15}) = \begin{bmatrix}
c_{15} & -s_{15} & 0 & 0 \\
s_{15} & c_{15} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_4_tibia_joint

$L_{23}$ **leg_4_femur** -> $L_{29}$ **leg_4_mx28_tibia** (continuous)
  Variable: $q_{16}$

- **origin xyz**: (0.015, 0.029, 0) m
- **origin rpy**: (0, 0, 2.443461) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{23}_{29}(q_{16}) = T_{fixed} \cdot R_{axis}(q_{16})$ where:

$$
T_{fixed} = \begin{bmatrix}
-0.766044 & -0.642788 & 0 & 0.015 \\
0.642788 & -0.766044 & 0 & 0.029 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{16}) = \begin{bmatrix}
c_{16} & -s_{16} & 0 & 0 \\
s_{16} & c_{16} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_5_tibia_joint

$L_{24}$ **leg_5_femur** -> $L_{30}$ **leg_5_mx28_tibia** (continuous)
  Variable: $q_{17}$

- **origin xyz**: (0.015, 0.029, 0) m
- **origin rpy**: (0, 0, 2.443461) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{24}_{30}(q_{17}) = T_{fixed} \cdot R_{axis}(q_{17})$ where:

$$
T_{fixed} = \begin{bmatrix}
-0.766044 & -0.642788 & 0 & 0.015 \\
0.642788 & -0.766044 & 0 & 0.029 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{17}) = \begin{bmatrix}
c_{17} & -s_{17} & 0 & 0 \\
s_{17} & c_{17} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_6_tibia_joint

$L_{25}$ **leg_6_femur** -> $L_{31}$ **leg_6_mx28_tibia** (continuous)
  Variable: $q_{18}$

- **origin xyz**: (0.015, 0.029, 0) m
- **origin rpy**: (0, 0, 2.443461) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{25}_{31}(q_{18}) = T_{fixed} \cdot R_{axis}(q_{18})$ where:

$$
T_{fixed} = \begin{bmatrix}
-0.766044 & -0.642788 & 0 & 0.015 \\
0.642788 & -0.766044 & 0 & 0.029 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{18}) = \begin{bmatrix}
c_{18} & -s_{18} & 0 & 0 \\
s_{18} & c_{18} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_1_tibia_mount

$L_{26}$ **leg_1_mx28_tibia** -> $L_{32}$ **leg_1_tibia** (fixed)

- **origin xyz**: (0, -0.0133, 0) m
- **origin rpy**: (3.141593, 0, 1.570796) rad

### Local Transform

$$
T^{26}_{32} = \begin{bmatrix}
0 & 1 & 0 & 0 \\
1 & 0 & 0 & -0.0133 \\
0 & 0 & -1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_2_tibia_mount

$L_{27}$ **leg_2_mx28_tibia** -> $L_{33}$ **leg_2_tibia** (fixed)

- **origin xyz**: (0, -0.0133, 0) m
- **origin rpy**: (-3.141593, 0, 1.570796) rad

### Local Transform

$$
T^{27}_{33} = \begin{bmatrix}
0 & 1 & 0 & 0 \\
1 & 0 & 0 & -0.0133 \\
0 & 0 & -1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_3_tibia_mount

$L_{28}$ **leg_3_mx28_tibia** -> $L_{34}$ **leg_3_tibia** (fixed)

- **origin xyz**: (0, -0.0133, 0) m
- **origin rpy**: (-3.141593, 0, 1.570796) rad

### Local Transform

$$
T^{28}_{34} = \begin{bmatrix}
0 & 1 & 0 & 0 \\
1 & 0 & 0 & -0.0133 \\
0 & 0 & -1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_4_tibia_mount

$L_{29}$ **leg_4_mx28_tibia** -> $L_{35}$ **leg_4_tibia** (fixed)

- **origin xyz**: (0, -0.0133, 0) m
- **origin rpy**: (-3.141593, 0, 1.570796) rad

### Local Transform

$$
T^{29}_{35} = \begin{bmatrix}
0 & 1 & 0 & 0 \\
1 & 0 & 0 & -0.0133 \\
0 & 0 & -1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_5_tibia_mount

$L_{30}$ **leg_5_mx28_tibia** -> $L_{36}$ **leg_5_tibia** (fixed)

- **origin xyz**: (0, -0.0133, 0) m
- **origin rpy**: (-3.141593, 0, 1.570796) rad

### Local Transform

$$
T^{30}_{36} = \begin{bmatrix}
0 & 1 & 0 & 0 \\
1 & 0 & 0 & -0.0133 \\
0 & 0 & -1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## leg_6_tibia_mount

$L_{31}$ **leg_6_mx28_tibia** -> $L_{37}$ **leg_6_tibia** (fixed)

- **origin xyz**: (0, -0.0133, 0) m
- **origin rpy**: (-3.141593, 0, 1.570796) rad

### Local Transform

$$
T^{31}_{37} = \begin{bmatrix}
0 & 1 & 0 & 0 \\
1 & 0 & 0 & -0.0133 \\
0 & 0 & -1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Global Transform Chains

Transform from root $L_0$ to any link, as product of local transforms along the kinematic chain.

$$T^{0}_{7} = T^{0}_{1} \cdot T^{1}_{7}(q_{1})\quad (L_0 \to L_{7}: \text{leg_1_coxa})$$

$$T^{0}_{8} = T^{0}_{2} \cdot T^{2}_{8}(q_{2})\quad (L_0 \to L_{8}: \text{leg_2_coxa})$$

$$T^{0}_{9} = T^{0}_{3} \cdot T^{3}_{9}(q_{3})\quad (L_0 \to L_{9}: \text{leg_3_coxa})$$

$$T^{0}_{10} = T^{0}_{4} \cdot T^{4}_{10}(q_{4})\quad (L_0 \to L_{10}: \text{leg_4_coxa})$$

$$T^{0}_{11} = T^{0}_{5} \cdot T^{5}_{11}(q_{5})\quad (L_0 \to L_{11}: \text{leg_5_coxa})$$

$$T^{0}_{12} = T^{0}_{5} \cdot T^{5}_{12}\quad (L_0 \to L_{12}: \text{bot})$$

$$T^{0}_{13} = T^{0}_{6} \cdot T^{6}_{13}(q_{6})\quad (L_0 \to L_{13}: \text{leg_6_coxa})$$

$$T^{0}_{14} = T^{0}_{1} \cdot T^{1}_{7}(q_{1}) \cdot T^{7}_{14}(q_{7})\quad (L_0 \to L_{14}: \text{leg_1_mx28_femur})$$

$$T^{0}_{15} = T^{0}_{2} \cdot T^{2}_{8}(q_{2}) \cdot T^{8}_{15}(q_{8})\quad (L_0 \to L_{15}: \text{leg_2_mx28_femur})$$

$$T^{0}_{16} = T^{0}_{3} \cdot T^{3}_{9}(q_{3}) \cdot T^{9}_{16}(q_{9})\quad (L_0 \to L_{16}: \text{leg_3_mx28_femur})$$

$$T^{0}_{17} = T^{0}_{4} \cdot T^{4}_{10}(q_{4}) \cdot T^{10}_{17}(q_{10})\quad (L_0 \to L_{17}: \text{leg_4_mx28_femur})$$

$$T^{0}_{18} = T^{0}_{5} \cdot T^{5}_{11}(q_{5}) \cdot T^{11}_{18}(q_{11})\quad (L_0 \to L_{18}: \text{leg_5_mx28_femur})$$

$$T^{0}_{19} = T^{0}_{6} \cdot T^{6}_{13}(q_{6}) \cdot T^{13}_{19}(q_{12})\quad (L_0 \to L_{19}: \text{leg_6_mx28_femur})$$

$$T^{0}_{20} = T^{0}_{1} \cdot T^{1}_{7}(q_{1}) \cdot T^{7}_{14}(q_{7}) \cdot T^{14}_{20}\quad (L_0 \to L_{20}: \text{leg_1_femur})$$

$$T^{0}_{21} = T^{0}_{2} \cdot T^{2}_{8}(q_{2}) \cdot T^{8}_{15}(q_{8}) \cdot T^{15}_{21}\quad (L_0 \to L_{21}: \text{leg_2_femur})$$

$$T^{0}_{22} = T^{0}_{3} \cdot T^{3}_{9}(q_{3}) \cdot T^{9}_{16}(q_{9}) \cdot T^{16}_{22}\quad (L_0 \to L_{22}: \text{leg_3_femur})$$

$$T^{0}_{23} = T^{0}_{4} \cdot T^{4}_{10}(q_{4}) \cdot T^{10}_{17}(q_{10}) \cdot T^{17}_{23}\quad (L_0 \to L_{23}: \text{leg_4_femur})$$

$$T^{0}_{24} = T^{0}_{5} \cdot T^{5}_{11}(q_{5}) \cdot T^{11}_{18}(q_{11}) \cdot T^{18}_{24}\quad (L_0 \to L_{24}: \text{leg_5_femur})$$

$$T^{0}_{25} = T^{0}_{6} \cdot T^{6}_{13}(q_{6}) \cdot T^{13}_{19}(q_{12}) \cdot T^{19}_{25}\quad (L_0 \to L_{25}: \text{leg_6_femur})$$

$$T^{0}_{26} = T^{0}_{1} \cdot T^{1}_{7}(q_{1}) \cdot T^{7}_{14}(q_{7}) \cdot T^{14}_{20} \cdot T^{20}_{26}(q_{13})\quad (L_0 \to L_{26}: \text{leg_1_mx28_tibia})$$

$$T^{0}_{27} = T^{0}_{2} \cdot T^{2}_{8}(q_{2}) \cdot T^{8}_{15}(q_{8}) \cdot T^{15}_{21} \cdot T^{21}_{27}(q_{14})\quad (L_0 \to L_{27}: \text{leg_2_mx28_tibia})$$

$$T^{0}_{28} = T^{0}_{3} \cdot T^{3}_{9}(q_{3}) \cdot T^{9}_{16}(q_{9}) \cdot T^{16}_{22} \cdot T^{22}_{28}(q_{15})\quad (L_0 \to L_{28}: \text{leg_3_mx28_tibia})$$

$$T^{0}_{29} = T^{0}_{4} \cdot T^{4}_{10}(q_{4}) \cdot T^{10}_{17}(q_{10}) \cdot T^{17}_{23} \cdot T^{23}_{29}(q_{16})\quad (L_0 \to L_{29}: \text{leg_4_mx28_tibia})$$

$$T^{0}_{30} = T^{0}_{5} \cdot T^{5}_{11}(q_{5}) \cdot T^{11}_{18}(q_{11}) \cdot T^{18}_{24} \cdot T^{24}_{30}(q_{17})\quad (L_0 \to L_{30}: \text{leg_5_mx28_tibia})$$

$$T^{0}_{31} = T^{0}_{6} \cdot T^{6}_{13}(q_{6}) \cdot T^{13}_{19}(q_{12}) \cdot T^{19}_{25} \cdot T^{25}_{31}(q_{18})\quad (L_0 \to L_{31}: \text{leg_6_mx28_tibia})$$

$$T^{0}_{32} = T^{0}_{1} \cdot T^{1}_{7}(q_{1}) \cdot T^{7}_{14}(q_{7}) \cdot T^{14}_{20} \cdot T^{20}_{26}(q_{13}) \cdot T^{26}_{32}\quad (L_0 \to L_{32}: \text{leg_1_tibia})$$

$$T^{0}_{33} = T^{0}_{2} \cdot T^{2}_{8}(q_{2}) \cdot T^{8}_{15}(q_{8}) \cdot T^{15}_{21} \cdot T^{21}_{27}(q_{14}) \cdot T^{27}_{33}\quad (L_0 \to L_{33}: \text{leg_2_tibia})$$

$$T^{0}_{34} = T^{0}_{3} \cdot T^{3}_{9}(q_{3}) \cdot T^{9}_{16}(q_{9}) \cdot T^{16}_{22} \cdot T^{22}_{28}(q_{15}) \cdot T^{28}_{34}\quad (L_0 \to L_{34}: \text{leg_3_tibia})$$

$$T^{0}_{35} = T^{0}_{4} \cdot T^{4}_{10}(q_{4}) \cdot T^{10}_{17}(q_{10}) \cdot T^{17}_{23} \cdot T^{23}_{29}(q_{16}) \cdot T^{29}_{35}\quad (L_0 \to L_{35}: \text{leg_4_tibia})$$

$$T^{0}_{36} = T^{0}_{5} \cdot T^{5}_{11}(q_{5}) \cdot T^{11}_{18}(q_{11}) \cdot T^{18}_{24} \cdot T^{24}_{30}(q_{17}) \cdot T^{30}_{36}\quad (L_0 \to L_{36}: \text{leg_5_tibia})$$

$$T^{0}_{37} = T^{0}_{6} \cdot T^{6}_{13}(q_{6}) \cdot T^{13}_{19}(q_{12}) \cdot T^{19}_{25} \cdot T^{25}_{31}(q_{18}) \cdot T^{31}_{37}\quad (L_0 \to L_{37}: \text{leg_6_tibia})$$

