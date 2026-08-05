# HexNav Hexapod - Comprehensive Movement Failure Analysis (Gemini Analysis)

## 1. Executive Summary

A deep, systematic engineering analysis of the HexNav firmware ([firmware/openrb150](firmware/openrb150)) and the URDF description ([robot_ros_simulation/HexNav_description/urdf/HexNav.urdf](robot_ros_simulation/HexNav_description/urdf/HexNav.urdf)) was performed. 

The primary conclusion is that **the math, gait tables, and inverse kinematics solvers themselves are robust and correctly derived from the Phoenix reference.** However, **two dramatic, foundational paramter/mapping defects in the configuration and integration layer completely prevent correct walking:**

1. **Dimensional Mismatch (Stride Collapse to ~7mm)**: The firmware models the tibia length as only **$24.86$ mm** (the joint offset to the link coordinate frame origin in the URDF) instead of the **$123.14$ mm** physical length of the femur-to-tip leg segment on this robot (Mark III uses $133.00$ mm). Since the mathematical tibia is far too short, setting the body height to $40$ mm and stride goals to $60$ mm places the feet near or beyond the extreme boundary of physical possibility. The reachability-aware safety shaper in `gait_pipeline.cpp` scales down horizontal travel to keep it reachable, shrinking the $60$ mm stroke to a tiny **$7$ mm** shuffle.
2. **Coordinate & Index Realignment (Chaotic Movements)**: Swapped coordinate axes (Right-Handed ROS vs. Left-Handed Firmware) and reversed leg index bindings send logical trajectories for one leg to a completely different physical leg on the robot. Specifically, Rear-Left and Rear-Right coordinates, and Front-Left and Front-Right coordinates are cross-circuited. Furthermore, forcing all servo directions to `sign = 1` ignores physical mirror-symmetry rules, causing opposite physical segments to fight the central locomotion.

---

## 2. In-Depth Technical Findings and Evidence

### 2.1 The Miniature Tibia Defect (`links.tibia_cmm`)
In [firmware/openrb150/src/config/config_schema.cpp](firmware/openrb150/src/config/config_schema.cpp), the mechanical links are initialized as:
* `cfg.links.coxa_cmm = 5608` ($56.08$ mm)
* `cfg.links.femur_cmm = 6651` ($66.51$ mm)
* `cfg.links.tibia_cmm = 2486` ($24.86$ mm)

**The Failure**: In the URDF, the coordinate frame `leg_n_tibia` is indeed positioned $24.86$ mm from the knee axis. But the **actual physical leg tip extensions (with the sensor foot) extend well past this coordinate origin down to the ground.** The physical leg tip-to-knee length on this robot is **$123.14$ mm**.

**The Stride Collapse Math**:
When the solver executes with a $24.86$ mm tibia, the planar 2-link reach $d$ has:
* $L_2 = 66.51$ mm (femur)
* $L_3 = 24.86$ mm (tibia)
* Maximum mathematical reach limit: $L_2 + L_3 = 91.37$ mm.

The default neutral foot sits at radius $127.0$ mm and height $-44.55$ mm. After coxa subtraction ($127.0 - 56.08 = 70.92$ mm), the diagonal planar reach $d$ is:
$$d = \sqrt{70.92^2 + (-44.55)^2} \approx 83.75 \text{ mm}$$

This target leaves a workspace margin of just $7.62$ mm. When `kReachMarginFrac = 0.95` is applied, the allowable limit is strictly capped at $86.8$ mm. High-level commands for a $60$ mm stride (requiring $+-30$ mm travel) ask for a planar reach that exceeds the boundary.
The global path scale shaper in `gait_pipeline.cpp` discovers this constraint:
$$\text{scale} \approx 0.12$$
It scales down the horizontal travel of *every* leg to keep the gait synchronized, collapsing the $60$ mm stride into a useless **$7$ mm jittering shuffle**.

---

### 2.2 Misaligned Leg Indexing and Swapped Coordinate Frames
The physical robot uses a right-handed ROS coordinate frame (**+X Forward, +Y Left, +Z Up**), whereas the gait engine uses a left-handed system (**+X Right/Lateral, +Y Forward/Longitudinal, +Z Up**). This maps physical targets to different, unexpected quadrants when mapped:

| Leg Index | URDF Real Position | Firmware Intended Role | Mapping Impact |
|:---:|:---|:---|---|
| **0** | **Rear-Right** | **Rear-Left** | Swaps left and right motion paths! |
| **1** | **Front-Right** | **Rear-Right** | Swaps front and rear control signals! |
| **2** | **Middle-Right** | **Middle-Right** | Aligned in position but reversed in coxa direction. |
| **3** | **Front-Left** | **Front-Right** | Swaps left and right motion paths! |
| **4** | **Rear-Left** | **Front-Left** | Swaps front and rear control signals! |
| **5** | **Middle-Left** | **Middle-Left** | Aligned in position but reversed in coxa direction. |

This complete transposition of mathematical roles maps kinematics computed for a "Rear-Left" swing phase onto the physical "Rear-Right" actuator channel, creating chaotic, apparently random leg movement.

---

### 2.3 The positive-only Mirror Inflow (`sign = 1`)
A symmetrical robot requires mirrored servo direction signs on opposite sides of the chassis to elevate, retract, or push. 
* Left-side legs and right-side legs must use inverse joint direction signs.
* The current configuration default forces `sign = 1` for all 18 servos.
* Additionally, when firmware upgrades occur, the legacy migration function `applyRobotMotionProfile(out)` erases custom EEPROM direction and trim configurations, forcing them back to the all-positive default.
The joints therefore act in reverse relative to their mathematical stance phases.

---

### 2.4 Lack of Gait-Decay and Settle on Stop
* In [firmware/openrb150/src/gait/gait_engine.cpp](firmware/openrb150/src/gait/gait_engine.cpp), the moment command twist magnitude reaches zero, the code immediately snaps the foot goals of all six legs back to their home stance.
* This bypasses the normal step completion: legs that are currently elevated (in swing) instantly snap back in a single control cycle.
* There is no stop-settle-switch sequence (as implemented in the Phoenix reference), resulting in heavy chassis vibration and mechanical jerking.

---

## 3. Systematic Integration Plan

```mermaid
graph TD
    classDef error fill:#fee2e2,stroke:#ef4444,stroke-width:2px;
    classDef fix fill:#ccfbf1,stroke:#0d9488,stroke-width:2px;

    U[URDF Reference] ---> M[1. Model Realignment]
    T_err[Tibia = 24.86 mm] :::error -->|Integrate physical tip length| T_fixed[Tibia = 123.14 mm] :::fix
    T_fixed --> M
    M -->|Wider allowed workspace| Stride[Full 60mm Stride scale=1.0] :::fix

    L_err[Swapped Leg Mappings] :::error -->|2. Align Leg Indices and Roles| L_fixed[Align logical indices strictly to physical ROS channels] :::fix
    L_fixed --> T_correct(Correct 18.channel allocation) :::fix

    Sign_err[All joints sign = 1] :::error -->|3. Symmetric Servo Signs| Sign_fixed[Mirror Group Directions: Left +1, Right -1] :::fix
    Sign_fixed --> Output(Verifiable locomotion) :::fix
```

### 3.1 Step 1: Realize True Physical Dimensions
In [firmware/openrb150/src/config/config_schema.cpp](firmware/openrb150/src/config/config_schema.cpp), restore physical leg tip lengths:
```cpp
cfg.links.coxa_cmm = 5608;    // 56.08 mm
cfg.links.femur_cmm = 6651;   // 66.51 mm
cfg.links.tibia_cmm = 12314;  // 123.14 mm (Knee to actual rubber tip contact)
```
Update the default stance height and radial values to match:
`cfg.geometry.home_radius_cmm = 14700;` ($147$ mm)
`cfg.geometry.home_foot_z_cmm = -6000;` ($-60$ mm ground stance)

### 3.2 Step 2: Enforce Symmetrical Servo Directions
Map mechanical servo directions according to side-aware mirror symmetry:
- **Left Legs (0, 4, 5)**: `sign = +1`
- **Right Legs (1, 2, 3)**: `sign = -1`

### 3.3 Step 3: Align Index Sequences and Coordinate Chains
Ensure that the logical leg-to-channel interface aligns with the physical ROS controllers. Translate the coordinate axes consistently to map longitudinal travel directly to forward.

### 3.4 Step 4: Implement Phase-Safe Stopping Settle
In `gait_engine.cpp`, implement a slow decay instead of an instant snap-to-home. Let any active step complete its swing phase and register a solid ground touchdown before transitioning back to the stationary default stance.
