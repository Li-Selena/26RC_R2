# New Mechanical Arm Kinematics Model

This note records the firmware model used by `robotarm_kinematics.c`.
It follows `arm_mdh_parameter_reference.md` as the single source of truth.

## Frames And Zero Pose

World / chassis frame:

```text
X0: right
Y0: forward
Z0: up
```

Power-on zero:

```text
J1/O0      = (0, 0, 0)
O1_mdh     = (0, 0, 98.18)
J2/O2      = (0, 42.64, 98.18)
J3/J4/O3/O4= (0, 42.64, 482.109445)
```

The zero-pose link from J2 to J3 is vertical:

```text
J3 - J2 = (0, 0, 383.929445)
```

## Tool Points

Tool points are expressed in joint4/tool frame coordinates:

| tool | x4 | y4 | z4 |
|---|---:|---:|---:|
| S1 suction | 0 | -46.79 | 45.5 |
| S2 suction | 0 | 92.6 | 0 |
| Gripper | 103 | -92 | 10.3 |

At zero, frame4 is parallel to frame0, so the world coordinates are:

| tool | x0 | y0 | z0 |
|---|---:|---:|---:|
| S1 suction | 0 | -4.15 | 527.609445 |
| S2 suction | 0 | 135.24 | 482.109445 |
| Gripper | 103 | -49.36 | 492.409445 |

## Firmware IK Contract

The v1 IK does not solve a full 6D end-effector pose. The chassis handles
planar `x/y` placement. The arm receives:

```text
tool, state, target_z_mm, approach_yaw_rad
```

and returns:

```text
theta1_rad, theta2_rad, theta3_rad
```

`theta1_rad` is the requested approach yaw, normalized to `[-pi, pi]`.

The posture constraint is represented by:

```text
phi = theta2 + theta3
```

with these fixed posture values:

| tool state | phi |
|---|---:|
| stow/stop, all tools | 0 |
| S1 use: S1 +Z down | pi |
| S2 use: S2 +Y down | -pi/2 |
| Gripper use: G -Y forward at yaw=0 | pi |

For a selected tool offset `(ox, oy, oz)`:

```text
tool_z = 98.18
       + 383.929445 * cos(theta2)
       + oy * sin(phi)
       + oz * cos(phi)
```

The solver computes:

```text
cos(theta2) = (target_z_mm - 98.18 - oy*sin(phi) - oz*cos(phi)) / 383.929445
theta3      = normalize_pi(phi - theta2)
```

When both `+theta2` and `-theta2` are valid, the configured preferred branch
is used. The default prefers the forward branch (`theta2 < 0`), but the joint
limit table can override this by rejecting a candidate.

## Safety Placeholders

Current default limits are compile-time placeholders:

```text
J1: [-pi, pi]
J2: [-pi/2, pi/2]
J3: [-pi, pi]
```

Actual motor zero offsets, motor directions, and mechanical hard limits should
be filled in after bench calibration.
