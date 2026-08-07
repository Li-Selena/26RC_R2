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

or, through the newer command path:

```text
tool, state, target_xyz_mm
```

and returns:

```text
theta1_rad, theta2_rad, theta3_rad
```

`theta1_rad` is the snapped work-plane yaw derived from either
`approach_yaw_rad` or `target_xyz_mm.x/y`: `Y+=0`, `X+=-pi/2`,
`X-=+pi/2`. `target_xyz_mm.z` participates in the J2/J3 height inverse
solution. `target_xyz_mm.x/y` only select the work space; planar distance that
the arm cannot reach is left to the chassis controller and is not implemented
inside the arm IK.

Tool posture names are defined in the `Y+` work-space template. When the arm
switches to `X+` or `X-`, the same posture is rotated by the selected J1 yaw
and becomes the equivalent local work-space direction: template `Y+` maps to
world `Y0+` in `Y+`, to world `X0+` in `X+`, and to world `X0-` in `X-`.
Vertical `Z0+ / Z0-` posture constraints stay world-vertical.

The posture constraint is represented by:

```text
phi = theta2 + theta3
```

where the numeric `state` is interpreted by the selected `tool`:

| tool | state | meaning | posture constraint |
|---|---:|---|---|
| S1 | 0 | prepare to pick a block | `S1Z+ // Z0-`, `phi=pi` |
| S1 | 1 | carrying a picked block | S1 keeps its offset, but copies the latest successful S2 posture |
| S1 | 2 | place a block | `S1Z+ // Z0-`, `phi=pi` |
| S2 | 0 | prepare to pick | suction plane is `15deg` to the horizontal block plane, tilted toward `Y0+`; `phi=pi+15deg` |
| S2 | 1 | suction down / picking | `S2Z+ // Z0-`, `phi=pi` |
| S2 | 2 | short-link parallel | `S2Z+` is parallel and same direction as the short crank segment from the bend point to J4; `phi=alpha-30deg-90deg` |
| S2 | 3 | place a block | `S2Z+ // Y0+`, `phi=-pi/2` |
| gripper | 0 | gripper up | `GZ+ // Y0+`, `phi=-pi/2` |
| gripper | 1 | gripper down | `GZ+ // Y0-`, `phi=+pi/2` |
| gripper | 2 | gripper forward / power-on | `GZ+ // Z0-`, `phi=pi` |

States not listed above are unsupported. The legacy `STOW` and `USE` aliases
remain only as numeric aliases for state `0` and state `1`; callers must still
interpret the state through the selected tool.

If IK is used without an explicit tool posture call, frame 4 keeps the power-on
orientation. In IK, J3 is therefore compensated by
`theta3 = pi - theta2` for that default posture.

Tool posture sketch:

![Tool posture states](tool_posture_states.svg)

## Convex Working Branch

The actual motor zero is the `-Y0` concave posture. After the startup move, all
normal `Y+ / X+ / X-` work-space poses use the convex side-view branch. For the
long physical segment angle `alpha`:

```text
offset = atan2(short * sin(30deg), long + short * cos(30deg))
gamma  = alpha - offset
theta2 = gamma - pi/2
alpha  = theta2 + pi/2 + offset
```

This sign is important: `gamma = alpha + offset` draws the opposite bent-link
side and appears concave in the normal work spaces.

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
theta3      = unwrap_near(phi - theta2, current_theta3)
```

The `theta3` solution is intentionally continuous. Keeping frame 4 stable must
not fold the compensation back into `[-180deg, 180deg]`; otherwise J3 may choose
the opposite equivalent rotation and the tool posture appears to drift with J2.

Current firmware records the following `target_z_mm` protection ranges. These
are tool-origin heights in the selected `Y+ / X+ / X-` work plane; changing
work direction changes yaw only and does not change the z range. The last
column is the protected ready height at `alpha=70deg`.

| tool/state | min z mm | max z mm | z at alpha=70deg mm |
|---|---:|---:|---:|
| S1 state 0, prepare | -259.239 | 431.525 | 387.379 |
| S1 state 1, carry by S2 | runtime S2 posture source | runtime S2 posture source | runtime S2 posture source |
| S1 state 2, place | -259.239 | 431.525 | 387.379 |
| S2 state 0, prepare 15deg | -237.706 | 453.058 | 408.912 |
| S2 state 1, suction down | -213.739 | 477.025 | 432.879 |
| S2 state 2, short-link parallel | -237.706 | 430.725 | 361.943 |
| S2 state 3, place Y+ | -306.339 | 384.425 | 340.279 |
| gripper state 0, up | -121.739 | 569.025 | 524.879 |
| gripper state 1, down | -305.739 | 385.025 | 340.879 |
| gripper state 2, forward | -224.039 | 466.725 | 422.579 |

Enable/disable physical topology, viewed in the `OYZ` side plane at `yaw=0`
(`Y+` work plane):

![Arm enable/disable physical topology](arm_enable_disable_physical_topology.svg)

Yaw work-plane switching is protected as a staged trajectory:

1. If the requested target moves between `Y+`, `X+`, and `X-`, the arm first
   commands the current work plane to `alpha=80deg`.
2. Only after J2 feedback confirms `alpha > 70deg` does J1 yaw rotate toward
   the new work-plane angle: `Y+=0deg`, `X+=-90deg`, `X-=+90deg`.
3. The original lower target is released only after feedback confirms yaw is
   within `2deg` of the target angle and `alpha > 70deg`.

When both `+theta2` and `-theta2` are valid, the configured preferred branch
is used. The default prefers the forward branch (`theta2 < 0`), but the joint
limit table can override this by rejecting a candidate.

## Safety Placeholders

Current default limits are compile-time placeholders:

```text
J1: [-pi, pi]
J2: [-pi/2, pi/2]
J3: [-2pi, 2pi]
```

Actual motor zero offsets, motor directions, and mechanical hard limits should
be filled in after bench calibration.
