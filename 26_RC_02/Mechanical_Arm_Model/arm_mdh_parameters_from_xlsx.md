# Legacy Mechanical Arm MDH Parameters From XLSX

Source workbook: external XLSX file supplied by the user.

This file is kept as a legacy direct expansion of the earlier XLSX table. For
the corrected physical power-on zero and tool frame, use:

```text
arm_poweron_zero_model.md
arm_poweron_zero_topology_axes.svg
```

Convention used to draw `arm_mdh_topology_from_xlsx.svg`:

```text
T(i-1,i) = Rx(alpha(i-1)) * Tx(a(i-1)) * Rz(theta(i)) * Tz(d(i))
```

World frame `{0}`:

```text
X0: right
Y0: forward
Z0: up
```

## MDH Table

| i | frame | alpha(i-1) | a(i-1) | d(i) | theta(i) | note |
|---:|---|---:|---:|---:|---:|---|
| 1 | joint1 | 0 | 0 | 262.5 | theta1 + PI/2 | z1 passes joint1 and is same direction as Z0 |
| 2 | joint2 | PI/2 | 42.64 | 98.18 | theta2 | z2 passes joint2 and is same direction as X0 at zero pose |
| 3 | joint3 | 0 | 383.9294 | 0 | theta3 + PI/2 | z3 passes joint3 and is same direction as X0 at zero pose |
| 4 | tool | PI/2 | 0 | 0 | PI | legacy row; current tool frame is defined in `arm_poweron_zero_model.md` |

## Zero-Pose Frame Origins

With `theta1 = theta2 = theta3 = 0`, the table gives:

```text
O0 = (0, 0, 0)
O1 = (0, 0, 262.5)
O2 = (98.18, 42.64, 262.5)
O3 = (98.18, 426.5694, 262.5)
OT = O3
```

The tool frame shares its origin with joint3.

## Zero-Pose Axis Directions

```text
frame 0:
  X0 = +X0
  Y0 = +Y0
  Z0 = +Z0

frame 1 / joint1:
  X1 = +Y0
  Y1 = -X0
  Z1 = +Z0

frame 2 / joint2:
  X2 = +Y0
  Y2 = +Z0
  Z2 = +X0

frame 3 / joint3:
  X3 = +Z0
  Y3 = -Y0
  Z3 = +X0

legacy tool frame from this table:
  X4 = -Z0
  Y4 = -X0
  Z4 = +Y0
```

## Check Note

Under this MDH convention, the XLSX row `d2 = 98.18` translates along `Z2`.
Because `Z2 = +X0` at zero pose, joint2 appears at:

```text
O2 = (98.18, 42.64, 262.5)
```

If the intended physical point is instead:

```text
joint2 = (0, 42.64, 98.18)
```

then the placement of `262.5`, `98.18`, or the row-2 axis assignment should be
reviewed before using this table for forward/inverse kinematics.

Current correction: joint4/tool origin coincides with joint3, and at the
physical power-on zero its `Z4` axis is aligned with `Z0`. The tool frame then
follows joint3 rotation; it is not fixed by the joint2-to-joint3 link direction.
