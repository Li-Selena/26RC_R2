# Actual Mechanical Arm Power-On Model

This file records the current physical model to use for follow-up kinematics,
zero-offset, motor-angle mapping, and diagrams. This actual power-on posture is
also the motor/encoder zero definition. It supersedes the earlier "power-on
vertical link" assumption for the real robot.

There are two different zero definitions:

```text
motor zero / encoder zero:
  the actual power-on concave posture recorded in this file

solver model zero / IK zero:
  the line between the actual J2 and J3 rotation joints is vertical
  frame 4 is parallel to the mechanical arm / J0 frame
```

## J0 Frame And View

Robot / joint0 frame:

```text
+X0: points out of the screen in the J0 OYZ side view
+Y0: horizontal axis in the side view
+Z0: vertical up
XOY plane: Z0 = 0
```

All side-view descriptions below are from the `OYZ` plane of `J0`.

## Joint And Motor Axis Facts

```text
J1 output shaft direction: +Z0
J1 motor output shaft : actual joint axis = 1 : 1

J2 output shaft direction: -X0
J2 motor output shaft : actual joint axis = 30 : 55

J3 output shaft direction: +X0
J3 motor output shaft : actual joint axis = 1 : 2
```

The ratios above are stored exactly as the mechanical description gives them.
When implementing motor-angle mapping, explicitly choose and document whether
the code uses `joint_angle = motor_angle * input/output` or the inverse.

## J2-J3 Convex Link Geometry

The link between J2 and J3 is a rigid bent link, not an extra joint.

```text
J2-side long segment  = 270.98076211 mm
J3-side short segment = 124.55 mm
included angle        = 150 deg
endpoint chord        ~= 383.928539331 mm from the two segments
reference chord used in earlier MDH notes = 383.929445 mm
```

Shape rule in the J0 OYZ side view:

```text
When the link lies on the +Y0 half-axis, it appears convex.
When the link falls on the -Y0 half-axis, it appears concave.
```

Current power-on posture is the `-Y0` concave case.

## Actual Power-On Pose

This pose is the motor zero:

```text
J1 motor zero = this J1 pose
J2 motor zero = this J2 pose
J3 motor zero = this J3 pose
```

At this zero, the long J2-side segment points toward `-Y0` and its far end is
below the XOY plane reference through J2:

```text
long segment angle to XOY plane = -6 deg
```

Important: this `6 deg` belongs to the long physical segment, not to the
straight J2-J3 endpoint chord.

Using the segment lengths above and the concave `-Y0` layout:

```text
J1 / O0 = (0, 0, 0)
O1_mdh  = (0, 0, 98.18)
J2 / O2 = (0, 42.64, 98.18)

concave fold point ~= (0, -226.86, 69.85)
J3 / J4           ~= (0, -340.64, 120.51)
```

The straight J2-J3 chord is only a reference line in this actual pose. It is
not the line used for the `6 deg` mechanical zero description.

## Solver Model Zero

The solver/IK model zero is not the motor zero. In the solver model zero:

```text
the actual J2-J3 rotation-joint line is vertical
J4 frame is parallel to the mechanical arm / J0 frame
```

Using the same base dimensions as the current model:

```text
J1 / O0 = (0, 0, 0)
O1_mdh  = (0, 0, 98.18)
J2 / O2 = (0, 42.64, 98.18)
J3 / J4 = (0, 42.64, 482.109445)
```

At this solver zero:

```text
+X4 = +X0
+Y4 = +Y0
+Z4 = +Z0
```

The firmware/IK joint targets should therefore be interpreted relative to this
solver model zero, then converted to motor output angles using the motor-zero
offsets, motor shaft directions, and reduction ratios above.

## J4 Tool Frame At Power-On

The real power-on J4 frame differs from the earlier topology model:

```text
+Z4 = -Z0  (vertical down)
```

For the current diagram and a right-handed frame, use:

```text
+X4 = +X0
+Y4 = -Y0
+Z4 = -Z0
```

## Diagram Files

The current actual power-on pose is drawn in:

```text
Mechanical_Arm_Model/arm_actual_poweron_initial_pose.svg
Mechanical_Arm_Model/arm_actual_poweron_initial_pose.png
```
