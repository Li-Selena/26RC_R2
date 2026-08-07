# MCU task-flow commands

The MCU currently provides `S1_UP_V2`, `S1_DOWN_V2`, `THROW_BLOCK_V1`, and `WEAPON_DOCK_TEST_V1`.
The former `S1_UP_S2_UP_V2` flow and its USB command `0x62` have been removed.

## Preconditions

- `S1_UP_V2` and `S1_DOWN_V2` first return the arm to `ARM_ENABLE_HOME`.
- Both pickup flows turn on suction 2 through active-high PE13 before raising the arm.
- Every J2/J3 state waits for the commanded joint feedback to stay within 1 degree of target for 100 ms; a step times out after 40 seconds.
- `THROW_BLOCK_V1` accepts only X+ or X- and requires a successful `S1_UP_V2` or `S1_DOWN_V2` completion marker.
- The throw flow keeps suction enabled while switching workspace, restores the saved J2/J3 pose, and then calls the suction 2 off interface.
- Releasing consumes the pickup marker, so another throw is rejected until a pickup flow completes again.
- Suction 1 output has been removed. The kinematic S1 tool name remains internal to arm pose calculations and has no actuator GPIO.

## serial_tool shell

```text
usb
S1_UP_V2
task_status

usb
S1_DOWN_V2
task_status

usb
throw_block X+
task_status

usb
throw_block X-
task_status
```

## serial_tool CLI

```powershell
python -m serial_tool send --port COM30 --baud 115200 S1_UP_V2 --wait 0.2
python -m serial_tool send --port COM30 --baud 115200 S1_DOWN_V2 --wait 0.2
python -m serial_tool send --port COM30 --baud 115200 THROW_BLOCK 1 --wait 0.2
python -m serial_tool send --port COM30 --baud 115200 THROW_BLOCK 2 --wait 0.2
python -m serial_tool poll --port COM30 --baud 115200 FLOW_GET_STATUS --rate 10
python -m serial_tool send --port COM30 --baud 115200 FLOW_WEAPON_GRAB --wait 0.2
python -m serial_tool send --port COM30 --baud 115200 FLOW_WEAPON_DOCK_TEST --wait 0.2
python -m serial_tool send --port COM30 --baud 115200 FLOW_CHASSIS_MOVE_DONE --wait 0.2
python -m serial_tool send --port COM30 --baud 115200 FLOW_DOCK_DONE --wait 0.2
```

## Serial assistant hex

```text
S1_UP_V2:        A5 5A 00 60 D3 02 FF
S1_DOWN_V2:      A5 5A 00 61 13 C3 FF
FLOW_GET_STATUS: A5 5A 00 66 D1 82 FF
THROW_BLOCK X+:  A5 5A 10 68 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 84 45 FF
THROW_BLOCK X-:  A5 5A 10 68 00 00 00 40 00 00 00 00 00 00 00 00 00 00 00 00 FB 6F FF
WEAPON_GRAB:       A5 5A 00 67 11 43 FF
WEAPON_DOCK_TEST:  A5 5A 00 69 D5 C2 FF
CHASSIS_MOVE_DONE: A5 5A 00 6A D4 82 FF
DOCK_DONE:         A5 5A 00 6B 14 43 FF
```

## Flow outlines

`S1_UP_V2`:

```text
01 ARM_ENABLE_HOME
02 CLIMB_AUTO_PAUSE upstairs
03 arm_j3_cw_60deg
04 arm_j3_cw_10deg
05 arm_j3_cw_5deg
06 arm_j2_cw_20deg
07 arm_j2_cw_1deg x4
08 arm_j2_cw_60deg
09 suction_2_on (PE13 high)
10 arm_j2_ccw_60deg
11 arm_j3_cw_30deg
12 CLIMB_AUTO_RESUME
```

Steps 03-07 are joint-angle equivalent to the replaced S1 sequence: J3 net CW75 degrees followed by J2 net CW24 degrees. The intermediate trajectory is different.

`S1_DOWN_V2` remains unchanged:

```text
01 ARM_ENABLE_HOME
02 CLIMB_AUTO_PAUSE downstairs
03 arm_j3_cw_90deg
04 arm_j3_cw_10deg
05 climb_all_legs_down_10
06 arm_j2_cw_90deg
07 arm_j3_cw_10deg
08 arm_j3_cw_5deg
09 arm_j2_cw_5deg
10 suction_2_on (PE13 high)
11 arm_j2_ccw_90deg
12 climb_all_legs_up_10
13 CLIMB_AUTO_RESUME
```

`THROW_BLOCK_V1 X+|X-`:

```text
01 PRECONDITION
   completed_s1_end must be S1_UP_END or S1_DOWN_END
02 ARM_WORKSPACE_SWITCH
   save current motor-feedback joint pose
   switch to requested X+ or X-
   wait for workspace switching to finish and restore J2/J3
03 TOOL_OFF_HELD
   S1_UP_END or S1_DOWN_END -> turn off suction 2 (PE13 low)
04 DONE
   clear completed_s1_end
```

Without a pickup marker, `FLOW_GET_STATUS` reports `THROW_BLOCK_V1 / ERROR / PRECONDITION`; the arm does not move and suction is not changed.
