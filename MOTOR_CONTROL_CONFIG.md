# Rolling Ball Stepper Control Notes

## Unit Conversion

K230 sends a normalized horizontal offset. Current calibration:

```text
K230 offset ~= +/-12.0
real track length = 25cm = 250mm
track half length = 125mm
```

Internal STM32 unit:

```text
visionOffsetX10 = K230 offset * 10
ballPositionMmX10 = ball position in mm * 10
```

Conversion:

```text
ballPositionMmX10 = visionOffsetX10 * 1250 / 120
```

Target requirement conversion:

```text
allowed actual offset = 0.5cm = 5mm = 50(mm*10)
equivalent K230 value ~= 5 / 125 * 12 = 0.48
equivalent visionOffsetX10 ~= 4.8, use 5 as practical threshold
```

Examples:

| K230 value | `visionOffsetX10` | Physical position |
| --- | --- | --- |
| `12.00` | `120` | `+125.0mm` |
| `6.00` | `60` | `+62.5mm` |
| `1.00` | `10` | `+10.4mm` |
| `0.30` | `3` | `+3.1mm` |
| `-12.00` | `-120` | `-125.0mm` |

The conversion is implemented in [main.c](D:/stm/123/Core/Src/main.c:397):

```c
static int32_t VisionOffsetX10ToBallMmX10(int32_t visionOffsetX10)
{
  return (visionOffsetX10 * BALL_TRACK_HALF_MM_X10) / VISION_OFFSET_MAX_X10;
}
```

## Main Parameters

| Macro | Value | Unit / Meaning |
| --- | --- | --- |
| `VISION_OFFSET_MAX_X10` | `120L` | K230 max offset `12.0 * 10` |
| `BALL_TRACK_LENGTH_MM` | `250L` | Real track length, 250mm |
| `BALL_TRACK_HALF_MM_X10` | `1250L` | Half track, 125.0mm |
| `BALL_TARGET_MAX_ERROR_MM_X10` | `50L` | Target max error, +/-5.0mm |
| `BALL_TARGET_MAX_ERROR_CM_X100` | `50L` | Same target expressed as 0.50cm |
| `BALL_TARGET_MAX_ERROR_VISION_X10` | `5` | Equivalent K230 threshold about `0.5` |
| `BALL_POSITION_DEADBAND_MM_X10` | `15L` | Center deadband, +/-1.5mm |
| `BALL_CENTER_BRAKE_WINDOW_MM_X10` | `120L` | Early braking window, +/-12.0mm |
| `MOTOR_ANGLE_LIMIT_DEG_X10` | `300L` | Angle limit, +/-30.0deg |
| `MOTOR_POS_SPEED_RPM` | `180U` | Position mode speed |
| `MOTOR_POS_ACC` | `10U` | Driver acceleration upper limit for PID mapping |
| `CONTROL_TASK_PERIOD_MS` | `5U` | Fixed control period driven by system tick |
| `MOTOR_MAX_STEP_PULSE` | `260L` | Legacy ordinary step limit, no longer used by main PID path |
| `MOTOR_CMD_INTERVAL_MS` | `3U` | Command interval |

## Pseudo-Velocity PID Control

The control core is now a pseudo-velocity PID. It only needs the visual position
feedback from K230, so it is simpler than a true position-loop plus velocity-loop
controller:

```text
position_error = ball_position - target_position
pseudo_velocity = (position_error - last_position_error) / dt

target_angle =
    KP * position_error
  + KI * integral(position_error)
  + KD * pseudo_velocity
```

Current signal flow:

```text
position_error = ball_position - target_position
pseudo_velocity = (position_error - last_position_error) / dt
target_angle = PID(position_error, pseudo_velocity)
motor_pulse = angle_to_pulse(target_angle)
motor_speed_rpm = f(abs(pseudo_velocity), abs(target_angle))
motor_acc = f(abs(target_angle), abs(pseudo_velocity))
```

The meaning is:

```text
KP: main correction strength. Raise it to respond faster to position error.
KD: pseudo-velocity damping / advance braking. Raise it to fight lag and overshoot.
KI: slow center-bias correction. Keep it at 0 first; add only a very small value if needed.
```

Current runtime gains:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `KP` | `900` | Position gain, sent in permille, so `900` means `0.900` |
| `KI` | `0` | Integral gain, sent in permille |
| `KD` | `80` | Pseudo-velocity gain, sent in permille, so `80` means `0.080` |
| `PSEUDO_PID_INTEGRAL_LIMIT` | `500000L` | Integral clamp reused by pseudo PID |
| `PID_DRIVE_SPEED_MIN_RPM` | `30U` | Minimum driver speed from PID mapping |
| `PID_DRIVE_SPEED_TARGET_VEL_DEN` | `20L` | Lower means pseudo velocity raises RPM faster |
| `PID_DRIVE_SPEED_ERROR_DEN` | `30L` | Lower means target angle raises RPM faster |
| `PID_DRIVE_ACC_MIN` | `0U` | Minimum driver acceleration from PID mapping |
| `PID_DRIVE_ACC_ANGLE_DEN` | `180L` | Lower means angle command raises acceleration faster |
| `PID_DRIVE_ACC_ERROR_DEN` | `900L` | Lower means pseudo velocity raises acceleration faster |
| `BALL_PID_DT_MIN_MS` | `5U` | dt lower limit |
| `BALL_PID_DT_MAX_MS` | `200U` | dt upper limit |

Compatibility note:

```text
Old VKP/VKI/VKD commands are still accepted by STM32 as aliases, but the upper
computer now sends KP/KI/KD directly.
```

## Measurement Processing

Vision input and motor control are now decoupled:

```text
UART frame arrival: parse and store latest ball position only
5ms control task: read latest processed position, run PID, send motor command
```

Current data processing:

```text
filtered_position = (1 * last_filtered + 4 * new_measurement) / 5
predicted_position = filtered_position + vision_velocity * predict_ms
predict_ms <= 15ms
```

Related macros:

| Macro | Value | Meaning |
| --- | --- | --- |
| `VISION_FILTER_OLD_NUM` | `1L` | Previous filtered weight |
| `VISION_FILTER_NEW_NUM` | `4L` | New measurement weight |
| `VISION_FILTER_DEN` | `5L` | Filter denominator |
| `VISION_PREDICT_MAX_MS` | `15U` | Maximum short-term prediction window |

## Stiction Compensation

Static friction compensation also uses real position:

| Macro | Value | Meaning |
| --- | --- | --- |
| `STICK_OFFSET_START_MM_X10` | `60L` | Start boost above 6.0mm offset |
| `STICK_OFFSET_IMPROVE_MM_X10` | `20L` | Clear boost if position improves by 2.0mm |
| `STICK_RAMP_DELAY_MS` | `50U` | Wait before boost |
| `STICK_RAMP_INTERVAL_MS` | `35U` | Add boost interval |
| `STICK_RAMP_STEP_ANGLE_X10` | `2L` | Add 0.2deg each step |
| `STICK_MAX_BOOST_ANGLE_X10` | `15L` | Max extra 1.5deg |

## Pulse Conversion

Motor data:

```text
360deg = 3200 pulses
1deg = 8.8889 pulses
targetPulse = targetAngleX10 * 3200 / 3600
```

The motor command uses the original Emm_V5 position-control function:

```c
Emm_V5_Pos_Control(MOTOR_ADDR, dir, speedRpm, acc, pulse, 1, false);
```

That means we are combining:

```text
upper layer: pseudo-velocity PID + stiction compensation decides target angle
lower layer: driver position mode with PID-derived speed and acceleration
```

The old ordinary command smoothing path is not used in the main control path now:

```text
target pulse comes directly from pseudo PID target angle
driver speed and acceleration also come from pseudo PID state
```

## Tuning Order

1. Start with `KP=0.900`, `KI=0.000`, `KD=0.080`.
2. If the motor still follows slowly, raise `KP` first, for example `1.100`, then `1.300`.
3. If it pushes the ball through center or oscillates, raise `KD`, for example `0.100` to `0.160`.
4. If small offset cannot overcome friction, raise `STICK_RAMP_STEP_ANGLE_X10` or `STICK_MAX_BOOST_ANGLE_X10`.
5. If driver acceleration feels too soft, raise `ACC` or reduce `PID_DRIVE_ACC_ANGLE_DEN`.
6. If it always settles with a fixed bias, add a very small `KI`, for example `0.001` to `0.005`.
