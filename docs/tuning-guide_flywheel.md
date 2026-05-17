# Flywheel Tuning Guide

**For:** CyberCoyotes 4829 - 2026-Rebuilt  
**Goal:** Live tune the flywheel in Phoenix Tuner X 2026, using RPS graphs first, then copy the final gains back into code.

## Hardware And Code References

- Flywheel leader motor: `Constants.Flywheel.FLYWHEEL_LEFT_MOTOR_ID = 25`
- Flywheel follower motor: `Constants.Flywheel.FLYWHEEL_RIGHT_MOTOR_ID = 26`
- Bus: `Constants.RIO_CANBUS`
- Control mode in code: `VelocityVoltage` on Slot 0
- Code gain source: `Constants.Flywheel.KP`, `Constants.Flywheel.KV`, `Constants.Flywheel.KD`
- Config application: [ShooterIOHardware.java](/C:/Users/jvanscoyoc/.codex/worktrees/35ba/2026-Rebuilt/src/main/java/frc/robot/subsystems/shooter/ShooterIOHardware.java#L67)
- Flywheel IDs and presets: [Constants.java](/C:/Users/jvanscoyoc/.codex/worktrees/35ba/2026-Rebuilt/src/main/java/frc/robot/Constants.java#L251)
- Tuning command in robot code: [ShooterSubsystem.java](/C:/Users/jvanscoyoc/.codex/worktrees/35ba/2026-Rebuilt/src/main/java/frc/robot/subsystems/shooter/ShooterSubsystem.java#L499)

## What To Tune In Tuner X

Tune the **leader only** in Phoenix Tuner X:

- Device `25`: tune Slot 0 gains here
- Device `26`: do not tune closed-loop gains here; it should remain a follower

Verify before tuning:

- `25` is the motor receiving the velocity command
- `26` follows `25`
- follower direction matches `MotorAlignmentValue.Opposed`
- both wheels produce the same surface direction at the flywheel

## Units Cheat Sheet

Phoenix Tuner X flywheel tuning is easiest in **RPS**.

- `3300 RPM = 55.0 RPS`
- `3200 RPM = 53.33 RPS`
- `3603 RPM = 60.05 RPS`
- `3800 RPM = 63.33 RPS`
- `4000 RPM = 66.67 RPS`

Code publishes both RPM and RPS:

- `Shooter/FlywheelRPM`
- `Shooter/FlywheelMotorRPS`
- `Shooter/TargetFlywheelRPM`
- `Shooter/FlywheelError`
- `Shooter/FlywheelAppliedVolts`
- `Shooter/IsReady`
- `Shooter/FlywheelAtRPM`

## Starting Values

For your current near-`1:1` dual Kraken X60 flywheel, start here:

```text
Slot0.kS = 0.00
Slot0.kV = 0.14
Slot0.kP = 0.04
Slot0.kD = 0.00
```

Reasonable adjustment range:

- `kV`: `0.13` to `0.15`
- `kP`: `0.02` to `0.08`
- `kD`: `0.00` to `0.005`
- `kS`: `0.00` to `0.10` only if startup is sticky

If you want the absolute safest first spin:

```text
Slot0.kS = 0.00
Slot0.kV = 0.13
Slot0.kP = 0.03
Slot0.kD = 0.00
```

## Phoenix Tuner X 2026 Setup

Open Phoenix Tuner X and connect to device `25`.

Set up these views:

1. `Self-Test Snapshot`
2. `Plotter` or live graph page
3. `Configs` -> `Slot 0`
4. Optional: open device `26` in a second pane just to confirm it follows

Graph these signals on device `25`:

- `Velocity`
- `Closed Loop Reference`
- `Motor Voltage`
- `Closed Loop Error`
- `Stator Current`
- `Supply Current`

Best graph combinations:

1. Velocity + Closed Loop Reference
2. Closed Loop Error + Motor Voltage
3. Stator Current + Supply Current

If Tuner X shows velocity in rotations per second already, leave it there. That is the cleanest view for tuning.

## How To Command The Flywheel While Tuning

Use the robot's existing tune command or a fixed preset while watching Tuner X.

Current code support:

- [ShooterSubsystem.java](/C:/Users/jvanscoyoc/.codex/worktrees/35ba/2026-Rebuilt/src/main/java/frc/robot/subsystems/shooter/ShooterSubsystem.java#L499) has `tuneFlywheelCommand(double rpm)`
- That command sends a fixed `VelocityVoltage` setpoint with no indexer and no hood motion

Recommended first target:

- `3300 RPM`
- which is `55.0 RPS`

Hold the tuning button long enough to see:

- spin-up
- settle
- hold for at least 3 more seconds

## Live Tuning Order

Tune in this order:

1. `kV`
2. `kP`
3. `kD` only if needed
4. `kS` only if low-speed startup is sticky

Do not start with `kI`.

## Phase 1: Tune kV

Start with:

```text
kS = 0.00
kV = 0.14
kP = 0.00
kD = 0.00
```

Command `55 RPS`.

Watch these graphs:

- Velocity vs Closed Loop Reference
- Motor Voltage
- Closed Loop Error

What you want:

- actual velocity rises close to the reference
- steady-state error becomes small
- voltage does not sit pinned at 12 V

Adjustments:

- actual settles low: increase `kV` by `0.005`
- actual settles high: decrease `kV` by `0.005`
- voltage stays near 12 V the whole time: target may be too high, or friction/current limit is the issue

Exit Phase 1 when:

- steady-state error is about `+/- 2 RPS` or better
- velocity trace sits close to reference without PID help

For this flywheel, expect final `kV` to likely land around:

```text
0.135 to 0.145
```

## Phase 2: Tune kP

Lock in the `kV` you found, then start with:

```text
kP = 0.04
```

Keep the same `55 RPS` target.

Watch:

- Velocity vs Closed Loop Reference
- Closed Loop Error
- Motor Voltage

What you want:

- faster convergence to target
- smaller final error
- no repeated overshoot or hunting

Adjustments:

- sluggish correction, error lingers: increase `kP` by `0.01`
- visible oscillation or hunting: decrease `kP` by `20%`
- noisy correction but no real gain in recovery: back `kP` down

Exit Phase 2 when:

- it reaches target quickly
- it holds within about `+/- 1 RPS` to `+/- 1.5 RPS`
- velocity trace does not rhythmically cross back and forth over the reference

For this flywheel, expect final `kP` to likely land around:

```text
0.03 to 0.06
```

## Phase 3: Only Add kD If You Actually Need It

Start with:

```text
kD = 0.00
```

Only try `kD` if:

- the flywheel overshoots after `kP` is high enough
- or the trace rings once or twice on each step

Adjustment:

- increase `kD` in very small steps: `0.001`

Stop if:

- response gets slower
- noise increases
- voltage becomes twitchy with no benefit

Most flywheels like this are fine with `kD = 0`.

## Phase 4: Add kS Only If Startup Is Sticky

If the flywheel starts cleanly, leave `kS = 0`.

Only add `kS` if:

- low-speed tests stall before motion starts
- the graph shows a deadband before the wheel breaks loose

Start with:

```text
kS = 0.05
```

Then:

- if it still hesitates from rest, increase by `0.02`
- if it jumps too hard at startup, reduce it

For your mechanism, low friction means `kS = 0` is still the most likely answer.

## What Good Graphs Look Like

Velocity vs Closed Loop Reference:

- actual rises smoothly
- actual approaches reference quickly
- little or no overshoot
- no repeating wave pattern

Closed Loop Error:

- large error at startup is normal
- error should decay toward zero quickly
- error should not alternate positive/negative repeatedly after settling

Motor Voltage:

- rises strongly during acceleration
- drops and stabilizes once at speed
- should usually settle well under 12 V at your normal presets

Current:

- brief acceleration spike is normal
- stable current at speed is normal
- repeated current pulsing often means the loop is too aggressive or mechanically unhappy

## Quick Symptom Guide

| Symptom | Likely Cause | Fix |
|---|---|---|
| Settles below target with `kP = 0` | `kV` too low | Increase `kV` |
| Settles above target with `kP = 0` | `kV` too high | Decrease `kV` |
| Reaches speed slowly but smoothly | `kP` too low | Increase `kP` a little |
| Overshoots and hunts | `kP` too high | Reduce `kP` |
| One or two rings after step | maybe needs a tiny `kD` | Add `0.001` `kD` |
| Won't start smoothly from very low speed | needs static help | Add a little `kS` |
| Voltage pinned near 12 V | saturated | lower target, reduce drag, or revisit mechanical setup |

## Validate At Real Presets

After `55 RPS` looks good, validate the real shooter presets from [Constants.java](/C:/Users/jvanscoyoc/.codex/worktrees/35ba/2026-Rebuilt/src/main/java/frc/robot/Constants.java#L260):

- `POPPER_RPM = 650` -> `10.83 RPS`
- `STANDBY_RPM = 1000` -> `16.67 RPS`
- `CLOSE_RPM = 3603` -> `60.05 RPS`
- `TOWER_RPM = 3200` -> `53.33 RPS`
- `TRENCH_RPM = 3200` -> `53.33 RPS`
- `FAR_RPM = 3800` -> `63.33 RPS`
- `PASS_RPM = 3603` -> `60.05 RPS`

Check at minimum:

1. `53.33 RPS`
2. `55.0 RPS`
3. `60.05 RPS`
4. `63.33 RPS`

You want the same behavior at all of them:

- no saturation
- no hunting
- recovery after a shot is quick

## Cross-Check With NetworkTables / Elastic

While Phoenix Tuner X is your live tuning tool, verify robot behavior with these dashboard values:

- `Shooter/FlywheelRPM`
- `Shooter/FlywheelMotorRPS`
- `Shooter/TargetFlywheelRPM`
- `Shooter/FlywheelError`
- `Shooter/FlywheelAppliedVolts`
- `Shooter/FlywheelAtRPM`
- `Shooter/IsReady`

Key checks:

- `FlywheelMotorRPS` should match the Tuner X velocity trend
- `FlywheelAppliedVolts` should roughly match Tuner X motor voltage
- `IsReady` should become true only after the flywheel is genuinely settled

## After Live Tuning: Copy Gains Back Into Code

Phoenix Tuner X changes are not your long-term source of truth. After you find final values, copy them into [Constants.java](/C:/Users/jvanscoyoc/.codex/worktrees/35ba/2026-Rebuilt/src/main/java/frc/robot/Constants.java#L277).

Update:

```java
public static final double KP = ...;
public static final double KV = ...;
public static final double KD = ...;
```

Those values are applied here:

- [ShooterIOHardware.java](/C:/Users/jvanscoyoc/.codex/worktrees/35ba/2026-Rebuilt/src/main/java/frc/robot/subsystems/shooter/ShooterIOHardware.java#L69)

Then redeploy and verify the robot still matches what you saw in Tuner X.

## Recommended Log Sheet

Record at least this much:

```text
Date:
Battery:
Wheel installed:
Flywheel leader ID: 25
Flywheel follower ID: 26

Target test point 1: 55.0 RPS
Final kS:
Final kV:
Final kP:
Final kD:

53.33 RPS result:
55.0 RPS result:
60.05 RPS result:
63.33 RPS result:

Shot recovery notes:
Ready timing notes:
```

## Done Criteria

- Device `25` tracks velocity cleanly in Tuner X
- Device `26` follows correctly and does not get independently commanded
- actual velocity settles close to reference across the full shot range
- no sustained oscillation in the velocity graph
- voltage is not saturating at normal shot presets
- `Shooter/IsReady` becomes true reliably after spin-up
- final gains are copied back into `Constants.java`

See also:

- [flywheel-ramp-test.md](/C:/Users/jvanscoyoc/.codex/worktrees/35ba/2026-Rebuilt/src/main/java/frc/robot/docs/flywheel-ramp-test.md)
- [tuning-guide_vision.md](/C:/Users/jvanscoyoc/.codex/worktrees/35ba/2026-Rebuilt/src/main/java/frc/robot/docs/tuning-guide_vision.md)
