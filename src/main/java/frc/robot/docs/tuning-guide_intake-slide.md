# Intake Slide PID And Feedforward Tuning Guide

**For:** CyberCoyotes 4829 - 2026-Rebuilt  
**Goal:** Start from a safe baseline, stop the squealing, and tune the intake slide in Phoenix Tuner X using the same control mode the robot code uses.

## Hardware And Code References

- Slide motor: `Constants.Intake.SLIDE_MOTOR_ID = 22`
- Bus: `Constants.RIO_CANBUS`
- Control mode in code: `MotionMagicVoltage` on Slot 0
- Slow mode in code: `DynamicMotionMagicVoltage`
- Current code gain source: `Constants.Intake.SlideConfig`
- Config application: [IntakeIOHardware.java](/C:/Users/jvanscoyoc/Git/2026-Rebuilt/src/main/java/frc/robot/subsystems/intake/IntakeIOHardware.java)
- Slide setpoints and profiles: [Constants.java](/C:/Users/jvanscoyoc/Git/2026-Rebuilt/src/main/java/frc/robot/Constants.java)
- Slide commands: [IntakeSubsystem.java](/C:/Users/jvanscoyoc/Git/2026-Rebuilt/src/main/java/frc/robot/subsystems/intake/IntakeSubsystem.java)

## What Your Code Is Doing Right Now

Your slide is currently running Motion Magic with:

```text
kP = 2.0
kI = 0.0
kD = 0.0
kS = 0.7
kV = 0.0
kA = 0.0
```

and the fast profile:

```text
Cruise Velocity = 64 rot/s
Acceleration = 64 rot/s^2
Jerk = 0
```

That means the loop is relying mostly on `kP` plus a fairly large static term, with no velocity feedforward. For a slide, that often sounds like:

- squealing near the setpoint
- buzzing while holding
- sharp reversals
- current spikes with not much extra useful motion

## Before Tuning Any Gains

Do these checks first. If one is off, PID tuning will feel fake and frustrating.

1. With robot disabled, move the slide by hand if possible.
2. Check for tight spots, bent rails, chain or belt misalignment, dry bushings, or cable drag.
3. Confirm the encoder position increases in the same direction as commanded extension.
4. Confirm soft limits match the real mechanism.
5. Verify the slide can retract and extend on low manual voltage without harsh binding.

If the mechanism squeals even in simple open-loop voltage tests, fix that before closed-loop tuning.

## What To Tune First

Tune in this order:

1. Motion profile
2. `kS`
3. `kV`
4. `kP`
5. `kD` only if needed
6. `kA` only if needed

Do not start with `kI`.

## Very Important Assumption

This guide assumes your intake slide is mainly horizontal, so it does **not** need a gravity term.

- If the slide is horizontal: `kG = 0`
- If the slide lifts weight vertically: stop and use a gravity-aware tune instead

## Safe First Baseline

Use this as your first clean baseline in Phoenix Tuner X:

```text
Slot0.kS = 0.20
Slot0.kV = 0.12
Slot0.kP = 0.25
Slot0.kD = 0.00
Slot0.kA = 0.00
```

Use a gentler fast profile for early testing:

```text
MotionMagicCruiseVelocity = 20
MotionMagicAcceleration = 30
MotionMagicJerk = 0
```

If the mechanism is still harsh, go even lower:

```text
Cruise = 12
Accel = 20
```

## Why Squealing Usually Happens Here

For this mechanism, squealing usually means one or more of these:

- `kP` is doing too much of the motion work
- `kS` is too high, so the motor keeps kicking hard near zero speed
- the motion profile is too aggressive
- the slide is binding mechanically
- the motor is hunting around the setpoint while in `Coast`

One practical note: while tuning, it is worth testing `Brake` neutral mode once. If the squeal drops a lot in `Brake`, that is a clue the loop is overworking to hold position in `Coast`.

## Phoenix Tuner X Setup

Open device `22` in Phoenix Tuner X.

Set up these views:

1. `Self-Test Snapshot`
2. `Plotter`
3. `Configs` -> `Slot 0`
4. `Configs` -> `Motion Magic`

Graph these signals:

- `Position`
- `Velocity`
- `Closed Loop Reference`
- `Closed Loop Error`
- `Motor Voltage`
- `Stator Current`
- `Supply Current`

Best graph combinations:

1. Position + Closed Loop Reference
2. Velocity + Motor Voltage
3. Closed Loop Error + Stator Current

## How To Command The Slide While Tuning

Use simple moves between known positions:

- `0` rotations -> retracted
- about `19.18` rotations -> home
- about `58` rotations -> extended

For early tuning, do short moves first:

1. `0` -> `19`
2. `19` -> `0`
3. `19` -> `35`
4. `35` -> `19`

Do not start with full-stroke slams.

## Phase 1: Tune The Motion Profile First

Before chasing gains, make the move shape reasonable.

Start with:

```text
Cruise = 20
Accel = 30
Jerk = 0
```

What you want:

- smooth launch
- no violent snap at the start
- no hard smash into the endpoint

Adjustments:

- starts too violently: lower acceleration first
- move is too slow but smooth: raise cruise by `2` to `4`
- reversals feel harsh: lower cruise and acceleration together

Do not push the profile back up until the loop is quiet.

## Phase 2: Tune kS

Set:

```text
kP = 0
kD = 0
kA = 0
```

Keep a small starting `kV`, like `0.10` to `0.12`.

Now focus on breakaway behavior.

What you want:

- slide starts moving cleanly from rest
- no long hesitation before motion
- no jumpy kick when motion begins

Adjustments:

- hesitates before moving: increase `kS` by `0.02`
- jumps too hard from rest: decrease `kS` by `0.02`
- squeals or chatters near zero speed: `kS` may be too high

For a healthy horizontal slide, final `kS` is often smaller than people expect.

## Phase 3: Tune kV

Now set:

```text
kP = 0
kD = 0
kA = 0
```

Leave your chosen `kS` in place.

Run a medium move and watch velocity and voltage.

What you want:

- the mechanism travels at roughly the commanded speed
- the motor voltage looks smooth
- steady-state velocity error is modest before adding `kP`

Adjustments:

- mechanism consistently runs slower than expected: increase `kV` by `0.01`
- mechanism consistently runs faster than expected: decrease `kV` by `0.01`
- response is jerky only at launch, not mid-move: that is more likely `kS` than `kV`

## Phase 4: Add kP

Once `kS` and `kV` are doing most of the work, add a little `kP`.

Start with:

```text
kP = 0.20
```

Then adjust in small steps:

- too lazy to settle: add `0.05`
- one overshoot and then settle: maybe okay
- repeated hunting or squeal near target: reduce `kP` by `20%`

Good signs:

- reaches the position cleanly
- little overshoot
- quiet at the endpoint
- error decays quickly without bouncing

For this kind of slide, a good final `kP` is often much lower than `2.0`.

## Phase 5: Add kD Only If Needed

Only touch `kD` if the slide overshoots after `kP` is otherwise close.

Start with:

```text
kD = 0.005
```

Then:

- if ringing decreases, keep going in tiny steps
- if noise increases or the motor gets twitchy, back it out

Very often, `kD` ends up tiny or zero.

## Phase 6: Add kA Only If Needed

`kA` is optional here.

Use it only if:

- the slide lags during acceleration even after `kV` is decent
- the trace looks fine at constant speed but weak at the start of the ramp

Start with:

```text
kA = 0.01
```

Increase slowly. If you cannot clearly see the benefit, leave it at zero.

## What Good Graphs Look Like

Position:

- clean approach to target
- little overshoot
- no repeated crossing over the setpoint

Velocity:

- smooth ramp up
- smooth ramp down
- no jagged reversal chatter

Motor Voltage:

- brief rise at launch
- calmer voltage once moving
- not bouncing sharply positive/negative near the endpoint

Current:

- launch spike is normal
- repeated pulsing at the setpoint usually means the loop is too aggressive or the mechanism is binding

## Quick Symptom Guide

| Symptom | Likely Cause | Fix |
|---|---|---|
| Squeals near target | `kP` too high or profile too aggressive | lower `kP`, lower accel |
| Jumps hard when starting | `kS` too high | reduce `kS` |
| Barely starts moving | `kS` too low | raise `kS` slightly |
| Moves but feels weak through the stroke | `kV` too low | raise `kV` |
| Overshoots and rings | `kP` too high | reduce `kP`, maybe add tiny `kD` |
| Pulsing current while holding | aggressive loop or mechanical drag | lower `kP`, inspect hardware |
| Works on extension but hates retraction | profile too fast, friction, or load shift | reduce accel, inspect mechanics |

## Recommended Test Order On The Robot

1. Tune with no game piece load first.
2. Tune short moves.
3. Tune full extend and retract.
4. Tune the slow retract profile last.
5. Validate with real intake use and fuel compression.

Once the fast mode is clean, revisit the slow profile:

```text
Slow Cruise = 3 to 6
Slow Accel = 3 to 8
```

The slow profile should feel boring and quiet.

## Copy Final Values Back Into Code

The robot code now has placeholders for slide feedforward gains in `Constants.Intake.SlideConfig`:

```java
public static final double KP = ...;
public static final double KI = 0.0;
public static final double KD = ...;
public static final double KS = ...;
public static final double KV = ...;
public static final double KA = ...;
```

Those values are applied in:

- [IntakeIOHardware.java](/C:/Users/jvanscoyoc/Git/2026-Rebuilt/src/main/java/frc/robot/subsystems/intake/IntakeIOHardware.java)

## Recommended Log Sheet

Record at least:

```text
Date:
Battery voltage:
Mechanism state:
Slide motor ID: 22

Fast profile cruise:
Fast profile accel:

Final kS:
Final kV:
Final kP:
Final kD:
Final kA:

0 -> 19 result:
19 -> 0 result:
19 -> 35 result:
35 -> 19 result:
0 -> 58 result:
58 -> 0 result:

Noise notes:
Current notes:
Mechanical notes:
```

## Done Criteria

- short moves are smooth and quiet
- full-stroke moves do not squeal at the endpoint
- position settles without repeated hunting
- current is not pulsing badly while holding
- slow retract is quieter than fast retract
- final gains are copied back into `Constants.java`

## Related Files

- [IntakeSubsystem.java](/C:/Users/jvanscoyoc/Git/2026-Rebuilt/src/main/java/frc/robot/subsystems/intake/IntakeSubsystem.java)
- [Constants.java](/C:/Users/jvanscoyoc/Git/2026-Rebuilt/src/main/java/frc/robot/Constants.java)
- [IntakeIOHardware.java](/C:/Users/jvanscoyoc/Git/2026-Rebuilt/src/main/java/frc/robot/subsystems/intake/IntakeIOHardware.java)
