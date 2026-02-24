# Vision Shooter Tuning Guide

This document covers everything needed to tune the vision-based shooting system end-to-end:
camera constants → distance verification → shooter lookup table → rotational alignment.

Work through the sections in order. Each section depends on the previous one being correct.

---

## § 1  Pre-Session: One-Time Physical Measurements

Measure these before any on-robot tuning. Get them right once and you will not need to revisit them unless the camera mount changes.

| Measurement | Location in code | How to measure |
|---|---|---|
| `CAMERA_HEIGHT_METERS` | `Constants.Vision` | Floor to the center of the Limelight 4 lens, robot on a level surface |
| `CAMERA_ANGLE_DEGREES` | `Constants.Vision` | Angle of camera from horizontal, positive = tilted up. Use a digital angle gauge or protractor against the camera face |
| `APRILTAG_HEIGHT_METERS` | `Constants.Vision` | Floor to center of the hub AprilTag face. Look this up in the 2026 game manual field drawings |

Also verify in Limelight web UI that the camera pose is configured to match physical mounting (used by MegaTag2 odometry). These values are separate from the robot code constants but must reflect the same physical reality.

---

## § 2  Hub AprilTag IDs

**Before any testing, verify these against the 2026 game manual and the WPILib field layout JSON.**

The WPILib field layout JSON is included with the WPILib release (usually `2026-crescendo.json` or equivalent) and lists every tag ID with its field position.

| Alliance | Constant | Current placeholder |
|---|---|---|
| Blue hub | `BLUE_HUB_MIN/MAX_TAG_ID` | 23 – 28 |
| Red hub  | `RED_HUB_MIN/MAX_TAG_ID`  | 1 – 6 (NOT verified) |

Wrong IDs → `isHubTarget()` returns false → vision targeting never activates. Verify first.

---

## § 3  Distance Verification

Before filling in the shooter lookup table you need to confirm that `Vision/Distance_m` on the Elastic dashboard matches the real floor distance to the hub.

**Procedure:**

1. Place the robot on a level floor, directly facing a hub AprilTag.
2. Use a tape measure to record the distance from the bumper (or a consistent reference point on the robot) to the tag face. Write down: `1.0 m`, `2.0 m`, `3.0 m`, `4.0 m`, `5.0 m`, `6.0 m`.
3. Enable Teleop. Watch `Vision/Distance_m` and `Vision/State` on Elastic.
4. For each measured distance, compare `Vision/Distance_m` to the tape measure reading.

**If values do not match:**

The distance formula is:
```
distance = (APRILTAG_HEIGHT_METERS - CAMERA_HEIGHT_METERS) / tan(CAMERA_ANGLE_DEGREES + ty)
```
- Reads too **far**: camera height is understated OR camera angle is overstated.
- Reads too **close**: camera height is overstated OR camera angle is understated.
- Adjust `CAMERA_HEIGHT_METERS` and `CAMERA_ANGLE_DEGREES` in `Constants.Vision` and retest.

**Acceptance criterion:** `Vision/Distance_m` within ±0.1 m of tape measure at all tested distances before proceeding.

**Also verify:**
- `Vision/State` shows `TARGET_ACQUIRED` or `ALIGNED` (not `NO_TARGET`).
- `Vision/TagID` shows the expected hub tag number.
- `Vision/IsHubTarget` (add to Elastic if desired, or check via NT Browser) is `true`.

---

## § 4  Shooter Lookup Table Tuning

Fill in `FLYWHEEL_RPM_MAP` and `HOOD_ROT_MAP` in `ShooterSubsystem.java`.

**Setup:**
- Level floor, robot facing hub tag directly (tx ≈ 0°).
- Balls/fuel loaded in hopper.
- Operator controller available to nudge RPM/hood between shots (or use POV cycling for preset reference).

**Procedure (repeat at each distance row):**

1. Place robot at the target distance (confirmed via `Vision/Distance_m`).
2. Hold RT. Robot enters `VisionAlignAndShoot` — flywheel spins to the current map value.
3. Watch `Shooter/FlywheelRPM`, `Shooter/TargetFlywheelRPM`, `Shooter/HoodRotations`, `Shooter/TargetHoodRotations` on Elastic.
4. Wait for `Shooter/IsReady = true`. The indexer will feed automatically.
5. Observe where the shot lands:
   - **Short / hits low**: increase RPM and/or decrease hood (lower number = flatter/closer angle).
   - **Long / hits high**: decrease RPM and/or increase hood.
   - **Correct but arc is wrong**: adjust hood angle primarily; adjust RPM secondarily to correct carry.
6. Adjust values in the map and rebuild. Repeat until shots land center target consistently.
7. Mark the row as `// Tuned` in the source.

**Data table — fill in as you go:**

| Distance (m) | RPM (measured) | Hood rot (measured) | Notes |
|---|---|---|---|
| 1.0 | | | |
| 2.0 | | | |
| 3.0 | | | |
| 4.0 | | | |
| 5.0 | | | |
| 6.0 | | | |

**Tips:**
- Tune the endpoints (1.0 m and 6.0 m) first. The interpolation will give you reasonable in-between values to start from.
- The flywheel RPM map changes slowly with distance; the hood angle map changes more aggressively. Expect the hood to do most of the work.
- If shots are consistently biased in one direction across all distances, recheck camera angle or tag height — there may be a systematic distance error.
- Add extra rows at any distance where interpolation produces noticeably poor results. `InterpolatingDoubleTreeMap` handles arbitrary key counts.

---

## § 5  Rotational Alignment (kP) Tuning

Constant: `Constants.Vision.ROTATIONAL_KP` (default: `0.06` rad/s per degree of tx error)

**Objective:** The drivetrain should smoothly center on the hub tag within ~1 second with no oscillation.

**Dashboard values to watch:**
- `Vision/HorizontalAngle_deg` — tx error in degrees
- `Vision/IsAligned` — becomes true when within ±2° (see `ALIGNMENT_TOLERANCE_DEGREES`)
- `Vision/State` — should reach `ALIGNED` before `Shooter/IsReady` for optimal flow

**Procedure:**

1. Place robot ~3 m from hub tag, offset to one side by ~20°.
2. Hold RT. Observe how the robot rotates to center.
3. Watch `Vision/HorizontalAngle_deg` converge toward 0.

| Symptom | Adjustment |
|---|---|
| Rotates but takes 2–3+ seconds to center | Increase `ROTATIONAL_KP` by 0.01 steps |
| Oscillates left/right past center | Decrease `ROTATIONAL_KP` by 0.01 steps |
| Snaps hard to target then overshoots once | Reduce `MAX_ALIGNMENT_ROTATION_RAD_PER_SEC` |
| Barely moves even with large tx error | Check `Vision/HorizontalAngle_deg` is publishing non-zero values |

**Acceptance criterion:**
- Robot centers from 20° offset in under 1.5 seconds.
- No steady-state oscillation.
- `Vision/IsAligned` holds true once centered.

**Tolerance adjustment:**
`ALIGNMENT_TOLERANCE_DEGREES` is currently `2.0°`. This is the threshold for `isAligned()` to return true and for feeding to begin. If shots are landing off-center, tighten this (e.g., `1.5°`). If the robot is spending too much time waiting to align before it can fire, loosen it (e.g., `2.5°`). Always re-check shot accuracy after changing it.

---

## § 6  Combined System Test

Once §3–5 are complete:

1. Position robot at random locations and angles relative to hub, 1–5 m away.
2. Enable Teleop, hold RT.
3. Verify sequence:
   - Flywheel spins up immediately (preset baseline).
   - Robot rotates to center hub tag.
   - `Vision/State` reaches `ALIGNED`.
   - `Shooter/IsReady` becomes true.
   - Indexer fires automatically.
   - Shot lands on target.
4. Drive around while holding RT. Verify:
   - Left stick translation still works freely.
   - Rotation tracks the tag as robot moves.
   - `Shooter/TargetFlywheelRPM` and `Shooter/TargetHoodRotations` update continuously as distance changes.
5. Test LOST_TARGET grace period: block the camera briefly while aligned. Robot should hold last-known targets for 0.5 s (`TARGET_TIMEOUT_SECONDS`) before resetting.

---

## § 7  Elastic Dashboard Recommended Widgets

| NT Key | Widget type | Purpose |
|---|---|---|
| `Vision/State` | Text display | Current alignment state machine state |
| `Vision/Distance_m` | Number bar (0–8) | Real-time distance to hub |
| `Vision/HorizontalAngle_deg` | Number bar (−30 to 30) | tx error — watch during kP tuning |
| `Vision/IsAligned` | Boolean indicator | Green when feeding is permitted |
| `Vision/TagID` | Integer display | Confirm correct hub tag is tracked |
| `Shooter/FlywheelRPM` | Graph | Actual vs. target convergence |
| `Shooter/TargetFlywheelRPM` | Number display | Current interpolated target |
| `Shooter/HoodRotations` | Number display | Actual hood position |
| `Shooter/TargetHoodRotations` | Number display | Current interpolated target |
| `Shooter/IsReady` | Boolean indicator | Green when both flywheel and hood are on-target |
| `Shooter/SelectedPreset` | Text display | Active fallback preset name |

---

## § 8  Quick Reference: Key Constants

| Constant | File | Purpose |
|---|---|---|
| `CAMERA_HEIGHT_METERS` | Constants.Vision | ty-based distance formula |
| `CAMERA_ANGLE_DEGREES` | Constants.Vision | ty-based distance formula |
| `APRILTAG_HEIGHT_METERS` | Constants.Vision | ty-based distance formula |
| `BLUE/RED_HUB_MIN/MAX_TAG_ID` | Constants.Vision | Hub tag filter (must match 2026 game manual) |
| `ALIGNMENT_TOLERANCE_DEGREES` | Constants.Vision | Threshold for isAligned() |
| `TARGET_TIMEOUT_SECONDS` | Constants.Vision | Grace period on LOST_TARGET |
| `ROTATIONAL_KP` | Constants.Vision | Rotation tracking aggressiveness |
| `MAX_ALIGNMENT_ROTATION_RAD_PER_SEC` | Constants.Vision | Rotation rate clamp |
| `FLYWHEEL_RPM_MAP` | ShooterSubsystem | Distance → RPM lookup |
| `HOOD_ROT_MAP` | ShooterSubsystem | Distance → hood position lookup |
