# Vision Tuning Guide (MegaTag2 + PoseAlignAndShoot)

This guide is updated for the current architecture:

- **Aim command:** `FuelCommands.poseAlignAndShoot(...)` (odometry + hub pose aiming)
- **Vision localization:** Limelight **MegaTag2** fused into drivetrain pose in `Robot.robotPeriodic()`
- **Shooter setpoints:** `ShooterSubsystem.updateFromDistance(distance)` from field pose distance to hub

Because your mechanical setup and camera orientation changed, you should tune in two places:

1. **Limelight configuration** (camera pose + pipeline + field map) → foundation
2. **Robot code constants** (alignment gains, tolerance, hub location, shooter map) → behavior

If Limelight pose is wrong, code tuning will never feel stable.

---

## Quick Answer: How much is Limelight vs code?

For a new camera mount/orientation, expect roughly:

- **60–70% Limelight setup** (camera pose, orientation correctness, pipeline/tag settings)
- **30–40% code tuning** (`ROTATIONAL_KP`, alignment tolerance, shooter distance map)

The order matters more than the percentage:

1. Fix Limelight geometry first.
2. Verify fused robot pose quality.
3. Tune code gains and shooter map last.

---

## §1 What is tuned in Limelight (not code)

Do this in Limelight web UI first.

## 1) Camera Pose (Robot Space)

Set the Limelight **camera pose relative to robot center** (forward, side, up, roll, pitch, yaw) to match your new mechanical mount.

If yaw/roll/pitch are wrong, MegaTag2 pose will be biased and `poseAlignAndShoot` will aim incorrectly even if `ROTATIONAL_KP` is perfect.

## 2) AprilTag pipeline and map

- Use your AprilTag pipeline (code expects `APRILTAG_PIPELINE = 0`).
- Verify field layout/year is correct on Limelight.
- Verify the camera is actually tracking hub-side tags with good consistency.

## 3) Basic quality checks on Limelight dashboard

With robot still + level:

- Botpose heading should match gyro heading directionally.
- Translation should not jump when stationary.
- Tag count should be > 0 when a hub tag is visible.

If these fail, do **not** tune code yet.

---

## §2 What is tuned in code

After Limelight pose is trustworthy, tune these in code:

### High-priority constants

- `Constants.Vision.ROTATIONAL_KP`
- `Constants.Vision.MIN_ALIGNMENT_ROTATION_RAD_PER_SEC`
- `Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC`
- `Constants.Vision.ALIGNMENT_TOLERANCE_DEGREES`
- `Constants.Vision.BLUE_HUB_LOCATION` / `RED_HUB_LOCATION` (verify field coordinates)
- Shooter distance maps in `ShooterSubsystem` (`updateFromDistance` source table)

### Lower-priority / optional

- `Constants.Vision.LEAD_COMPENSATION_DEG_PER_MPS` (only if shooting while translating fast)

---

## §3 Important architecture note (why old ty tuning is less critical now)

Current aim flow does **not** use `ty` trig distance for shooting. It uses:

1. Limelight MegaTag2 → robot `Pose2d`
2. Drivetrain pose estimator fusion
3. Hub field coordinate distance (`hypot(dx,dy)`) in `poseAlignAndShoot`
4. Shooter lookup from that distance

That means camera height/angle (`CAMERA_HEIGHT_METERS`, `CAMERA_ANGLE_DEGREES`) are no longer the primary distance-tuning knobs for the active shoot command.

Keep them documented, but prioritize Limelight camera pose + fused odometry correctness instead.

---

## §4 Bring-up sequence for a new mechanical orientation

Run this in order.

### Step A — Mechanical/geometry verification (Limelight)

1. Measure camera offsets from robot center (forward/left/up) and mount angles (roll/pitch/yaw).
2. Enter/update Limelight camera pose (Robot Space).
3. Confirm AprilTag pipeline and field map/year.
4. Reboot Limelight and verify botpose is stable when stationary.

### Step B — Fusion sanity check (robot code + NT)

1. Put robot on carpet, point at known field landmark.
2. Compare drivetrain field pose with expected location.
3. Drive 2–3 m and return; check drift and correction behavior.
4. Ensure vision updates are actually accepted (tagCount > 0, not spinning too fast).

### Step C — Align controller tune (`ROTATIONAL_KP`)

At ~3–5 m from hub:

1. Start ~15–25° off heading.
2. Hold aim/shoot trigger.
3. Watch `VisionShoot/headingError_deg` and `VisionShoot/rotRate_radps`.

Adjust:

- Slow to converge → increase `ROTATIONAL_KP` by small steps (e.g., +0.002 to +0.005)
- Oscillation/hunting → decrease `ROTATIONAL_KP`
- Stiction near zero error → raise `MIN_ALIGNMENT_ROTATION_RAD_PER_SEC` slightly
- Too aggressive snap → lower `MAX_ALIGNMENT_ROTATION_RAD_PER_SEC`

Target behavior: quick settle without bouncing.

### Step D — Feed/ready threshold

Tune `ALIGNMENT_TOLERANCE_DEGREES` to balance speed vs precision:

- Too strict: robot waits forever to feed
- Too loose: early feed while still mis-aimed

Start with current value and adjust in small increments (0.25–0.5°).

### Step E — Shooter distance map

Now tune shot results by distance (because distance comes from pose now):

1. Test fixed distances (e.g., 2 m, 3.5 m, 5 m, 6 m).
2. Adjust flywheel RPM + hood map entries.
3. Add extra interpolation points where miss pattern changes rapidly.

---

## §5 Symptoms → likely fix location

| Symptom | Usually fix in Limelight | Usually fix in code |
|---|---|---|
| Robot aims to consistently wrong angle at many spots | ✅ Camera pose yaw/roll/pitch, field map | ✅ (sometimes) hub coordinates/offset constants |
| Pose jumps while stationary | ✅ Pipeline/tag quality, camera pose, exposure/setup | ❌ |
| Robot rotates too slowly/too fast around target | ❌ | ✅ `ROTATIONAL_KP`, min/max rot clamps |
| Robot says aligned but misses left/right slightly | ⚠️ maybe pose bias | ✅ `ALIGNMENT_TOLERANCE_DEGREES`, shooter map |
| Shots are high/low by distance | ❌ | ✅ shooter RPM/hood lookup table |
| Misses only while driving sideways | ❌ | ✅ `LEAD_COMPENSATION_DEG_PER_MPS` |

---

## §6 Current-code references (for pit debugging)

- MegaTag2 fusion path: `Robot.robotPeriodic()`
  - sets robot orientation to Limelight
  - reads `getBotPoseEstimate_wpiBlue_MegaTag2(...)`
  - adds vision measurement to drivetrain estimator with distance-scaled XY std dev
- Aim + shoot command: `FuelCommands.poseAlignAndShoot(...)`
  - computes hub distance from drivetrain pose
  - computes heading error and applies `ROTATIONAL_KP` + rate clamps
  - only feeds when shooter is ready
- Vision constants: `Constants.Vision`

---

## §7 Practical recommendation for your current situation

Since you said you're currently using:

- vision align
- pose-command 3D pose
- MegaTag2

with a new mechanical setup/orientation:

1. Spend your first session almost entirely on **Limelight camera pose correctness** and pose sanity.
2. Spend second session on **`ROTATIONAL_KP` + alignment tolerance**.
3. Spend third session on **distance-to-shot map polish**.

If you skip step 1, you'll keep “retuning” code to compensate for a geometry error and it will break again in different field positions.
