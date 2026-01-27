# Vision Tuning Quick Reference Card

## Setup (5 minutes)

1. **Deploy code** to robot
2. **Open Shuffleboard**
3. **File → Load Layout** → `shuffleboard_vision_tuning.json`
4. **Connect** to robot (10.TE.AM.2)
5. **Place AprilTag** on wall, position robot 1-2m away

## Tuning Process (One Axis)

### Step 1: Set I and D to zero
- `kI = 0.0`
- `kD = 0.0`

### Step 2: Increase kP gradually
- Start: `kP = 0.02`
- Increase by 0.01 each test
- Stop when: slight overshoot or oscillation appears
- Back off 20%: `kP_final = kP_current * 0.8`

### Step 3: Add D if needed
- Start: `kD = kP / 10`
- Increase until overshoot disappears

### Step 4: Add I only if stuck before target
- Start: `kI = 0.001`
- Increase slowly until error eliminates

## Quick Diagnostics

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Oscillates | kP too high | ↓ kP by 20-30% |
| Overshoots | kP too high, kD too low | ↓ kP, ↑ kD |
| Stops short | kP too low | ↑ kP |
| Too slow | kP too low, MaxSpeed too low | ↑ kP, ↑ MaxSpeed |
| Too aggressive | kP too high, MaxSpeed too high | ↓ kP, ↓ MaxSpeed |
| Gets stuck near target (Model A) | MinSpeed too low | ↑ MinSpeed |

## Parameter Locations (SmartDashboard)

**Model A (Rotation Only):**
- `Tuning/Vision/ModelA/Rotation_kP`
- `Tuning/Vision/ModelA/Rotation_kD`
- `Tuning/Vision/ModelA/MinRotationSpeed`
- `Tuning/Vision/ModelA/MaxRotationSpeed`

**Model B (Rotation + Range):**
- Rotation: same as Model A
- Range: `Tuning/Vision/ModelB/Range_kP`, `Range_kD`
- Target: `Tuning/Vision/ModelB/TargetDistance_m`

**Model C (3-Axis):**
- Rotation: same as Model B
- Range: same as Model B
- Lateral: `Tuning/Vision/ModelC/Lateral_kP`, `LateralDeadband_deg`

## Typical Good Values

### Model A (Rotation)
- kP: 0.04 - 0.08
- kD: 0.002 - 0.01
- MinSpeed: 0.15 - 0.30 rad/s
- MaxSpeed: 1.0 - 2.5 rad/s
- Tolerance: 1.0 - 2.0 degrees

### Model B (Range)
- kP: 0.8 - 1.5
- kD: 0.05 - 0.15
- MaxSpeed: 0.5 - 1.0 m/s
- Tolerance: 0.05 - 0.10 m

### Model C (Lateral)
- kP: 0.03 - 0.06
- Deadband: 2.0 - 5.0 degrees

## Test Sequence

For each configuration change:
1. ✓ Test 3-5 times
2. ✓ Vary starting position/angle
3. ✓ Record: time, success, overshoot, feel (1-10)
4. ✓ Compare to previous best

## Saving Final Values

### 1. Update VisionConstants.java
```java
public static final class ModelA {
    public static final double ROTATION_KP = 0.07; // Your tuned value
    public static final double ROTATION_KD = 0.005; // Your tuned value
    // ...
}
```

### 2. Update TunableVisionConstants.java
```java
public static final TunableNumber ROTATION_KP =
    new TunableNumber("Vision/ModelA/Rotation_kP", 0.07); // Match VisionConstants
```

### 3. Commit to git
```bash
git add src/main/java/frc/robot/subsystems/vision/VisionConstants.java
git add src/main/java/frc/robot/subsystems/vision/TunableVisionConstants.java
git commit -m "Update Model A PID values after tuning session"
```

## Competition Mode

**Disable tuning before competition:**

In `Robot.java`:
```java
@Override
public void robotInit() {
    TunableNumber.setTuningEnabled(false);
    // ...
}
```

## D-Pad Tuning (No Laptop Needed)

Add to `RobotContainer.java`:
```java
// Up/Down: kP ±0.005
driverController.povUp().onTrue(Commands.runOnce(() -> {
    double kp = TunableVisionConstants.ModelA.ROTATION_KP.get();
    TunableVisionConstants.ModelA.ROTATION_KP.set(kp + 0.005);
}));

driverController.povDown().onTrue(Commands.runOnce(() -> {
    double kp = TunableVisionConstants.ModelA.ROTATION_KP.get();
    TunableVisionConstants.ModelA.ROTATION_KP.set(kp - 0.005);
}));

// Left/Right: kD ±0.001
driverController.povLeft().onTrue(Commands.runOnce(() -> {
    double kd = TunableVisionConstants.ModelA.ROTATION_KD.get();
    TunableVisionConstants.ModelA.ROTATION_KD.set(kd - 0.001);
}));

driverController.povRight().onTrue(Commands.runOnce(() -> {
    double kd = TunableVisionConstants.ModelA.ROTATION_KD.get();
    TunableVisionConstants.ModelA.ROTATION_KD.set(kd + 0.001);
}));
```

Current values print to Driver Station console.

## Analyze Results

After tuning session:
```bash
python analyze_alignment.py logs/tuning_session.wpilog --model A --plot
```

Check metrics:
- Alignment time < 2.0s ✓
- Final error < tolerance ✓
- Overshoot < 15% ✓
- Oscillations < 3 ✓

## Emergency Reset

Reset all to defaults:
```java
TunableVisionConstants.resetAllToDefaults();
```

Or restart robot code.

---

**Remember**: Start conservative (low gains), make small changes (10-20%), test multiple times, record everything!
