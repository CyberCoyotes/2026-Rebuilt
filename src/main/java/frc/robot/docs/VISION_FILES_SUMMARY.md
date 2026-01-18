# Vision Subsystem Files Summary

## Files Created/Modified

### ✅ Core Vision Files

#### 1. **VisionIO.java** - Interface
- **Purpose:** Defines the contract for vision hardware
- **Key components:**
  - `updateInputs()` - Get latest data from camera
  - `setPipeline()` - Switch camera pipeline
  - `setLEDMode()` - Control camera LEDs
  - `VisionIOInputs` class - Container for all vision data with AdvantageKit logging
  - `LEDMode` enum - LED control options
- **Why it exists:** Allows swapping hardware implementations without changing VisionSubsystem

#### 2. **VisionIOLimelight.java** - Limelight Implementation
- **Purpose:** Talks to real Limelight 4 hardware via NetworkTables
- **Key features:**
  - Caches NetworkTable entries for performance
  - Accurate timestamp calculation (accounts for latency)
  - Converts angles to radians for consistency
  - Thread-safe (synchronized methods)
  - Defensive copying of arrays
- **NetworkTables it reads:**
  - `tv` - Target valid (0 or 1)
  - `tx` - Horizontal offset (degrees)
  - `ty` - Vertical offset (degrees)
  - `ta` - Target area (%)
  - `tid` - AprilTag ID
  - `botpose` - Robot pose from AprilTag
  - `tl` - Pipeline latency
  - `cl` - Capture latency

#### 3. **VisionIOSim.java** - Simulation Implementation
- **Purpose:** Provides fake vision data for testing without hardware
- **Use cases:**
  - Testing shooter logic at home
  - Unit testing commands
  - Simulation mode
- **Methods for testing:**
  - `setSimulatedTarget(distance, angle, tagId)` - Simulate seeing target
  - `clearTarget()` - Simulate losing target
  - `setTargetVisible(boolean)` - Show/hide target

#### 4. **VisionSubsystem.java** - Main Subsystem
- **Purpose:** Manages vision processing, state machine, and data calculations
- **State machine:** `AlignmentState` enum
  - `NO_TARGET` - No valid AprilTag visible
  - `TARGET_ACQUIRED` - Tag visible, not aligned
  - `ALIGNED` - Robot aligned within tolerance
  - `LOST_TARGET` - Had target, using last known values (0.5s grace period)

- **Public API - State Queries:**
  - `hasTarget()` - Is valid target visible?
  - `isAligned()` - Is robot aligned and ready to shoot?
  - `getAlignmentState()` - Get current state
  - `getTagId()` - Get AprilTag ID being tracked

- **Public API - For Shooter:**
  - `getDistanceToTargetMeters()` - Distance calculation for flywheel/hood
  - `getDistanceToTargetCM()` - Same but in centimeters

- **Public API - For Drivetrain:**
  - `getHorizontalAngleDegrees()` - Angle for rotational alignment
  - `getHorizontalAngleRadians()` - Same but in radians

- **Features:**
  - Automatic validity checking (tag ID, distance, area)
  - Last known value tracking (smooth recovery from target loss)
  - Comprehensive telemetry (SmartDashboard + AdvantageKit)
  - State transition logging

### ✅ Constants

#### 5. **Constants.java** - Vision Section Added
- **Camera configuration:**
  - `LIMELIGHT_NAME` - NetworkTables name
  - `CAMERA_HEIGHT_METERS` - 🔴 **MUST MEASURE**
  - `CAMERA_ANGLE_DEGREES` - 🔴 **MUST MEASURE**

- **Target configuration:**
  - `APRILTAG_HEIGHT_METERS` - 🔴 **UPDATE FOR 2026 GAME**
  - `MIN_VALID_TAG_ID` / `MAX_VALID_TAG_ID` - 🔴 **UPDATE FOR 2026 FIELD**

- **Tuning parameters:**
  - `ALIGNMENT_TOLERANCE_DEGREES` - How close = aligned?
  - `MIN_TARGET_AREA_PERCENT` - Minimum valid target size
  - `MAX_DISTANCE_METERS` - Maximum trusted distance
  - `TARGET_TIMEOUT_SECONDS` - Grace period for lost targets

### ✅ Documentation

#### 6. **VISION_GUIDE.md** - Comprehensive Guide
- Architecture explanation (IO pattern)
- State machine documentation
- Integration examples (shooter, drivetrain, commands)
- Constants to measure
- Testing without robot
- Troubleshooting guide
- Next steps checklist

#### 7. **VISION_FILES_SUMMARY.md** - This file
- Quick reference of all files
- What each file does
- Key methods and features

---

## File Dependencies

```
VisionIO (interface)
    ↑ implements
    |
    ├── VisionIOLimelight (real hardware)
    └── VisionIOSim (testing/simulation)
         ↑ uses
         |
    VisionSubsystem (main subsystem)
         ↑ uses
         |
    Constants.Vision (configuration)
```

---

## Files Deleted

- ❌ **VisionSim.java** - Removed (replaced by VisionIOSim.java)
- ❌ **LimelightHelpers.java** - Removed (no longer needed, using NetworkTables directly)

---

## What Changed from BabyTaz

### Improvements:
1. **Cleaner IO pattern** - Proper interface/implementation separation
2. **Simpler state machine** - 4 states instead of 7+ overlapping states
3. **Better documentation** - Extensive javadocs and markdown guides
4. **AdvantageKit integration** - Full logging support via LoggableInputs
5. **No LimelightHelpers dependency** - Direct NetworkTables control
6. **Last known values** - Smooth handling of target loss
7. **Target validation** - Checks tag ID, distance, area before trusting
8. **Focused on GOAL 1** - Stationary shooting (not over-designed for future features)

### Removed complexity:
- Multiple vision modes (IDLE, ROTATION_ONLY, PERPENDICULAR, etc.)
- Complex alignment states (SEARCHING, HUNTING, SEEKING vs ALIGNING)
- LED subsystem integration (can add back if needed)
- Unnecessary state transitions

---

## Quick Start Checklist

### Before First Use:

1. **Measure robot:**
   - [ ] Camera height from floor → `CAMERA_HEIGHT_METERS`
   - [ ] Camera angle from horizontal → `CAMERA_ANGLE_DEGREES`

2. **Update for 2026 game:**
   - [ ] AprilTag height from manual → `APRILTAG_HEIGHT_METERS`
   - [ ] Max tag ID on field → `MAX_VALID_TAG_ID`

3. **Configure Limelight:**
   - [ ] Set pipeline 0 to AprilTag detection
   - [ ] Verify NetworkTables name matches `LIMELIGHT_NAME`

4. **Test:**
   - [ ] Place robot at known distance (e.g., 2.0m measured)
   - [ ] Check `Vision/Distance_m` on SmartDashboard
   - [ ] Adjust camera angle constant if needed

### To Use in RobotContainer:

```java
// Real robot
vision = new VisionSubsystem(
    new VisionIOLimelight(Constants.Vision.LIMELIGHT_NAME)
);

// Simulation/testing
vision = new VisionSubsystem(new VisionIOSim());
```

### To Use in Shooter:

```java
public void updateFromVision(VisionSubsystem vision) {
    if (vision.hasTarget()) {
        double distance = vision.getDistanceToTargetMeters();
        targetFlywheelRPM = calculateRPM(distance);
        targetHoodAngle = calculateAngle(distance);
    }
}
```

### To Use in Alignment Command:

```java
@Override
public void execute() {
    double angleError = vision.getHorizontalAngleDegrees();
    double rotation = angleError * kP;
    drivetrain.rotate(rotation);
}

@Override
public boolean isFinished() {
    return vision.isAligned();
}
```

---

## Reference for Integration

**See `VISION_GUIDE.md` for:**
- Complete code examples
- Shooting sequence command
- Troubleshooting
- Path to Goals 2 & 3

**Key concept:**
- Vision provides *data* (distance, angles)
- Vision tracks *state* (aligned or not)
- Commands *use* that data to control robot
- Vision does NOT control other subsystems directly

---

## Support

All files include:
- ✅ Extensive javadoc comments
- ✅ Inline explanations
- ✅ Usage examples in comments
- ✅ Clear variable names
- ✅ Organized by section

If something is unclear, check:
1. The javadoc in the file
2. VISION_GUIDE.md
3. The code examples in comments
