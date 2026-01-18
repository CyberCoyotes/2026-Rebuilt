# Vision Subsystem Guide

## Overview

This vision subsystem is designed for **GOAL 1: Stationary shooting with vision alignment** using a Limelight 4 (LL4) camera with AprilTags.

### What it does:
1. **Tracks AprilTags** - Detects and identifies field AprilTags
2. **Calculates distance** - Uses camera geometry to determine distance to target
3. **Provides alignment data** - Gives horizontal angle for drivetrain rotation
4. **Manages state** - Tracks whether target is visible, acquired, or aligned
5. **Supports shooter** - Provides distance/angle data for flywheel and hood adjustments

---

## File Structure

```
src/main/java/frc/robot/
├── subsystems/vision/
│   ├── VisionIO.java              # Interface - defines what vision hardware can do
│   ├── VisionIOLimelight.java     # Implementation - talks to real Limelight
│   ├── VisionIOSim.java            # Simulation - for testing without hardware
│   └── VisionSubsystem.java        # Subsystem - uses IO, manages state, provides data
└── Constants.java                  # Contains Vision constants
```

---

## Architecture Pattern: IO Interface

### Why use this pattern?

**Problem:** If we write code that talks directly to the Limelight, we can't test without hardware.

**Solution:** Create an interface (VisionIO) that defines what vision hardware *can do*, then create implementations:
- `VisionIOLimelight` - Real hardware (talks to Limelight via NetworkTables)
- `VisionIOSim` - Fake data for testing at home

**Benefit:** VisionSubsystem doesn't care which implementation it gets - it just calls the interface methods.

### How it works:

```java
// VisionIO - Interface (contract)
public interface VisionIO {
    void updateInputs(VisionIOInputs inputs);  // "Get me the latest data"
}

// VisionIOLimelight - Real implementation
public class VisionIOLimelight implements VisionIO {
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Read from NetworkTables
        inputs.hasTargets = limelightTable.getEntry("tv").getDouble(0.0) > 0.5;
        // ... more NetworkTables reads
    }
}

// VisionSubsystem - Doesn't know about hardware
public class VisionSubsystem {
    private final VisionIO io;  // Could be Limelight OR Sim

    public void periodic() {
        io.updateInputs(inputs);  // Works with any implementation!
    }
}
```

---

## State Machine

### AlignmentState Enum

| State | Meaning | When it happens |
|-------|---------|----------------|
| **NO_TARGET** | No valid AprilTag visible | Camera doesn't see any tags |
| **TARGET_ACQUIRED** | Tag visible, not aligned yet | Camera sees tag, robot not rotated correctly |
| **ALIGNED** | Robot aligned within tolerance | Horizontal angle error < 2° |
| **LOST_TARGET** | Had target but lost it (grace period) | Target lost, using last known values for 0.5s |

### State Transitions

```
NO_TARGET
    ↓ (tag becomes visible)
TARGET_ACQUIRED
    ↓ (robot rotates, error < tolerance)
ALIGNED
    ↓ (tag lost)
LOST_TARGET (grace period for 0.5s)
    ↓ (timeout)
NO_TARGET
```

### Why LOST_TARGET state?

When a target disappears briefly (like when the robot bumps), we don't want to immediately lose all data. LOST_TARGET gives us a 0.5 second grace period where we still return the last known distance/angle. This makes commands smoother and more reliable.

---

## Key Methods

### State Queries

```java
// Check if we have a valid target
boolean hasTarget()  // true if TARGET_ACQUIRED or ALIGNED

// Check if aligned and ready to shoot
boolean isAligned()  // true only if ALIGNED

// Get current state
AlignmentState getAlignmentState()

// Get the tag we're looking at
int getTagId()  // -1 if no target
```

### For Shooter (distance calculation)

```java
// Get distance to target for flywheel/hood calculations
double getDistanceToTargetMeters()  // In meters
double getDistanceToTargetCM()      // In centimeters (legacy)
```

**How distance is calculated:**
```
                    ^
                   /|
                  / |
    distance     /  | heightDiff
                /   |
               /    |
              /angle|
robot -------/------+------ floor
            camera  tag
```

Formula: `distance = (tagHeight - cameraHeight) / tan(cameraAngle + ty)`

**Important:** You MUST measure and set these constants in `Constants.java`:
- `CAMERA_HEIGHT_METERS` - Height of Limelight lens from floor
- `CAMERA_ANGLE_DEGREES` - Angle camera is tilted (positive = up)
- `APRILTAG_HEIGHT_METERS` - Height of AprilTag center (from game manual)

### For Drivetrain (alignment)

```java
// Get horizontal angle to target for rotation
double getHorizontalAngleDegrees()  // + = target right, - = target left
double getHorizontalAngleRadians()  // Same but in radians
```

**Usage in alignment command:**
```java
double angleError = vision.getHorizontalAngleDegrees();
double rotationSpeed = angleError * kP;  // Proportional control
drivetrain.rotate(rotationSpeed);
```

---

## Integration Examples

### Example 1: Create VisionSubsystem in RobotContainer

```java
public class RobotContainer {
    // Subsystems
    private final VisionSubsystem vision;

    public RobotContainer() {
        // Create vision with real Limelight
        vision = new VisionSubsystem(
            new VisionIOLimelight(Constants.Vision.LIMELIGHT_NAME)
        );

        // Or for simulation/testing:
        // vision = new VisionSubsystem(new VisionIOSim());
    }
}
```

### Example 2: Use in Shooter

```java
public class ShooterSubsystem extends SubsystemBase {

    /**
     * Updates shooter targets based on vision data.
     * Call this before spinning up for a shot.
     */
    public void updateFromVision(VisionSubsystem vision) {
        if (vision.hasTarget()) {
            // Get distance from vision
            double distanceMeters = vision.getDistanceToTargetMeters();

            // Calculate required velocity and hood angle
            targetFlywheelRPM = calculateFlywheelRPM(distanceMeters);
            targetHoodAngle = calculateHoodAngle(distanceMeters);
        }
        // If no target, keep last known values or use defaults
    }

    // Example calculation (you'll tune these!)
    private double calculateFlywheelRPM(double distance) {
        // Linear interpolation example
        // Close shot (1m) = 3000 RPM
        // Far shot (5m) = 5000 RPM
        return 3000 + (distance - 1.0) * 500;
    }

    private double calculateHoodAngle(double distance) {
        // Linear interpolation example
        // Close shot = 25 degrees
        // Far shot = 45 degrees
        return 25 + (distance - 1.0) * 5;
    }
}
```

### Example 3: Alignment Command

```java
public class AlignToTargetCommand extends Command {
    private final DriveSubsystem drive;
    private final VisionSubsystem vision;

    public AlignToTargetCommand(DriveSubsystem drive, VisionSubsystem vision) {
        this.drive = drive;
        this.vision = vision;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (!vision.hasTarget()) {
            drive.stop();
            return;
        }

        // Proportional control
        double angleError = vision.getHorizontalAngleDegrees();
        double rotationSpeed = angleError * 0.05;  // kP = 0.05 (tune this!)

        // Limit rotation speed
        rotationSpeed = Math.max(-0.5, Math.min(0.5, rotationSpeed));

        drive.arcadeDrive(0, rotationSpeed);  // Only rotate, don't drive
    }

    @Override
    public boolean isFinished() {
        return vision.isAligned();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
```

### Example 4: Complete Shooting Sequence

```java
public class ShootCommand extends SequentialCommandGroup {

    public ShootCommand(
            ShooterSubsystem shooter,
            IndexerSubsystem indexer,
            VisionSubsystem vision,
            DriveSubsystem drive) {

        addCommands(
            // 1. Update shooter targets from vision
            new InstantCommand(() -> shooter.updateFromVision(vision)),

            // 2. Spin up shooter
            new InstantCommand(() -> shooter.setState(ShooterState.SPINUP)),

            // 3. Align to target (in parallel with spinup)
            new ParallelDeadlineGroup(
                new WaitUntilCommand(() ->
                    shooter.isReady() && vision.isAligned() && indexer.hasGamePiece()
                ),
                new AlignToTargetCommand(drive, vision)
            ),

            // 4. Shoot!
            new InstantCommand(() -> indexer.setState(IndexerState.FEEDING)),

            // 5. Wait for game piece to leave
            new WaitUntilCommand(() -> !indexer.hasGamePiece()),

            // 6. Stop everything
            new InstantCommand(() -> {
                shooter.setState(ShooterState.IDLE);
                indexer.setState(IndexerState.IDLE);
            })
        );
    }
}
```

---

## Constants You Need to Set

In `Constants.java` → `Vision` class:

### Critical (MUST MEASURE):

```java
// Measure these on your robot!
public static final double CAMERA_HEIGHT_METERS = 0.5;     // TODO: Measure
public static final double CAMERA_ANGLE_DEGREES = 25.0;    // TODO: Measure
```

**How to measure:**
1. **Camera height:** Measure from floor to center of Limelight lens
2. **Camera angle:** Use a protractor/level to measure tilt from horizontal

### Game-specific (UPDATE FOR 2026):

```java
public static final double APRILTAG_HEIGHT_METERS = 1.45;  // TODO: Check 2026 manual
public static final int MAX_VALID_TAG_ID = 16;              // TODO: Check 2026 field
```

### Tuning parameters:

```java
// How close to center = "aligned"?
public static final double ALIGNMENT_TOLERANCE_DEGREES = 2.0;  // Start here, tune

// Minimum area to trust target
public static final double MIN_TARGET_AREA_PERCENT = 0.1;  // Prevents far/fake targets

// Maximum distance to trust vision
public static final double MAX_DISTANCE_METERS = 5.0;  // Depends on camera/lighting
```

---

## Testing Without Robot

### Use VisionIOSim

```java
// In RobotContainer
VisionIOSim sim = new VisionIOSim();
VisionSubsystem vision = new VisionSubsystem(sim);

// Simulate seeing a target 3 meters away, 5 degrees to the right
sim.setSimulatedTarget(3.0, 5.0, 7);  // distance, angle, tagID

// Test your shooter logic
shooter.updateFromVision(vision);
// Check: did flywheel/hood set correctly?

// Simulate losing target
sim.clearTarget();
```

---

## AdvantageKit Logging

The vision subsystem automatically logs to AdvantageKit:

### What gets logged:
- All raw inputs (tx, ty, ta, tagID, latency)
- Calculated values (distance, angles)
- State transitions (NO_TARGET → TARGET_ACQUIRED)
- Timestamps

### How to use:
1. **During match:** Everything logs automatically
2. **After match:** Open log file in AdvantageScope
3. **Replay:** See exactly what vision saw during the match
4. **Debug:** "Why did we miss that shot?" → Check vision data at that timestamp

---

## Troubleshooting

### "Vision always says NO_TARGET"

1. Check Limelight is powered and connected
2. Check NetworkTables name matches: `Constants.Vision.LIMELIGHT_NAME`
3. Check Limelight pipeline is configured for AprilTags
4. Use SmartDashboard to see raw `Vision/HasTarget` value

### "Distance calculation is wrong"

1. **Measure camera height and angle** - This is the #1 cause of bad distance
2. Check camera is level (no roll)
3. Verify `APRILTAG_HEIGHT_METERS` from game manual
4. Test at known distance (measure with tape measure)

### "Never reaches ALIGNED state"

1. Check `ALIGNMENT_TOLERANCE_DEGREES` - might be too tight (try 5.0 for testing)
2. Check drivetrain alignment command is working (is robot rotating?)
3. Check vision latency isn't too high (should be < 50ms total)

### "State flickers between TARGET_ACQUIRED and LOST_TARGET"

1. Lighting issues - AprilTag detection struggling
2. Robot vibration - camera shaking
3. Adjust `MIN_TARGET_AREA_PERCENT` or `TARGET_TIMEOUT_SECONDS`

---

## Next Steps

### 1. Measure and configure constants
- [ ] Measure camera height
- [ ] Measure camera angle
- [ ] Get 2026 AprilTag height from game manual
- [ ] Set max valid tag ID for 2026 field

### 2. Test distance calculation
- [ ] Place robot at known distance (measure with tape)
- [ ] Check `Vision/Distance_m` on SmartDashboard
- [ ] Adjust if needed (probably camera angle)

### 3. Create alignment command
- [ ] Write `AlignToTargetCommand` (see example above)
- [ ] Tune kP for smooth rotation
- [ ] Test with driver input disabled

### 4. Integrate with shooter
- [ ] Add `updateFromVision()` to ShooterSubsystem
- [ ] Create velocity/angle calculation functions
- [ ] Test at multiple distances

### 5. Create shooting sequence
- [ ] Combine alignment + spinup + feeding
- [ ] Test stationary shots
- [ ] Tune for consistency

---

## Going Further (GOAL 2/3)

### Goal 2: Shooting while moving

Once Goal 1 works, you can add:
- Continuous vision updates while driving
- Drivetrain "aim assist" mode (vision controls rotation, driver controls translation)
- Predictive aiming (account for robot velocity)

### Goal 3: Turret shooter

Alternative approach:
- Turret subsystem rotates instead of whole robot
- Easier to shoot while driving
- More complex mechanically

**Recommendation:** Master Goal 1 first! A reliable stationary shot is better than an unreliable moving shot.

---

## Questions?

This vision system is designed to be:
- **Simple to understand** - Clear state machine, well-documented
- **Easy to test** - Works without hardware via VisionIOSim
- **Reliable** - Graceful handling of lost targets, validity checks
- **Extensible** - Easy to add Goal 2/3 features later

Focus on Goal 1 first, then iterate!
