# Vision Subsystem - Quick Reference Card

## Most Common Methods (You'll Use These Daily)

### State Checks

```java
// Do we see a valid target?
vision.hasTarget()  // → boolean

// Are we aligned and ready to shoot?
vision.isAligned()  // → boolean

// What state are we in?
vision.getAlignmentState()  // → NO_TARGET, TARGET_ACQUIRED, ALIGNED, LOST_TARGET
```

### For Shooter (Distance)

```java
// Get distance to calculate flywheel/hood
vision.getDistanceToTargetMeters()  // → double (meters)
vision.getDistanceToTargetCM()       // → double (centimeters)
```

### For Drivetrain (Alignment)

```java
// Get angle to rotate robot
vision.getHorizontalAngleDegrees()  // → double (+ = right, - = left)
vision.getHorizontalAngleRadians()  // → double (radians)
```

### Tag Info

```java
// Which tag are we looking at?
vision.getTagId()  // → int (-1 if no target)
```

---

## Typical Usage Patterns

### Pattern 1: Update Shooter from Vision

```java
// In ShooterSubsystem
public void updateFromVision(VisionSubsystem vision) {
    if (vision.hasTarget()) {
        double dist = vision.getDistanceToTargetMeters();
        targetFlywheelRPM = calculateRPM(dist);
        targetHoodAngle = calculateAngle(dist);
    }
}
```

### Pattern 2: Align to Target

```java
// In AlignToTargetCommand
@Override
public void execute() {
    if (!vision.hasTarget()) {
        drive.stop();
        return;
    }
    double angle = vision.getHorizontalAngleDegrees();
    drive.rotate(angle * kP);
}

@Override
public boolean isFinished() {
    return vision.isAligned();
}
```

### Pattern 3: Wait Until Ready to Shoot

```java
// In a command
new WaitUntilCommand(() ->
    vision.isAligned() &&
    shooter.isReady() &&
    indexer.hasGamePiece()
)
```

---

## State Machine Quick Ref

| State | When | What It Means |
|-------|------|---------------|
| `NO_TARGET` | No tag visible | Can't shoot, need to find target |
| `TARGET_ACQUIRED` | Tag visible, not aligned | Aligning... keep rotating |
| `ALIGNED` | Tag visible, within tolerance | READY TO SHOOT! |
| `LOST_TARGET` | Just lost tag (< 0.5s ago) | Using last known values, might recover |

**State Flow:**
```
NO_TARGET → TARGET_ACQUIRED → ALIGNED → LOST_TARGET → NO_TARGET
```

---

## Constants to Tune

In `Constants.java`:

```java
// MUST MEASURE ON ROBOT
CAMERA_HEIGHT_METERS = 0.5;     // Measure from floor to lens
CAMERA_ANGLE_DEGREES = 25.0;    // Measure tilt angle

// MUST GET FROM GAME MANUAL
APRILTAG_HEIGHT_METERS = 1.45;  // 2026 game manual
MAX_VALID_TAG_ID = 16;           // 2026 field layout

// TUNE THESE
ALIGNMENT_TOLERANCE_DEGREES = 2.0;  // How close = aligned?
MIN_TARGET_AREA_PERCENT = 0.1;      // Ignore tiny/far targets
MAX_DISTANCE_METERS = 5.0;          // Don't trust beyond this
TARGET_TIMEOUT_SECONDS = 0.5;       // Grace period for lost targets
```

---

## Setup in RobotContainer

```java
// Real robot
private final VisionSubsystem vision = new VisionSubsystem(
    new VisionIOLimelight(Constants.Vision.LIMELIGHT_NAME)
);

// Testing/sim
private final VisionSubsystem vision = new VisionSubsystem(
    new VisionIOSim()
);
```

---

## SmartDashboard Values to Watch

While testing, put these on your dashboard:

- `Vision/State` - Current alignment state
- `Vision/HasTarget` - Is target visible?
- `Vision/IsAligned` - Ready to shoot?
- `Vision/Distance_m` - Calculated distance
- `Vision/HorizontalAngle_deg` - Angle error
- `Vision/TagID` - Which tag we're tracking

---

## Common Mistakes

### ❌ Don't Do This:
```java
// DON'T check hasTarget and then call methods that need target
if (vision.hasTarget()) {
    double dist = vision.getDistanceToTargetMeters();
    // dist could be 0 if we JUST lost target between calls!
}
```

### ✅ Do This Instead:
```java
// Methods handle NO_TARGET internally, return 0/safe values
double dist = vision.getDistanceToTargetMeters();
if (dist > 0.1) {  // Check the value, not hasTarget
    // Use dist
}
```

### ❌ Don't Do This:
```java
// DON'T spin up shooter without checking alignment
if (vision.hasTarget()) {
    shooter.spinUp();
    indexer.feed();  // Might shoot while not aligned!
}
```

### ✅ Do This Instead:
```java
// Wait for ALIGNED state
if (vision.isAligned() && shooter.isReady()) {
    indexer.feed();
}
```

---

## Troubleshooting Quick Fixes

| Problem | Quick Fix |
|---------|-----------|
| Always NO_TARGET | Check Limelight power, NetworkTables name |
| Wrong distance | Measure camera height/angle again |
| Never ALIGNED | Increase ALIGNMENT_TOLERANCE_DEGREES to 5.0 |
| Flickering states | Reduce TARGET_TIMEOUT_SECONDS or check lighting |
| Distance too high | Camera angle probably wrong (re-measure) |
| Distance too low | Camera height probably wrong (re-measure) |

---

## Testing Checklist

- [ ] Vision subsystem created in RobotContainer
- [ ] Constants measured and set
- [ ] Limelight powered and connected
- [ ] Pipeline 0 configured for AprilTags
- [ ] SmartDashboard shows vision data
- [ ] Distance accurate at 2-3 known distances
- [ ] Alignment command rotates robot correctly
- [ ] isAligned() returns true when centered
- [ ] Shooter integrates with vision distance

---

## Full Example: Shooting Sequence

```java
public Command getShootCommand() {
    return Commands.sequence(
        // Update shooter from vision
        Commands.runOnce(() -> shooter.updateFromVision(vision)),

        // Spin up
        Commands.runOnce(() -> shooter.setState(ShooterState.SPINUP)),

        // Align and wait for ready (parallel)
        Commands.deadline(
            Commands.waitUntil(() ->
                vision.isAligned() &&
                shooter.isReady() &&
                indexer.hasGamePiece()
            ),
            new AlignToTargetCommand(drive, vision)
        ),

        // Shoot
        Commands.runOnce(() -> indexer.setState(IndexerState.FEEDING)),

        // Wait for piece to leave
        Commands.waitUntil(() -> !indexer.hasGamePiece()),

        // Stop
        Commands.runOnce(() -> {
            shooter.setState(ShooterState.IDLE);
            indexer.setState(IndexerState.IDLE);
        })
    );
}
```

---

## Key Takeaways

1. **Vision provides DATA** (distance, angles) - it doesn't control anything
2. **Vision tracks STATE** (aligned or not) - commands use this to decide when to shoot
3. **Methods are safe** - they return 0/safe values when no target
4. **Use `isAligned()`** - don't just check `hasTarget()`
5. **Measure constants** - garbage in, garbage out!

For full details, see `VISION_GUIDE.md`
