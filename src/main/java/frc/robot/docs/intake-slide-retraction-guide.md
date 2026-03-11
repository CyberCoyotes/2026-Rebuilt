# Intake Slide Retraction Approaches

This document explains the different methods available for retracting the intake slides in `IntakeSubsystem`, when to use each one, and why each approach exists.

---

## Overview

The intake slide mechanism uses CTRE MotionMagic for position control. However, real-world performance under load led to the development of multiple retraction strategies, each optimized for different use cases.

**Key Concept**: MotionMagic profiles control how fast the motor moves to a target position. The intake uses two profiles:
- **Normal MotionMagic**: Fast retraction for quick stowing
- **DynamicMotionMagic (Slow)**: Gradual retraction for compressing fuel while avoiding jams

---

## Low-Level Methods (Actuators)

These are called by command factories. They do NOT return Commands.

### 1. `retractSlides()`
**What it does**: Moves slide to retracted position using the normal (fast) MotionMagic profile.

**Technical details**:
- Single call to `io.setSlidePosition(SLIDE_RETRACTED_POS)`
- Motor holds position after arrival via MotionMagic
- Uses the default fast profile configured in Constants/IO layer

**When to use**:
- Quick stowing after intaking
- When speed matters more than smoothness
- When NOT compressing fuel (just moving the slide)

**Usage pattern**:
```java
// Typically wrapped in a runOnce command
retractSlides();  // One call → motor handles the rest
```

---

### 2. `retractSlidesSlow()`
**What it does**: Moves slide to retracted position using the DynamicMotionMagic slow profile.

**Technical details**:
- Calls `io.setSlidePositionSlow(SLIDE_RETRACTED_POS)`
- **Must be called continuously** (every loop cycle) to maintain the slow profile
- If called once and stopped, the slow profile won't stay active

**When to use**:
- Inside `Commands.run()` blocks (not `runOnce`)
- When compressing fuel into the hopper
- When you need gradual, controlled motion to avoid jamming

**Usage pattern**:
```java
// Must be called repeatedly — use inside Commands.run()
Commands.run(() -> retractSlidesSlow(), this)
    .until(this::isSlideFullyRetracted);
```

---

### 3. `retractSlidesIncremental()`
**What it does**: Retracts slide by 15 rotations from its current position, clamped to minimum.

**Technical details**:
- Calculates `currentPosition - 15.0`, then clamps to `SLIDE_MIN_POSITION`
- Uses normal (fast) MotionMagic
- Relative positioning — doesn't commit to full retraction in one step

**When to use**:
- Quick "nudge" to start retracting without full commit
- First phase of multi-step retraction (see `retractSlidesAuton()`)
- Manual incremental control via repeated button presses

**Usage pattern**:
```java
// Each call moves 15 rotations back
retractSlidesIncremental();  // Click → jump back 15 rotations
```

---

## Command Factories (Teleop)

These return `Command` objects for button bindings.

### 4. `retractSlidesCmd()`
**What it does**: Command wrapper for `retractSlides()` using `runOnce`.

**When to use**:
- Bind to a button with `.onTrue()` for instant retraction
- When you want a fire-and-forget retract (no manual control)

**Button binding example**:
```java
operator.buttonX().onTrue(intake.retractSlidesCmd());
```

---

### 5. `retractSlidesSlowCmd()`
**What it does**: Command wrapper for `retractSlidesSlow()` using `runOnce`.

**⚠️ Warning**: This sends the slow profile command ONCE, but DynamicMotionMagic needs continuous re-sends to stay active. This command may not maintain the slow profile as expected.

**When to use**:
- Rarely used in practice
- Consider using `compressFuel()` or `compressFuelHeld()` instead

---

### 6. `retractSlidesIncrementalCmd()`
**What it does**: Command wrapper for `retractSlidesIncremental()` using `runOnce`.

**When to use**:
- Bind to a button for manual incremental retraction
- Let drivers control slide position in stages

**Button binding example**:
```java
operator.dpadDown().onTrue(intake.retractSlidesIncrementalCmd());
// Each press moves slide back 15 rotations
```

---

## Command Factories (Teleop Combinations)

High-level commands combining slide motion with roller control.

### 7. `compressFuel(double timeoutSeconds)`
**What it does**: Slowly retracts slides while running the roller for a fixed duration.

**Technical details**:
- Uses `Commands.run()` to continuously call `retractSlidesSlow()` + `runRoller()`
- Automatically stops after timeout
- `finallyDo()` ensures roller stops even if interrupted

**When to use**:
- Timed compression — you want it to finish on its own
- Typical duration: 2.0 seconds (matches slide travel time)

**Button binding example**:
```java
operator.leftBumper().onTrue(intake.compressFuel(2.0));
// Press once → runs for 2 seconds → stops automatically
```

---

### 8. `compressFuelHeld()`
**What it does**: Slowly retracts slides while running roller, but only while button is held.

**Technical details**:
- Uses `startEnd()` instead of timed execution
- Stops the instant button is released
- Actually uses `retractSlidesIncremental()` in the current implementation (not slow retract)

**When to use**:
- Manual compression control — you control exactly how long it runs
- When you want to stop mid-compression

**Button binding example**:
```java
operator.leftBumper().whileTrue(intake.compressFuelHeld());
// Hold → compress; Release → stop
```

---

## Workaround Commands

### 9. `retractSlidesStack()`
**What it does**: Re-sends the retract setpoint 4 times with 1-second pauses between each.

**Why it exists**:
MotionMagic occasionally fails to complete retraction in a single `runOnce` call under heavy load (fuel jams, mechanical resistance). Re-commanding the setpoint multiple times with pauses gives the mechanism time to settle after each attempt.

**Technical details**:
```java
retractSlides() → wait 1s → retractSlides() → wait 1s → ... (×4)
```

**When to use**:
- Fallback when normal retraction fails
- When retraction reliability matters more than speed

**Status**:
- Still in use as a backup strategy
- If MotionMagic becomes fully reliable, this can be removed

---

## Autonomous Commands

### 10. `retractSlidesAuton()`
**What it does**: Two-phase retraction with roller running throughout.

**Phase 1 (runOnce)**:
- Quick 15-rotation jump back via `retractSlidesIncremental()`
- Roller starts running

**Phase 2 (run loop)**:
- Slow DynamicMotionMagic finish to fully retracted position
- Continuously re-commands slow profile until `isSlideFullyRetracted()` returns true

**When to use**:
- Autonomous routines where you want to retract WHILE shooting
- Designed to run alongside a shooter command in a `Commands.deadline()`

**Typical auton usage**:
```java
Commands.deadline(
    FuelCommands.shootWithPreset(shooter, indexer, rpm, hood),  // Deadline
    intake.retractSlidesAuton()  // Runs alongside, stops when deadline finishes
);
```

**Why two phases?**
- Phase 1: Gets the slide moving quickly to avoid mechanical interference with shooter
- Phase 2: Finishes smoothly while keeping roller running to feed fuel into hopper

---

## Decision Tree: Which Method Should I Use?

```
Are you in autonomous?
├─ YES → Use retractSlidesAuton() (two-phase with roller)
│
└─ NO (Teleop) → What's your goal?
    │
    ├─ Quick stowing (no compression)
    │   └─ Use retractSlidesCmd() or stopFuel()
    │
    ├─ Compressing fuel with timed auto-stop
    │   └─ Use compressFuel(2.0)
    │
    ├─ Compressing fuel with manual control
    │   └─ Use compressFuelHeld()
    │
    ├─ Manual incremental control (nudge back)
    │   └─ Use retractSlidesIncrementalCmd()
    │
    └─ Retraction keeps failing under load
        └─ Use retractSlidesStack() (workaround)
```

---

## Common Mistakes

### ❌ Using `retractSlidesSlow()` in a `runOnce`
```java
// WRONG — slow profile needs continuous re-send
Commands.runOnce(this::retractSlidesSlow, this);
```

**Fix**: Use inside `Commands.run()` with an end condition:
```java
// CORRECT
Commands.run(this::retractSlidesSlow, this)
    .until(this::isSlideFullyRetracted);
```

---

### ❌ Forgetting to stop the roller
```java
// WRONG — roller keeps running forever
Commands.run(() -> {
    retractSlidesSlow();
    runRoller();
}, this);
```

**Fix**: Add `finallyDo()` to clean up:
```java
// CORRECT
Commands.run(() -> {
    retractSlidesSlow();
    runRoller();
}, this)
.until(this::isSlideFullyRetracted)
.finallyDo(() -> stopRoller());  // Always stops, even if interrupted
```

---

### ❌ Calling actuator methods directly in button bindings
```java
// WRONG — no Command wrapper
operator.button().onTrue(() -> retractSlides());
```

**Fix**: Use the command factory:
```java
// CORRECT
operator.button().onTrue(intake.retractSlidesCmd());
```

---

## Summary Table

| Method | Type | Speed | Roller? | Use Case |
|--------|------|-------|---------|----------|
| `retractSlides()` | Actuator | Fast | No | Quick stowing |
| `retractSlidesSlow()` | Actuator | Slow | No | Compression (must loop) |
| `retractSlidesIncremental()` | Actuator | Fast | No | 15-rotation nudge |
| `retractSlidesCmd()` | Command | Fast | No | Teleop quick retract |
| `retractSlidesSlowCmd()` | Command | Slow* | No | Rarely used |
| `retractSlidesIncrementalCmd()` | Command | Fast | No | Manual nudge button |
| `compressFuel(seconds)` | Command | Slow | Yes | Timed compression |
| `compressFuelHeld()` | Command | Incremental | Yes | Manual compression |
| `retractSlidesStack()` | Command | Fast | No | Workaround for failures |
| `retractSlidesAuton()` | Command | Fast→Slow | Yes | Auton two-phase retract |

\* `retractSlidesSlowCmd()` may not maintain slow profile due to single runOnce call.

---

## Further Reading

- [IntakeSubsystem.java](../subsystems/intake/IntakeSubsystem.java) — Full source code
- [Constants.java](../Constants.java) — Slide position setpoints and voltage values
- [io-hardware-subsystem-pattern.md](io-hardware-subsystem-pattern.md) — AdvantageKit IO layer architecture
