# Shooter — 2026 Season Retrospective

## What this subsystem does

The shooter launches FUEL using a flywheel-and-hood configuration. Two TalonFX motors drive twin flywheels in a leader/follower pair, spinning up to a target RPM. A TalonFXS-driven hood adjusts the launch angle. Shot targeting comes from one of three sources: a vision-based distance lookup (interpolated from the hub AprilTag), an operator-selected preset (close, tower, trench, far, pass, etc.), or a tuning override for characterization.

## Architecture at a glance

The shooter follows the same three-file split as the other subsystems:

- `ShooterIO.java` — interface defining what the hardware can do
- `ShooterIOHardware.java` — real hardware implementation with CTRE Phoenix 6
- `ShooterSubsystem.java` — `SubsystemBase` containing the real state machine, vision interpolation, and presets

This subsystem is the most sophisticated in the codebase. It is also the success case for nearly every pattern the team has been developing — real FSM, CAN bus optimization, fast/slow input split, safety guards, vision integration. The retrospective findings here are mostly polish, not architecture.

## Decisions we made and why

### A real state machine — the pattern actually working

Unlike the indexer (where the enum became a status indicator) the shooter's `ShooterState` is a genuine finite state machine:

- `updateStateMachine()` in `periodic()` reads `currentState` and drives motor outputs accordingly. The state controls the behavior.
- `setState()` runs entry actions on transition (e.g. stop flywheels when entering `IDLE`, set initial commands when entering `READY`).
- The `SPINNING_UP → READY` promotion is owned by `periodic()` alone — commands cannot set `READY` directly. The comment makes this explicit: *"Only periodic() earns the READY state — never set directly."*
- Vision updates respect mode boundaries — `updateFromDistance()` refuses to override `POPPER`, `EJECT`, or `PASS` states.

This is what the indexer's enum was trying to be. When the indexer offseason rewrite happens, this is the model to point at.

### CAN bus optimization with per-signal update rates

`ShooterIOHardware` does real performance work that no other subsystem does:

```java
// Disable all status frames not explicitly re-enabled below.
flywheelLeader.optimizeBusUtilization();
flywheelFollower.optimizeBusUtilization();
hoodMotor.optimizeBusUtilization();

BaseStatusSignal.setUpdateFrequencyForAll(100.0, flywheelLeader.getDutyCycle(), flywheelLeaderVoltage);
BaseStatusSignal.setUpdateFrequencyForAll(50.0,  flywheelLeaderVelocity, hoodPosition);
BaseStatusSignal.setUpdateFrequencyForAll(10.0,  flywheelLeaderTempCelsius, flywheelFollowerTempCelsius);
```

Each signal gets the update rate it actually needs — velocity at 50 Hz for the control loop, temperature at 10 Hz for diagnostics, everything else disabled. Two real gotchas were caught and documented along the way:

- *"setUpdateFrequency calls MUST come AFTER optimizeBusUtilization() or they will be cleared."*
- *"Followers MUST be set AFTER optimizeBusUtilization() — aggressive frame disabling can break the follower control link if set before."*

Given the team's loop-overrun struggles this season, this is exactly the right approach and should be the template for every subsystem next year.

### Fast/slow input split

The IO interface declares two refresh methods — `updateInputs()` for control-critical signals every cycle, and `updateSlowInputs()` for diagnostics at 10 Hz. The subsystem uses both:

```java
@Override
public void periodic() {
    io.updateInputs(inputs);          // every cycle
    // ... state machine ...
    if (++periodicCounter % 5 == 0) {
        io.updateSlowInputs(inputs);   // every 5th cycle
        publishToElastic();
    }
}
```

The comment in the IO interface explains the rationale: *"These signals are set to 10Hz update rate on the CAN bus, so refreshing them every 20ms just reads stale cached values and wastes JNI calls."* This is loop-overhead thinking applied correctly.

### Safety guards on dangerous transitions

`eject()` refuses to enter EJECT mode if the flywheel is spinning too fast:

```java
public void eject() {
    if (Math.abs(getCurrentVelocityRPM()) > Constants.Flywheel.EJECT_MAX_ENTRY_RPM) {
        return; // Flywheel spinning too fast to safely reverse — refuse eject
    }
    setState(ShooterState.EJECT);
}
```

Reversing a flywheel at 6000 RPM would be violent and potentially destructive. This is the kind of guard that prevents a class of catastrophic bug — a misbinding or rookie button press can't damage the mechanism. Same idea as the slide soft limits in the intake.

### MotionMagic velocity for the flywheel

Flywheel control uses `MotionMagicVelocityVoltage` (not plain `VelocityVoltage`), with acceleration and jerk limits configured in Slot 0. This gives a smooth ramp to target instead of a velocity-step demand the controller has to chase. Combined with the `STANDBY` pre-rev mode, the spin-up time on a fresh shot is much shorter than a cold start.

### Operator preset display without firing

The operator can hold a preset button to show what shot is currently selected on the Elastic dashboard, *without* requiring the driver to pull the trigger. The trigger fires; the operator's button only displays. This is a small human-factors detail with big practical impact — the operator can confirm "yes, you're about to fire a Tower shot" before the driver commits.

```java
public void setDisplayPreset(ShotPreset preset) {
    displayPreset = preset;
    // Does NOT change shooter state or target RPM/hood.
}
```

## What worked well

### The state machine pattern, in full

Beyond the points above — every pattern that should be in an FSM is in this one. Entry actions on `setState`, continuous commands in `updateStateMachine`, automatic promotion via sensor (`SPINNING_UP → READY` on `isFlywheelAtVelocity()`), explicit transition refusal in `eject()`. This is the textbook example. Once an indexer FSM rewrite is done in the offseason, point students at the shooter and ask "what's missing from yours that the shooter has?"

### Interpolating distance-to-shot lookup

`InterpolatingDoubleTreeMap` for both flywheel RPM and hood pose, keyed by distance from the hub tag. Linear interpolation between measured points, clamping to endpoints outside the range. Plus a great defensive comment:

> *"Both maps MUST have identical distance keys — they are co-indexed. Adding a distance to one map without adding it to the other produces inconsistent RPM/hood pairings at that distance. Always update both."*

This is the right pattern for distance-aware shooting. The implementation is solid; the data going into it is the weak point (see findings below).

### Documentation of the tuning procedure inline

The vision lookup section includes a numbered procedure for how to characterize a new distance — place robot, enable, watch dashboard, command shot, adjust, record. That's exactly the kind of documentation that doesn't exist on most teams and gets re-learned by every cohort. Worth lifting to TUNING.md.

### `tuneFlywheelCommand(double rpm)` is explicitly labeled

Compare to the intake's test commands, which are mixed into the production command surface with no marker. The shooter has a single tuning command, clearly named, with documentation about what to watch on the dashboard. Same kind of code (open-loop tuning helper), much better hygiene.

### `setIdle()` as the single shot-end point

All command `finallyDo()` callers route through `setIdle()`, which centrally handles the "do we return to STANDBY or all the way to IDLE" decision based on the standby flag. One place to change shot-end behavior. Good.

## What we'd reconsider

### Dead fields in `ShooterIOInputs`

Several fields are declared in `ShooterIOInputs` but never populated by `ShooterIOHardware`:

- `flywheelCurrentAmps` — declared, no `refresh()` or assignment
- `hoodAppliedVolts` — declared, no `refresh()` or assignment
- `hoodCurrentAmps` — declared, no `refresh()` or assignment
- `hoodAngleDegrees` — declared, no conversion from rotations

Each of these reads `0.0` for the entire match. The most concrete consequence:

```java
public boolean isOverCurrent() {
    return inputs.flywheelCurrentAmps > 150.0;
}
```

This check **always returns false** because `flywheelCurrentAmps` is always zero. If anything depends on it, that dependency silently fails. The fix is to either:
1. Populate the fields in `updateSlowInputs()`, or
2. Delete the fields from `IOInputs` and delete `isOverCurrent()` if no one uses it.

Either is defensible. The current half-state is the worst option.

### `setHoodVoltage()` exists in the interface but isn't implemented

`ShooterIO.setHoodVoltage(double volts)` is declared as a `default void` empty method on the interface for "open-loop slow movement when finding hood travel limits." `ShooterIOHardware` does not override it. Calling it does nothing.

If this capability is still needed (for hood-zero discovery, mechanical inspection), wire it up. If not, delete it from the interface. Phantom capabilities are confusing to read and tempting to call.

### `setFlywheelVelocityTorqueFOC()` — declared but not wired

Same pattern as `setHoodVoltage`. The interface declares it with a clear comment about CAN FD / CANivore requirements. The hardware does not implement it. The subsystem never calls it. It's a stub for a future CANivore migration that didn't happen.

Either bring it forward as a real plan with the CANivore migration on the offseason list, or delete it. Comment in the file currently reads: *"On RIO CAN it will not perform as intended — for comparison testing only until flywheel motors are moved to CANivore."* — that intent should either be tracked or retired.

### Toggle method duplication

There are two methods that do the same thing:

- `toggleStandby()`
- `toggleStandbyMode()`

Both flip `standbyEnabled` and conditionally transition between `IDLE` and `STANDBY`. Their bodies are nearly identical. Same problem as the intake's fuel-pump explosion, smaller scale — pick one, delete the other.

Similarly:
- `setStandby()` and `setState(ShooterState.STANDBY)` — `setStandby()` is just a forwarder. Pick one.
- `setIdle()` and `setPostShotState()` — also appear to do the same thing. Pick one.
- `isStandbyEnabled()` is commented out earlier in the file and then defined again later — the commented-out version is dead code.

### Sparse vision lookup table

The active distance entries in the lookup are: `CLOSE`, `TOWER`, `FAR`. That's it. Several entries are commented out:

```java
// FLYWHEEL_RPM_MAP.put(Constants.Shooter.TRENCH_DISTANCE, Constants.Shooter.TRENCH_RPM);
// FLYWHEEL_RPM_MAP.put(3.50, 2625.0);
// FLYWHEEL_RPM_MAP.put(4.50, 3465.0);
```

Three measured points across the full shot envelope means linear interpolation does most of the heavy lifting between very widely-spaced anchors. Mid-range shots (e.g. 3-4 m) are extrapolated from data that wasn't taken at those distances.

This is a real product weakness, not a code issue. The fix is **measurement, not coding** — block out time on the practice field with a tape measure, fill in TRENCH plus 2-3 mid-range distances, watch the shot grouping tighten. The infrastructure to use that data is already in place; it's waiting for inputs.

This will also come up in the vision retrospective. The shooter's lookup can only be as good as the distance measurements that feed it.

### Hood-at-pose isn't part of the main shot-readiness check

The state machine promotes `SPINNING_UP → READY` based only on `isFlywheelAtVelocity()`. The comment justifies it: *"Hood moves near-instantly so flywheel velocity is the only meaningful gate."*

This is an assumption that should be verified, not asserted. A few questions for the season-after review:
- Was hood travel time ever a problem in practice? E.g. firing a tower shot right after a close shot, where the hood has to traverse a long range.
- Was there ever a missed shot where the flywheel was at speed but the hood hadn't arrived yet?
- The Hoot logs from a shot-heavy match should answer this — plot `HoodError` vs `FlywheelError` over the seconds leading up to fire commands.

If the hood is indeed always faster than the flywheel, the current gate is fine and the comment should stay. If there's evidence of hood lag at the moment of fire, `isReady()` should AND with `isHoodAtPose()`. The data exists; it just hasn't been looked at.

### `commandFlywheelVelocity()` is a 1:1 pass-through

The private helper:

```java
private void commandFlywheelVelocity(double rpm) {
    io.setFlywheelVelocity(rpm);
}
```

The comment says *"Routes flywheel velocity command to the active control mode"* — suggesting there was originally going to be control-mode switching (VoltageVel vs TorqueFOC). Since TorqueFOC was never wired up, this method adds no value. Either inline it or fulfill its original purpose.

### Duplicate state initialization in `setState()` and `updateStateMachine()`

Both methods set flywheel velocity and hood pose for the `READY` state. `setState()` does it once on transition; `updateStateMachine()` does it every cycle. The cycle-level command will overwrite the transition-level command 20 ms later anyway, so the transition-level command is redundant.

Not a bug, but reading the code is slightly confusing — students might wonder if the duplication is intentional. Either delete the redundancy or add a comment ("set immediately so the first cycle after transition doesn't have stale outputs").

### Documentation drift from the IOSim pull

`ShooterIO.java` header still describes the three-file pattern including `ShooterIOSim`. Same documentation drift as every other subsystem. When the sim layer was pulled, the docs didn't come with it.

### Unclosed comment formatting before `setState()`

Minor — the comment block opens with `/*` but never closes with `*/`. The following line of code closes it accidentally. Doesn't break anything, but is a small code-review polish item.

## Open questions for the student owner

1. **Did hood lag ever bottleneck a shot?** Look at Hoot logs for matches with rapid shot variety. If `HoodError` stays non-zero through the moment of fire, the `isReady()` gate is missing the hood and needs to AND with `isHoodAtPose()`.
2. **Was the CANivore migration ever real?** `setFlywheelVelocityTorqueFOC()` is wired into the interface but never used. Either offseason migration is on the list or this should be deleted.
3. **Were all three vision distance points actually characterized for the competition robot?** The TRENCH distance is commented out as "so close to Tower, it might add confusion." That's a design choice, but it means there's a 2+ meter gap between TOWER and FAR with no measured anchor.
4. **What was the practical use of `STANDBY` mode in matches?** Was the operator toggling it on at the start of teleop and leaving it on, or actively managing it during play? Drives whether the toggle should default ON next season or stay opt-in.
5. **Did the operator preset-display feature get used by the driver in match?** If so, it's a model for other "show but don't commit" patterns. If not, it's clutter that can simplify.

## Lessons for next season

1. **This is the FSM model.** When the indexer offseason rewrite happens, the shooter is the reference implementation. Specifically: `updateStateMachine()` drives behavior from state, `setState()` runs entry actions, periodic-only promotion to gated states, and explicit transition refusal for unsafe moves.
2. **`optimizeBusUtilization()` + per-signal update rates is the CAN bus playbook.** Apply to every subsystem that talks to Phoenix 6 devices, not just the shooter. The two gotchas (frame rates after optimize, followers after optimize) are now documented and should propagate.
3. **Fast/slow input split is the playbook for loop overhead.** Control-critical signals at the full cycle rate; diagnostic signals at 10 Hz to match their CAN update rate. Worth extending to indexer and intake — both currently refresh everything every cycle.
4. **Safety guards on dangerous transitions are mandatory.** The shooter's eject-velocity check and the intake's slide soft limits are the same idea — refuse the operation if the precondition makes it unsafe. Adopt as a code review checklist item: *"What's the worst thing this command could do if called at the wrong time, and is there a guard?"*
5. **IOInputs fields without populators are landmines.** The dead `flywheelCurrentAmps` field makes `isOverCurrent()` silently broken. New rule: a field in IOInputs that isn't being populated in `updateInputs` or `updateSlowInputs` is a build error or a code-review reject. Same for interface methods declared with no hardware implementation.
6. **A measurement gap is not a code problem.** The shooter's vision lookup table is sparse because the distances weren't measured, not because the lookup code is bad. Distinguish "this needs more engineering" from "this needs more time on the practice field" — and budget for the field time accordingly during build season.
