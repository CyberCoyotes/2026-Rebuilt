# Intake — 2026 Season Retrospective

## What this subsystem does

The intake acquires FUEL from the floor and transfers it into the indexer. It has two motors: a **slide** (Kraken X60 with MotionMagic position control) that extends and retracts the intake assembly, and a **roller** (TalonFX with velocity control) that spins the intake wheels. The slide is zeroed against a hard stop at startup; the roller has no position feedback because it doesn't need any.

## Architecture at a glance

The intake follows the same three-file split as the indexer:

- `IntakeIO.java` — interface defining what the hardware can do
- `IntakeIOHardware.java` — real hardware implementation
- `IntakeSubsystem.java` — `SubsystemBase` with command factories, state queries, telemetry

Like the indexer, the simulation layer (`IntakeIOSim`) was pulled mid-season and only the hardware abstraction survived.

## Decisions we made and why

### Software soft limits on the slide

The slide configuration sets both `ForwardSoftLimitEnable` and `ReverseSoftLimitEnable` with thresholds from Constants. Even if a buggy command requests a position past the mechanical hard stop, the motor controller refuses to drive there.

This is defense in depth. Slide commands can have bugs, manual nudges can drift, MotionMagic targets can be miscomputed — none of those should ever physically crash the slide. Worth lifting as a standard pattern for any positional mechanism next season.

### MotionMagic profile swapping at runtime

The slide uses one motor and one `MotionMagicVoltage` request, but supports both a fast profile (for normal extend/retract) and a slow profile (for graceful retraction during fuel compression). The implementation swaps `MotionMagicConfigs` at runtime, gated by a `slowProfileActive` boolean so the same config doesn't get re-applied every call.

Clean separation: PID, current limits, soft limits stay constant; only the motion profile changes. Good example of "use one motor's capabilities to their fullest before adding complexity."

### Derived state instead of stored state

`getIntakeState()` returns a `String` computed from current sensor readings and roller direction:

```java
public String getIntakeState() {
    if (isSlideExtended() && rollerState == RollerState.RUNNING)
        return "Intaking";
    if (isSlideExtended() && rollerState == RollerState.REVERSED)
        return "Ejecting";
    if (isSlideExtended())
        return "Extended";
    // ...
}
```

This is structurally the opposite of the indexer's approach. The indexer *stores* an `IndexerState` field that gets set when commands run. The intake *derives* its state from "what does my slide position tell me + which way is my roller going." Derived state can't get out of sync with reality because it's recomputed every time it's read.

Worth noting as a teaching contrast: for the intake, derived state is cheaper and safer than maintained state because the underlying signals (slide position, roller direction) are already authoritative. The indexer's enum tries to be authoritative on its own, which is where it runs into trouble.

### Roller is uninstrumented at the IO layer

`IntakeIOInputs` exposes slide position, velocity, voltage, current, and temperature — but nothing about the roller. The comment in `IntakeSubsystem` is honest: "Raw motor signals are captured automatically by CTRE Hoot — no need to duplicate them here."

This is a deliberate tradeoff. The slide needs position in code every cycle (for MotionMagic targeting and at-target checks). The roller doesn't have control-critical signals — current and temperature are diagnostic, not driving any decisions. Pushing them through `IOInputs` would be ceremony.

The inconsistency with the indexer (which logs everything) is worth a conversation. Either pattern is defensible; the team should pick one and apply it everywhere so a student opening any IOInputs file knows what to expect.

## What worked well

### The slide-position-not-refreshing bug, captured in a comment

In `IntakeIOHardware.updateInputs()`:

> *"Refresh cached signals before reading — same pattern as IndexerIOHardware. Without this, slidePositionRotations is always 0 (startup value), so isSlideFullyExtended() / isSlideFullyRetracted() never update correctly."*

This is exactly the kind of "lesson preserved at the point of pain" comment that makes future debugging faster. Whoever hits this class of bug next season will find the answer in the code, not in someone's memory.

### IO pattern enforcement — `getSlidePosition()` removed

Visible in the commented-out code at the bottom of `IntakeIOHardware`:

> *"This was not following the IO pattern and was being called directly by the subsystem"*

There was originally a `getSlidePosition()` method on the IO interface that bypassed the `IOInputs` struct. The subsystem was calling it directly to get current position, which leaked hardware access through a back door. It got removed and the subsystem now reads `inputs.slidePositionRotations` like it should.

Small, but it shows the IO pattern is being *enforced* rather than just declared. Pattern adoption only matters if the team notices when something violates the pattern and fixes it. This is that.

### Software soft limits earned their keep

No mechanical crashes attributable to slide commands this season. With as much mechanical iteration as the team went through, that's largely thanks to the soft limits catching what would otherwise have been driver or auton bugs.

### `PhoenixUtil.applyConfig` retry pattern

Same as the indexer — eliminates a class of "config didn't actually stick on first boot" bugs.

## What we'd reconsider

### The fuel-pump command explosion

This is the headline finding for the intake, and the most direct visible artifact of the build-team feedback that they "put too much on software with all the changes."

Count of fuel-pump-related command factories in `IntakeSubsystem`:

- `compressFuel(timeoutSeconds)`
- `compressFuel(initialWaitSeconds, timeoutSeconds)`
- `compressFuelCycle(initialWaitSeconds, timeoutSeconds)` — appears functionally identical to the two-arg `compressFuel`
- `fuelCompression()` — defaults wrapper
- `fuelPumpSlow()` — held version
- `compressFuelHeld()` — alias for `fuelPumpSlow`
- `fuelPump()` — alias for `fuelPumpCycleDelayed`
- `fuelPumpCycle()` — calls `fuelPumpCycleDelayed(0.0)`
- `fuelPumpCycleDelayed(initialWaitSeconds)` — the actual implementation
- `fuelPumpCycleDelayed()` — defaults wrapper
- `fuelPumpCycleAuto(seconds)` — auton timeout wrapper
- `fuelPumpCycleUntil(stopCondition, ..., ...)` — sensor-gated version
- `fuelPumpBasic()` — earlier non-cycle implementation
- `fuelPumpSetCycles()` — fixed 2-cycle test
- `fuelPumpSetCyclesAlpha()` — **byte-identical** to `fuelPumpSetCycles`
- `fuelPumpSetCyclesRetract()` — 3-cycle + retract, with a comment noting it's missing roller commands

That's fifteen+ ways to invoke roughly the same family of behavior. Some are deliberate variants (held vs auto vs sensor-gated), some are aliases of others, some are tuning experiments that never got deleted, at least one is a literal duplicate.

This is what happens during rapid mechanical iteration when the discipline is "add a new command for each new experiment" but there's no matching "delete the old one once we picked a winner." Every "can we also try X?" got a new method. None got retired.

**Why this matters beyond aesthetics:**
- Students reading the file can't tell which commands are actually bound to buttons vs experimental dead-ends.
- Test commands in production code signal to rookies that this is normal — and it gets worse next season.
- The duplicate methods (`fuelPumpSetCycles` / `fuelPumpSetCyclesAlpha`) are a real maintenance trap. If someone fixes a bug in one, the other still has it. Worse: they share a `.withName("FuelPumpBasic")` value, so logs don't distinguish them.

**The cleanup is mechanical:**
1. List every fuel-pump command that's actually referenced in `RobotContainer` button bindings or auton routines.
2. Everything not on that list either moves to `IntakeTestCommands.java` (preserve for next year's tuning) or gets deleted.
3. The duplicates get reconciled — keep one, delete the other.
4. The aliases get evaluated: do they still earn their keep, or is the underlying name fine?

Realistic outcome: this file should be 200–300 lines shorter without losing any functionality used in a match.

### Identical duplicate methods

`fuelPumpSetCycles()` and `fuelPumpSetCyclesAlpha()` appear to have identical bodies — same sequence, same timeouts, same `runSlowRoller()` calls, even the same wrong `.withName("FuelPumpBasic")` at the end. One was almost certainly meant as a divergent experiment that never diverged. Pick one, delete the other.

### Misplaced `.finallyDo()` in `fuelPumpSetCycles()`

Look at the end of `fuelPumpSetCycles`:

```java
Commands.waitSeconds(0.1)
    .finallyDo(this::stopRoller).withName("FuelPumpBasic"));
```

The `.finallyDo(this::stopRoller)` is attached to the *final* `Commands.waitSeconds(0.1)` — not to the outer `Commands.sequence(...)`. That means the roller only stops if that specific wait is interrupted. If the sequence completes naturally, or is interrupted during a non-final step, the roller stop may not fire as intended.

The fix is one set of parentheses moved — wrap the whole sequence and attach `finallyDo` outside. Worth flagging because it's a class of bug that's easy to write and easy to miss in review.

### `withName` mismatches

Multiple commands `withName("FuelPumpBasic")` even when they aren't `fuelPumpBasic`. Two of the duplicate commands above carry this name, as does `fuelPumpSetCyclesAlpha`. In Glass or AdvantageScope, all of these show up as "FuelPumpBasic" running, making it impossible to tell which command is actually scheduled. Names should match factory methods or describe the actual behavior; copy-paste names cost real debugging time later.

### Commented-out code accretion

Significant commented-out code throughout:

- The entire roller-follower configuration (originally a dual-roller setup that was abandoned)
- The `extendSlides()` / `retractSlides()` backward-compat aliases
- The `getSlidePosition()` IO method (correctly removed for IO-pattern reasons, but the dead code remains)
- `retractSlidesStack()` — the multi-step retract workaround, with a comment saying "Still in use as a fallback" while the actual code is commented out (contradiction)
- Sensor methods at the end of `IntakeIO` from an intake-distance-sensor plan that didn't ship

Each of these had a reason at the time. None of them belongs in main-branch code now. Either restore them with a clear reason, or remove them — the in-between state is the worst of both worlds because it implies the code is "almost active" when in fact it's just clutter.

### Method naming inconsistency

Some command factories carry a `Cmd` suffix (`extendSlidesFastCmd`, `retractSlidesFastCmd`), others don't (`intakeRoller`, `reverseIntakeRoller`, `intakeFuel`, `stopFuel`). The convention should be picked and applied consistently. Lean toward dropping `Cmd` — the return type already says `Command`, and the suffix adds noise without disambiguating.

### Re-sending MotionMagic targets every loop in `fuelPumpCycleDelayed`

The `Commands.run(...)` inside `fuelPumpCycleDelayed` calls `setSlidesToPosition(...)` every 20 ms, even when the target hasn't changed. MotionMagic is already holding the position — re-issuing the same target each loop is wasteful work on the controller side and contributes (a small amount) to loop time.

The cleanup: track the current pump phase, only call `setSlidesToPosition(...)` on phase change, otherwise just call `runRoller()`. Not a correctness bug, but the kind of thing worth fixing as part of any loop-overrun investigation.

### Documentation drift from the IOSim pull

The header comment in `IntakeIOHardware.java` includes:

> *"All telemetry logged via AdvantageKit"*

AdvantageKit logging was substantially scaled back this season and the sim layer was pulled. The comment is no longer accurate. Same pattern as the indexer — when a layer is pulled, its docs need to come with it.

## Open questions for the student owner

1. **Which fuel-pump commands were actually bound to buttons or used in auton routines?** This list determines what survives the cleanup and what moves to test commands or gets deleted. The student owner should literally walk through `RobotContainer` and the auton files and produce the list.
2. **Was the dual-roller plan ever going to come back?** The follower roller config is commented out across all three files. Either restore it with a working implementation or delete the dead code.
3. **Did `retractSlidesStack` actually fire as a workaround in any real match, or was the MotionMagic retraction problem solved by the time competition arrived?** The comment claims "still in use as a fallback" but the method is commented out, so it's literally not in use. One of those two states is correct; figure out which.
4. **What were the `SLIDE_PUMP_OUT_POS` and `SLIDE_PUMP_IN_POS` semantics intended to mean?** "Pump out" suggests extending, "pump in" suggests retracting, but they're used inside bouncing motions where direction depends on cycle phase. Worth clarifying the naming or adding a comment so the next person doesn't have to reverse-engineer it.
5. **Was the asymmetric instrumentation (slide fully logged in IOInputs, roller relying on Hoot) deliberate, or did it just happen?** If deliberate, document the rule for next season ("control-critical signals go in IOInputs, diagnostic-only signals rely on Hoot"). If accidental, pick one approach and apply it to both motors.

## Lessons for next season

1. **Pruning is a build-season discipline, not a postseason chore.** The fuel-pump explosion happened because every experiment got a new method and none got retired. Adopt a rule: when a new variant is committed, the obsolete one is deleted (or moved to an explicit test-commands file) in the same PR. Otherwise accretion is inevitable.
2. **A command not bound to anything is dead code.** At the end of build season — and again before each event — grep `RobotContainer` and auton files for every command factory in a subsystem. Anything not referenced gets a deletion PR.
3. **Software soft limits should be the default for positional mechanisms.** Not an opt-in. Set them whenever soft-limit constants exist; treat their absence as the exception that needs justification.
4. **Derived state beats stored state when the underlying signals are authoritative.** The intake's `getIntakeState()` doesn't store anything — it computes the answer from current sensor readings + tracked roller direction. Can't drift. Apply this pattern wherever the question "what's this subsystem doing right now?" can be answered from inputs alone.
5. **Identical-twin methods are a code review failure mode.** `fuelPumpSetCycles` and `fuelPumpSetCyclesAlpha` being byte-identical means at least one PR was approved without anyone reading the diff. Worth a conversation about review expectations.
6. **Pick a logging instrumentation convention and apply it everywhere.** Either control-critical-only goes through IOInputs (intake's approach) or everything diagnostic goes through IOInputs (indexer's approach). The inconsistency between subsystems is more confusing than either pattern alone.
