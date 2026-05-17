# Indexer — 2026 Season Retrospective

## What this subsystem does

The indexer moves FUEL from the intake to the shooter through two motor stages — a conveyor (Minion on TalonFXS) that feeds the hopper, and a kicker (twin Krakens in a leader/follower pair) that pushes fuel into the shooter wheels. A CANrange sensor at the chute confirms when fuel has cleared into the shooter.

## Architecture at a glance

The indexer is split into three files:

- `IndexerIO.java` — interface defining what the hardware can do
- `IndexerIOHardware.java` — real hardware implementation owning all Phoenix 6 objects
- `IndexerSubsystem.java` — the `SubsystemBase` that holds business logic, command factories, telemetry

This is the AdvantageKit-style IO pattern, minus the simulation layer.

## Decisions we made and why

### Kept the IO interface split. Pulled the sim layer.

Going into the season we adopted the full IO pattern (interface + hardware + sim) on the indexer as a selective experiment — not on every subsystem. The sim layer (`IndexerIOSim`) was removed mid-season because the cost wasn't paying off in our workflow:

- Mechanical iteration was extremely rapid. Every hopper geometry change rippled through the IO interface, the hardware implementation, *and* the sim model. We were maintaining three things instead of one.
- Loop overruns appeared as the indexer code got more complex; ioSim was one contributor (though confirming this in the Hoot logs is a TODO — see Open Questions).
- Students were getting confused about where the "real" code lived. Two implementations of the same interface is overhead when you're not actually running both.

The interface/hardware split survived because that part *did* earn its keep:

- Phoenix 6 hardware setup (configs, status signals, control requests) is verbose. Pushing it into a dedicated `IOHardware` file kept `IndexerSubsystem.java` focused on logic.
- The interface gives us a place to specify *what the hardware can do* separate from *how it does it* — useful for code review with students.

**Takeaway:** the IO pattern is two ideas glued together — hardware abstraction and simulation. They can be adopted independently. We kept the part that paid for itself.

### Closed-loop velocity for forward, voltage for reverse/popper

Forward operation on both conveyor and kicker uses `VelocityVoltage` with Slot 0 PID. Reverse and air-popper use `VoltageOut`.

This is consistent with the team principle: closed-loop only when a target RPM can be defined. Forward operation has a specific feed rate that matters for shot consistency. Reverse just needs to back things out — there's no target velocity that means anything. Popper is a one-shot impulse.

Worth documenting because future-us will look at the mixed mode and wonder if it was deliberate or sloppy. It was deliberate.

### Follower direction governed by the Follower request, not Inverted

The kicker follower config deliberately omits `MotorOutput.Inverted` — the comment in `kickerFollowerConfig()` notes "while in follower mode, direction is governed by the Follower request." This is a Phoenix 6 subtlety worth flagging because setting Inverted on a follower fights the leader. Easy mistake to make and hard to debug.

## What worked well

### `feedUntilChuteEmpty(safetyTimeout)`

This command bridges sensor-based completion with a timer fallback. It feeds until the chute sensor confirms fuel has cleared *and* enough time has passed (`FUEL_CLEAR_TIME`), with a hard timeout in case the sensor misses the event. In auton, this replaced fixed-duration `feed().withTimeout(seconds)` calls and made shot completion deterministic instead of optimistic.

The `hasSeenFuel` guard is the subtle bit that makes it work. Without it, `isChuteEmpty()` would return true immediately — the chute *is* empty before any ball has arrived. Requiring a prior detection forces the command to actually see a ball before declaring the shot done.

This is a pattern worth lifting to the rest of the codebase wherever we need "wait until something completes." Good template for next season.

### `PhoenixUtil.applyConfig` retry logic

Single-attempt `apply()` calls on the RIO CAN bus can return `OK` prematurely if the device is still booting. The retry helper closed that class of "config didn't actually stick" bugs that used to surface as weird first-boot behavior.

### Typed `StatusSignal<T>` migration

We moved from `StatusSignal<?>` to typed signals like `StatusSignal<AngularVelocity>` and `StatusSignal<Current>` mid-season (per the comment in `IndexerIOHardware`). The compiler now catches type mismatches at signal usage. Worth noting as an example of "small refactors that pay off."

## What we'd reconsider

### The state machine that became a status indicator

This is worth telling as the story it actually was, because the journey is the lesson.

We *intended* `IndexerState` to be a full FSM. The original design called for `periodic()` to read the current state and drive motor output accordingly, with transition guards preventing illegal moves (e.g. can't go directly from `EJECTING` to `FEEDING`). Commands would request state changes; `periodic()` would enforce them. That was the plan.

What actually shipped: a status indicator. The enum gets set inside `startEnd` command lambdas alongside direct motor calls. `periodic()` never references `currentState`. The label is reporting what's happening — it isn't *causing* what's happening.

Trace through `feed()` to see it:

```java
public Command feed() {
    return Commands.startEnd(
        () -> {
            setState(IndexerState.FEEDING);  // label gets stamped
            conveyorForward();                // motors actually run — independent of the label
            kickerForward();
        },
        () -> {
            stop();
            setState(IndexerState.IDLE);
        },
        this);
}
```

If `setState(...)` calls were deleted entirely, the indexer would behave identically. That's the test: in a real FSM, removing the state assignment would break the subsystem. Here it wouldn't.

**Why this happened:** we were new to state machines as a team. Under build-season pressure, the FSM design morphed into "set a label when commands run" because that was the path of least resistance — and once a few command factories were written that way, the rest followed the same pattern. The intent didn't survive contact with the iteration pace.

**When we noticed:** mid-season. The enum was visibly "fancy return labels" rather than a controller. There wasn't bandwidth to rewrite during the season, so we shipped it as-is and made a note.

**Why this isn't catastrophic:** the indexer doesn't have many illegal transitions worth guarding against. Driver-initiated feed and eject are short-lived, the operator can't really cause harm by mashing buttons, and the `hasSeenFuel` guard already covers the one real coordination problem (don't declare the shot done before a ball was actually there). A status indicator is genuinely useful for telemetry, LED feedback, and other subsystems querying "what's the indexer up to right now."

**What we plan to do:** offseason rewrite as deliberate practice — convert this subsystem to a real FSM where `periodic()` is the only place that drives motors, commands only call `requestState(...)`, and transition guards live in one place. The indexer is a good practice target because it's simple enough to fully understand but has enough states to make the pattern real. Once the team is comfortable with the pattern here, we can apply it to subsystems where the FSM actually earns its keep (shooter spin-up gating, superstructure coordination for the 2027 game).

**Sketch of the rewrite target** (for the offseason planning doc):

```java
@Override
public void periodic() {
    io.updateInputs(inputs);
    updateFuelDetection();

    // periodic() owns motor output. Commands just request state.
    switch (currentState) {
        case IDLE:
            io.stop();
            break;

        case FEEDING:
            io.setConveyorVelocity(Constants.Indexer.CONVEYOR_FORWARD_RPS);
            io.setKickerVelocity(Constants.Indexer.KICKER_FORWARD_RPS);
            // Automatic transition: chute clears, we're done
            if (isChuteEmpty()) {
                currentState = IndexerState.IDLE;
            }
            break;

        case EJECTING:
            io.setConveyorMotor(Constants.Indexer.CONVEYOR_REVERSE_VOLTAGE);
            io.setKickerMotorVolts(Constants.Indexer.KICKER_REVERSE_VOLTAGE);
            break;

        case SENDING_FUEL:
            // ramp logic lives here
            break;
    }

    publishTelemetry();
}

// Commands become tiny — they just request a state.
public Command feed() {
    return Commands.startEnd(
        () -> requestState(IndexerState.FEEDING),
        () -> requestState(IndexerState.IDLE),
        this);
}

// Transition guard — illegal transitions get rejected in one place.
private void requestState(IndexerState requested) {
    // Example: can't go directly from EJECTING to FEEDING
    if (currentState == IndexerState.EJECTING && requested == IndexerState.FEEDING) {
        return;
    }
    currentState = requested;
}
```

The key shifts: motor control consolidates into `periodic()`, commands stop calling `conveyorForward()` directly, illegal transitions get filtered in one place, and reading `periodic()` tells you every possible behavior the subsystem can have.

### Duplicated detection logic — the most concrete piece of debt in the file

`IndexerIOInputs` exposes `chuteDetected` — a boolean from the CANrange's onboard `getIsDetected()`, configured by `ProximityThreshold` and `ProximityHysteresis` in Phoenix Tuner X.

`IndexerSubsystem.periodic()` then does:

```java
isFuelDetected = inputs.chuteDistanceMeters < Constants.Indexer.FUEL_DETECTION_DISTANCE;
```

…which is a *second*, parallel detection path comparing raw distance against a constant in code. Both are present. Only `isFuelDetected` is used downstream. `inputs.chuteDetected` is populated in the inputs struct and never read.

Why this matters: there are now two thresholds — one in Phoenix Tuner X on the sensor, one in `Constants.Indexer.FUEL_DETECTION_DISTANCE`. If those drift apart, the bug will be subtle. And the hysteresis we configured on the sensor is being thrown away because the software comparison doesn't replicate it.

Fix is small: pick one source of truth. Cleanest is to drop the software comparison, use `inputs.chuteDetected` directly, and delete `FUEL_DETECTION_DISTANCE` from Constants since the threshold lives on the sensor now. Roughly a 3-line change with a clean rationale.

### Test profiles shipped in production code

`conveyorReverseThenForwardHold()` and `conveyorPulseProfile()` are tuning experiments — fixed voltage profiles used during initial conveyor characterization. They're still in the subsystem as command factories at the bottom of the file.

These should either move to a separate `IndexerTestCommands.java` (kept around for diagnostics) or be removed. They're not harmful but they pollute the command surface area and signal to students that "test profiles live in the main subsystem file," which sets a bad precedent.

### Leftover debug prints

The kicker config apply blocks have `System.out.println` statements marked `TEMPORARY CHECK TO MAKE SURE MOTOR CONFIGS ARE BEING APPLIED CORRECTLY`. They didn't get removed after the check passed. Two minutes to clean up; flag in the next code review.

### Documentation drift from the IOSim pull

The header comment in `IndexerIO.java` still describes the three-implementation pattern including `IndexerIOSim`. The sim file is gone. Anyone reading this code for the first time will look for a sim file that doesn't exist.

General lesson: when you pull a layer, the docs need to come with it. Add to the code review checklist.

## Open questions for the student owner

Things a code review session should ask the person who owned this subsystem:

1. **What drove `FUEL_CLEAR_TIME = 2 seconds`?** Was there a specific match where a ball briefly cleared and re-triggered the sensor? The value should be documented with its origin story.
2. **Were the popper/pulse test profiles ever used in a real match, or were they purely bench-test features?** If not used, candidate for removal.
3. **Did `feedUntilChuteEmpty` ever exit on timeout (vs sensor) in a real match?** If so, the sensor's reliability is suspect and worth investigating before next season.
4. **Were the loop overruns actually traceable to ioSim, or was something else the bigger contributor?** Worth confirming in the Hoot logs from late-season matches — this determines whether the lesson is "don't add sim under rapid iteration" or something more specific.

## Lessons for next season

1. **Hardware abstraction and simulation are separable.** Adopt them on separate decision cycles. Hardware abstraction earned its keep this season; sim didn't, given our iteration rate.
2. **A state enum is not a state machine.** If `periodic()` doesn't read the state to drive behavior, you have a status indicator — which is fine, just be honest about it. The indexer offseason rewrite is the team's deliberate-practice project for learning the real pattern.
3. **One source of truth for sensor thresholds.** Either the sensor decides or the code decides. Never both.
4. **Pulling a pattern requires pulling its documentation.** Add to code-review checklist.
5. **The `hasSeenFuel + cleared-for-N-seconds` pattern works.** Apply to other "wait for completion" scenarios next season.

---

## Template structure (for the other subsystem entries)

This entry has seven sections. Each subsystem retrospective should hit at least these five:

1. **What this subsystem does** — 1 paragraph, plain English. A drive coach or judge should understand it.
2. **Architecture at a glance** — file structure and what each file owns.
3. **Decisions we made and why** — 2–4 design choices, each with rationale. This is the section that teaches future students.
4. **What worked well** — at least two specific patterns or behaviors, with enough detail that someone could lift them.
5. **What we'd reconsider** — be honest. Include code-level debt (debug prints, dead code, doc drift) and architectural questions. Aim for at least three concrete findings even on subsystems that worked well — if a student can't find three, they probably didn't look hard enough.

Two sections are optional but high-value:

6. **Open questions for the student owner** — drives the actual review conversation. These should be answered *in writing*, in the same file, before the binder is final. That's where the actual learning happens.
7. **Lessons for next season** — what the program (not just this subsystem) should carry forward.
