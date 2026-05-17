# Cross-Cutting Findings — 2026 Season

This document is the team-level retrospective that lives above the file-level retros. The individual retros each cover their own file; this one looks at the patterns that appeared in *multiple* retros and asks what the team can learn at the program level rather than the file level.

Read this *after* several individual retros. The patterns make more sense when you've seen them in context.

## The big themes

There are six recurring negative patterns and four recurring positive patterns. They show up across the codebase, in different specific forms, but they're the same underlying phenomena. Recognizing them as patterns — not as one-off file-level issues — is the team's most important takeaway from the season.

## Negative pattern 1: Abstraction added, old code not deleted

The most common code-debt pattern in the codebase, by a wide margin. It shows up everywhere:

- **Commands retro.** The late-season `AlignAndShootCommand` was extracted from `FuelCommands` as a deliberate refactor. The old alignment logic in `FuelCommands.poseAlign` was never deleted. Result: three or four copies of similar TX-vs-odometry switching code, each subtly different.
- **AutoRoutines retro.** `shootPresetAuton(preset, timeout)` was written as a parameterized auto-shoot factory. The four named wrappers (`shootClose`, `shootTrench`, `shootTower`, `shootFar`) were *not* deleted in favor of calls to the parameterized version. They still exist, with copy-paste duplication.
- **Intake retro.** Roughly 15 fuel-pump command variants exist, most of which are aliases, overloads, or experimental forks of one base implementation. The team intended each new variant to *replace* an earlier one; in practice they accumulated.
- **Indexer retro.** The IO/Hardware split was kept after the IOSim was pulled, but the documentation referencing IOSim wasn't updated.
- **Constants retro.** Two `PASS_RPM` constants exist — one in `Flywheel` (value `0`, unused) and one in `Shooter` (value `4004`, used). The Flywheel one is a ghost from before the constants got reorganized.
- **RobotContainer retro.** The auto chooser has ~13 commented-out routine bindings — routines that exist in `AutoRoutines.java` but never shipped to the drive coach. They accumulated as the team experimented and refined the auto strategy.

**Why this happens:** under build-season pressure, the path of least resistance is "add the new thing, comment out the old thing, ship it." Deleting feels riskier — what if we need the old version back? — so it's deferred. Once deferred, it's never reclaimed.

**The systemic fix is a code review rule:** when a PR introduces a new way to do something that already has a way, the PR must either (a) delete the old way, or (b) document why both must exist. The default is delete. The middle state — both exist, no rationale — is not acceptable. This rule alone would have prevented half the findings in the file-level retros.

## Negative pattern 2: Test commands shipping in production

Almost every subsystem has tuning/experimental commands mixed into the production command surface:

- **Intake**: `conveyorReverseThenForwardHold`, `conveyorPulseProfile`, `fuelPumpSetCycles`, `fuelPumpSetCyclesAlpha`, `fuelPumpBasic`.
- **Shooter**: `tuneFlywheelCommand` — better-named than most, but still in the production file.
- **Indexer**: test profiles for conveyor tuning.
- **AutoRoutines**: experimental routines mixed with deployed routines (though the `Test` inner class is an attempt at quarantine).
- **PitTests_EXPERIMENTAL**: the file name explicitly marks it experimental but it was never wired up — it's the inverse problem (test infrastructure that didn't ship).

**The cost:** students reading a subsystem file can't tell which commands are bound to buttons vs which are bench-tuning artifacts. Test commands signal to rookies "this is normal" and the next season accumulates more. Test commands are also a maintenance burden — they consume code review attention and have to keep compiling even when no one's running them.

**The fix:** every subsystem gets a separate `SubsystemNameTestCommands.java` file (or equivalent), explicitly excluded from teleop bindings. New experimental commands go there from the start, get promoted to production only after explicit review. The `PitTests_EXPERIMENTAL` file is the closest existing example of the right pattern — finish wiring it up for 2027.

## Negative pattern 3: Missing logging in fusion and decision code

The most consistently-cited diagnostic gap across the retros:

- **Robot.java (vision fusion).** Battery and CAN utilization are logged. *Which* vision estimator was used, *what* the std-dev was, *whether* a measurement was rejected (and why) are not. Vision-debugging from logs is impossible.
- **Drivetrain (path following).** `m_pathXController` computes errors internally but those errors are not published anywhere. Path-following debugging requires manual trajectory-vs-odometry correlation.
- **PhoenixUtil.** The retry logic either succeeds silently or fails loudly. The "took 3 retries to succeed" middle case — diagnostic gold — is silent.
- **Vision subsystem.** The `txDegrees` value is logged but the *decision* to use TX vs odometry bearing (covered by the multi-tag flip strategy) is not. When alignment goes wrong, you can't tell whether the TX/odometry switch fired correctly.
- **Indexer / Intake.** Detection and state-machine transitions are mostly visible via raw sensor signals, but the *decisions* made on top of them ("hasSeenFuel" guard fired, "chute cleared" condition met) aren't logged distinctly.

**The principle:** anywhere code makes a decision based on multiple inputs, log the decision. Don't just log the inputs — log "we chose A because B." This converts log review from forensic analysis ("the inputs were these, what did the code do with them?") to direct read ("the code chose A, so we know it took this branch").

Five lines of `Logger.recordOutput` calls in each decision-making method would transform debugging across the codebase. This is the single highest-leverage change for 2027 reliability, and it's nearly free to implement.

## Negative pattern 4: Documentation drift after structural changes

When a layer is pulled or a class is renamed, the documentation referring to it is rarely updated:

- **IO interfaces** (indexer, intake, shooter, vision) all have header comments referencing `*IOSim` files that no longer exist.
- **FuelCommands** has a "TUNING NOTES" block referencing NT keys like `Vision/IsAligned` and `Vision/HorizontalAngle_deg` that don't match the actual publish names.
- **FuelCommands** has a "RobotContainer.java NamedCommands" example referencing a `ShooterCommands` class that doesn't exist.
- **RobotContainer** has commented-out `autoRoutines.test::Bulldozer` that wouldn't compile if uncommented (the class structure drifted).
- **Indexer** doc references the three-implementation pattern after it became a two-implementation pattern.
- **Intake's IOHardware** header says "All telemetry logged via AdvantageKit" after AdvantageKit was substantially scaled back.

**Why this happens:** structural changes are surgical (touch a few files), but the documentation about that structure is distributed. Updating it requires cross-file awareness that's hard to maintain under pressure.

**The fix:** add to the code review checklist — *"When this PR removes or restructures a file/class/layer, search the codebase for references to the old name/structure. Update or remove them."* Two minutes per PR; saves hours of student-time over a season.

## Negative pattern 5: Commented-out code accretion

Every file has it. The intake has commented-out roller-follower configs. The indexer has commented-out IOSim references. The shooter has commented-out config values. The commands have commented-out PASS-shot bindings. The auto routines have commented-out alternative behaviors. The constants have commented-out alliance positions. RobotContainer has commented-out auto-chooser bindings.

The *pattern* is identical everywhere: someone had an idea, tried it, didn't commit, but didn't delete either. The code lives in a half-state — "maybe needed later" — that's the worst of both worlds. Students reading the file can't tell what's almost-active vs definitely-dead.

**The fix:** code review rule — commented-out code older than 30 days gets deleted. Git history preserves it. If something is genuinely "we might want this back," put it behind a feature flag or in a separate file, not as `//`-prefixed lines in the production file.

The shooter and vision retros both noted this. The intake retro counted ~14 fuel-pump variants partly because of commented-out aliases. The auton retro identified entire commented-out routine families. This is the most pervasive style issue in the codebase, and the easiest to fix institutionally.

## Negative pattern 6: Tunable values living outside Constants

The codebase has a Constants file. Most tunable values live there. Several don't:

- **CommandSwerveDrivetrain.followPath** has the path-PID gains (`new PIDController(10, 0, 0)`) inline.
- **AutoRoutines** has timeout constants as static fields on the class (`shootTimeout`, `intakeTimeout`, `intakeDelay`).
- **FuelCommands** has at least one hardcoded value inside a builder (`.40` for `ALIGNMENT_DRIVETRAIN_CLAMP` is in Constants, but `0.5` for stop-before-shoot timeout is inline).
- **Intake** has the `intakeDelay - 0.15` arithmetic inline in auto routines — what's `0.15`?

**Why this matters:** tuning is a search-and-edit activity during build season. When the drive team asks for a slightly-faster path or a slightly-longer settle, "where do I change that?" should have one answer: Constants. When values are scattered across subsystem and command files, tuning becomes a grep exercise, and during a regional event under time pressure, the wrong value gets changed or the right one gets missed.

**The fix:** "if it could plausibly need re-tuning at an event, it goes in Constants" as a team rule. The drivetrain retro flagged this most concretely; the same rule applies everywhere.

## Positive pattern 1: Defensive coding for real failure modes

The strongest engineering in the codebase. Every major subsystem has at least one example:

- **Drivetrain**: `resetPoseFromVisionCommand` recovers from wheel-slip on game pieces. Has omega-filter guard and dashboard feedback.
- **Drivetrain**: Pigeon reconnection re-seeds the field-centric heading. Most teams don't think about this.
- **Intake**: software soft limits on the slide. Prevents mechanical crashes even with buggy commands.
- **Shooter**: `eject()` refuses to enter EJECT mode if the flywheel is spinning too fast. Prevents violent reversal.
- **Vision**: alliance-aware hub tag filtering. Won't align to a non-hub AprilTag.
- **Robot.java**: omega gate on vision fusion. Won't fuse during rapid rotation when MT2 yaw seeding is unreliable.

**The principle:** real robots fail in specific ways (wheel slip, sensor dropouts, IMU drift, spinning the wrong direction). Code that anticipates these failures and recovers gracefully — *with appropriate guards on the recovery* — is the difference between "works on the bench" and "works in elim 3 at champs."

**Lift this pattern to a team rule:** for every command that takes a significant action, ask "what's the worst thing this could do if called at the wrong time, and is there a guard?" The shooter's eject-velocity check and the intake's soft limits are the templates.

## Positive pattern 2: Sign-convention and tribal-knowledge preservation in comments

Comments that preserve real engineering knowledge at the point of use:

- **AlignAndShootCommand**: *"Sign confirmed on robot: negative = correct for rear-mounted camera."*
- **RobotContainer**: *"It is critical that these inputs are (-). Do not change them."*
- **CommandSwerveDrivetrain.stop()**: *"Cannot use a runOnce()"* with explanation.
- **ShooterIOHardware**: *"setUpdateFrequency calls MUST come AFTER optimizeBusUtilization() or they will be cleared."*
- **ShooterIOHardware**: *"Followers MUST be set AFTER optimizeBusUtilization() — aggressive frame disabling can break the follower control link if set before."*
- **VisionSubsystem**: *"tagCount == 1 → TX is the best rotation reference; tagCount >= 2 → odometry bearing is more stable."*
- **Robot.java vision fusion**: *"Theta std dev = 9999999: gyro always owns heading."*

**Why this works:** these comments are at the point where a future student would naturally try to "clean up" something that looks unnecessary. The comment says *"this looks unnecessary but it isn't, here's why."* It survives turnover.

**Lift this as a code review standard:** if a piece of code looks weird-but-correct, the PR must include a comment explaining the weirdness. Counterintuitive code without explanation is a regression vector.

## Positive pattern 3: Hardware-abstraction split survived; simulation didn't

Across every subsystem with an IO pattern (indexer, intake, shooter, vision), the same decision was made: keep the `IOHardware` split, pull the `IOSim` layer. This wasn't a coincidence — it was the same engineering judgment applied independently in multiple places.

The reasoning, articulated in the indexer retro and repeated across the others: **hardware abstraction and simulation are separable.** Hardware abstraction earned its keep — it made code review cleaner, made hardware swapping easier, made the IOInputs struct a single point of telemetry. Simulation didn't — it added a third file to maintain per subsystem, and the team's rapid mechanical iteration meant sim models had to be updated as often as the real hardware, which doubled the work.

**Lift this principle:** patterns are usually bundles. Adopt the bundle one piece at a time. The IO pattern from 6328 is "interface + hardware + sim"; the team adopted "interface + hardware" and got most of the value at half the cost.

## Positive pattern 4: Dependency injection through suppliers

The vision subsystem and the alignment commands all use `DoubleSupplier` parameters for values that come from other subsystems (yaw, yaw rate, joystick inputs). This avoids hard coupling — `VisionSubsystem` doesn't know about `CommandSwerveDrivetrain`, it just consumes lambdas that happen to read from the drivetrain.

The benefit: testability (a unit test or sim can supply any source), readability (the constructor signature documents the dependencies), and modularity (subsystems compose without circular references).

This pattern wasn't applied everywhere — `Robot.java` reaches into `m_robotContainer.drivetrain` directly, for example — but where it was applied, it was applied cleanly.

**Lift this principle:** when a subsystem needs data from another subsystem, prefer `Supplier<T>` over a hard reference. The performance cost is negligible; the architectural payoff compounds.

## Themes that didn't make it to a pattern (yet)

Honorable mentions — observations that appeared in only one or two retros but might be worth watching:

- **AI-assisted code quality is uneven within a single file.** The shooter's `setHoodVoltage` is declared in the interface but not implemented; `flywheelCurrentAmps` is declared in inputs but never populated. These have the texture of AI-generated scaffolding that wasn't fully wired up. Worth tracking for 2027: when AI helps write code, does someone verify every declared field/method has a real implementation?
- **Two-tolerance design.** AlignAndShootCommand has two tolerance values (alignment for PID deadband, feed for fire gate). The "tight tolerance for control, loose tolerance for action" pattern likely generalizes — anywhere "is X done enough to act?" is a question, the answer might be two thresholds, not one.
- **Single-point-of-fusion as an architectural principle.** Vision fusion calls `addVisionMeasurement` in exactly one place (Robot.java). The vision subsystem deliberately doesn't. This is the cleanest design decision in the codebase. Generalize to: any sensor that feeds a Kalman filter should have one fusion call site.

## What this all adds up to

The 2026 codebase has real engineering strengths and real engineering debt. The strengths are concentrated in the late-season work — the shooter's state machine, the vision fusion math, the drivetrain's recovery commands, the alignment refactor. The debt is concentrated in the accreted/copy-pasted layers — the auto routines, the fuel-pump variants, the dead bindings, the commented-out alternatives.

The team is in a strong position for 2027 because the patterns that work are now visible and reusable, and the patterns that don't work are now identified and pruneable. The offseason backlog captures both — the new things to do well and the old things to clean up.

The single most important meta-lesson, repeated across nearly every retro:

> **Complexity must earn its place. When complexity is added, the simpler approach must be removed. When complexity stops earning, the team is allowed to remove it.**

That principle, applied consistently, would have prevented most of the negative findings in this binder.

For 2027: institutionalize it. Code review rule, build-season discipline, end-of-season cleanup pass. The team doesn't lack engineering judgment — the retros prove that. What it lacks is the *enforcement* of judgment under pressure. Make the rule, follow the rule, retro on the rule next May.
