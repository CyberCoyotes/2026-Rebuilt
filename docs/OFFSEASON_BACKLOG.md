# Cyber Coyotes — 2026 Offseason Backlog

This document is the prioritized work plan for the post-season period (May 2026 – kickoff 2027). Every "Lessons for next season" item from the retros is captured here, ordered by a combination of impact and feasibility, and scoped well enough to file as a GitHub issue.

The items are grouped by tier. Within each tier, ordering is rough — student availability and dependencies will reshape the actual sequence.

## How to use this backlog

For the programming lead (student):
- The top-tier items (P0) are the most important offseason work. Pick one and commit to seeing it through.
- The P1 items are the highest-leverage cleanups — they're smaller and discrete enough to use as "training projects" for newer programmers.
- The P2 items are nice-to-haves that can slide to early build season if needed.
- The P3 items are aspirational — flagged for visibility, not commitment.

For the programming mentor:
- Each item links back to the retros that surfaced it. When students push back on priority, the source retros provide the rationale.
- The "Effort" estimate is rough — measured in student-afternoons (~3-4 hours of focused work).
- Items marked `[teaching]` are particularly good for instructional use because the work itself teaches the principle being applied.

## Tier P0 — Critical for 2027 readiness

These three items are large enough to be the *named offseason project* for whichever student takes them on.

---

### P0-01 — Rewrite Indexer as a real FSM `[teaching]`

**Source retros:** `RETROSPECTIVE_indexer.md`, `RETROSPECTIVE_shooter.md` (as reference implementation), `RETROSPECTIVE_crosscutting.md`.

**Problem statement:**
The indexer's `IndexerState` enum was intended to be a full FSM but became a status indicator under build-season pressure. `periodic()` doesn't read `currentState` to drive behavior — motors are set directly in command lambdas, with `setState(...)` called alongside as a label. The enum gives compile-safe state names for telemetry but doesn't deliver the FSM benefits (illegal transition prevention, consolidated motor control, automatic state promotion).

The team identified this mid-season and committed to an offseason rewrite as deliberate practice.

**Acceptance criteria:**
- `periodic()` is the only place that drives motor output, switching on `currentState`.
- Commands call `requestState(...)` only — they don't call `conveyorForward()`, `kickerForward()`, etc. directly.
- A single `requestState(...)` method enforces transition guards (e.g., refuse IDLE → SENDING_FUEL).
- Automatic transitions (e.g., `FEEDING → IDLE` on chute clear) live in `periodic()`.
- The duplicated detection logic (`chuteDetected` from sensor vs `isFuelDetected` from code threshold) is consolidated into one source of truth.
- Test profiles (`conveyorReverseThenForwardHold`, `conveyorPulseProfile`) are moved to `IndexerTestCommands.java` or deleted.
- IOSim documentation drift is fixed (or the references removed).
- All existing button bindings continue to work; no driver-facing behavior changes unless intentional.

**Reference:** `ShooterSubsystem.java` is the model. Read it carefully before starting; the indexer rewrite should mirror its structure.

**Effort:** 4–6 student-afternoons.

**Dependencies:** None. Start whenever.

---

### P0-02 — Extract AlignmentCalculator helper

**Source retros:** `RETROSPECTIVE_commands.md`, `RETROSPECTIVE_crosscutting.md`.

**Problem statement:**
The alignment logic (TX-vs-odometry switching, heading PID with continuous input, minimum-rotation-rate floor, two-tolerance feed gate) exists in 3–4 independent copies:

1. `AlignAndShootCommand.execute()` — canonical implementation
2. `AlignOnlyCommand.execute()` — near-verbatim copy
3. `FuelCommands.Auto.poseAlignAndShoot` inner `Commands.run` body
4. `FuelCommands.poseAlign` — older simpler version

Any tuning change has to be made in 3–4 places. Bug fixes drift between copies.

**Acceptance criteria:**
- New class `AlignmentCalculator` (or similar) in `commands/util/` or equivalent location.
- Static methods for `computeTargetHeading(pose, hub, vision, offsetDeg)` and `computeRotationRate(pid, currentHeading, targetHeading, minRate, maxRate)`.
- All four call sites use the helper. No duplicate TX-vs-odometry switching code remains in any caller.
- The older `FuelCommands.poseAlign` factory is either updated to use the helper *or* deleted if no longer referenced.
- All call sites continue producing identical behavior — no functional regressions.

**Reference:** The commands retro has a code sketch of the proposed `AlignmentCalculator` API.

**Effort:** 3–4 student-afternoons. Half of that is regression testing.

**Dependencies:** None functionally, but this should be done *before* any further alignment tuning so the tuning lands in one place.

---

### P0-03 — Build an AutoRoutineBuilder API and migrate `AutoRoutines.java`

**Source retros:** `RETROSPECTIVE_auton.md`, `RETROSPECTIVE_robotcontainer.md`, `RETROSPECTIVE_crosscutting.md`.

**Problem statement:**
`AutoRoutines.java` is 1143 lines. Most of that is copy-paste duplication between Lt/Rt mirror routines and Single/Double cycle variants. Adding a new routine is a 30-line copy-paste activity. RobotContainer's auto chooser only wires up ~7 of the dozens of routines — most are dead code.

A thin builder API would compress the routine declarations to ~5 lines each, reducing the file to ~200 lines and making routine additions trivial.

**Acceptance criteria:**
- New class `AutoRoutineBuilder` with a fluent API (e.g., `.driveAndIntake(...)`, `.drive(...)`, `.stopAndShoot()`, `.driveTo(...)`).
- All shipped routines (confirmed via the auto chooser in RobotContainer) reimplemented using the builder.
- Dead routines (not referenced in any active chooser binding) are *deleted*, not migrated.
- Lt/Rt mirror pairs become a single `trenchRampSingle(Side side)` method with a `Side` enum.
- The Choreo event-marker pattern is preferred over `Commands.deadline + intakeFuelTimer` for new routines — see Lessons.
- File size reduced from 1143 lines to <300 lines.

**Reference:** The auton retro has a code sketch of the proposed builder API.

**Effort:** 6–8 student-afternoons. This is the largest single item in the backlog.

**Dependencies:** Should be done after P0-02 (AlignmentCalculator) so the auto-shoot path uses the consolidated alignment code. Could be done in parallel by a different student.

---

## Tier P1 — High-value cleanups

Discrete, scoped items that produce visible improvements. Several of these are great training projects for newer programmers because the work itself teaches a real principle.

---

### P1-01 — Add fusion-decision logging to Robot.java vision fusion

**Source retros:** `RETROSPECTIVE_robot.md`, `RETROSPECTIVE_vision.md`, `RETROSPECTIVE_crosscutting.md`.

**Problem statement:**
The vision pose-fusion code in `Robot.robotPeriodic()` makes decisions every cycle — which estimator to use (MT2 vs MT1), whether to reject (omega gate, no tags), what std-dev to apply — but logs none of those decisions. When a match goes badly and the pose estimate looks wrong, replay can't answer "did vision fire at this moment, and with what confidence?"

**Acceptance criteria:**
- New `Logger.recordOutput` calls for: `UsedEstimator` (string: "MT2" | "MT1" | "Rejected:Spinning" | "Rejected:NoTags"), `XYStdDev`, `TagCount`, `AvgTagDistM`, `OmegaRps`.
- Existing fusion behavior unchanged.
- Logged values visible in AdvantageScope when replaying a Hoot log.

**Effort:** 1 student-afternoon.

**Dependencies:** None.

---

### P1-02 — Fix duplicate vision reads (cache in VisionSubsystem, read from cache in Robot.java)

**Source retros:** `RETROSPECTIVE_robot.md`, `RETROSPECTIVE_vision.md`.

**Problem statement:**
`VisionIOLimelight.updateInputs()` reads MT2 and MT1 every cycle. `Robot.robotPeriodic()` reads them *again* via `LimelightHelpers` directly. Each read is a NetworkTables parse with non-trivial CPU cost. Given the team's loop-overrun struggles, this is a concrete place to save loop time.

**Acceptance criteria:**
- `VisionSubsystem` exposes `getMegaTag2Estimate()` and `getMegaTag1Estimate()` returning cached `LimelightHelpers.PoseEstimate` values populated in its `periodic()`.
- `Robot.robotPeriodic()` uses the cached values instead of calling `LimelightHelpers` directly.
- Vision data is read from NetworkTables exactly once per cycle.
- Vision fusion behavior unchanged.

**Effort:** 1–2 student-afternoons.

**Dependencies:** Should follow P1-01 so the logging is in place to verify behavior is unchanged.

---

### P1-03 — Add CAN bus optimization helper to PhoenixUtil and apply to all subsystems `[teaching]`

**Source retros:** `RETROSPECTIVE_shooter.md`, `RETROSPECTIVE_utilities.md`, `RETROSPECTIVE_crosscutting.md`.

**Problem statement:**
`ShooterIOHardware` is the only subsystem using `optimizeBusUtilization` + `setUpdateFrequencyForAll` for CAN bus management. The pattern is sophisticated, documented (including two gotchas about ordering), and likely contributed to the shooter's loop-time profile being healthier than other subsystems. The same pattern would benefit indexer, intake, and drivetrain.

**Acceptance criteria:**
- New method `PhoenixUtil.optimizeAndSetFrequencies(device, signalsByFrequency)` (signature TBD by author).
- Helper handles both gotchas internally: setUpdateFrequency after optimize, followers configured after optimize.
- `ShooterIOHardware` migrated to use the helper. Behavior unchanged.
- `IndexerIOHardware`, `IntakeIOHardware` adopted to use the helper for their motors.
- Loop overrun frequency measurably reduced (target: ~50% reduction in overruns under load).

**Effort:** 2–3 student-afternoons.

**Dependencies:** None.

**Teaching value:** This is the canonical "lift a working pattern from one subsystem to many" exercise.

---

### P1-04 — Move path PID gains to Constants

**Source retros:** `RETROSPECTIVE_drivetrain.md`, `RETROSPECTIVE_crosscutting.md`.

**Problem statement:**
`CommandSwerveDrivetrain.followPath` instantiates path-following PIDs with hardcoded gains:
```java
private final PIDController m_pathXController = new PIDController(10, 0, 0);
private final PIDController m_pathYController = new PIDController(10, 0, 0);
private final PIDController m_pathThetaController = new PIDController(7, 0, 0);
```

Tuning these requires editing the subsystem file. Inconsistent with the rest of the codebase, where tunable values live in `Constants`.

**Acceptance criteria:**
- New constants `Constants.Drivetrain.PATH_PID_KP`, `PATH_PID_KP_ROT`, etc.
- `CommandSwerveDrivetrain` reads from Constants.
- `enableContinuousInput` moved from `followPath()` (called every cycle) to the constructor or static init (called once).
- Path-PID error logging added (`Logger.recordOutput("Path/XError_m", ...)` etc.) per the drivetrain retro.

**Effort:** 1 student-afternoon.

**Dependencies:** None.

---

### P1-05 — Prune the fuel-pump command explosion in IntakeSubsystem `[teaching]`

**Source retros:** `RETROSPECTIVE_intake.md`, `RETROSPECTIVE_crosscutting.md`.

**Problem statement:**
`IntakeSubsystem` has ~15 fuel-pump command variants. Some are aliases of others. At least two (`fuelPumpSetCycles`, `fuelPumpSetCyclesAlpha`) are byte-identical and share a wrong `.withName` value. The accreted command surface confuses students and pollutes Glass / AdvantageScope command tracking.

**Acceptance criteria:**
- Walk `RobotContainer.java` and `AutoRoutines.java` for every `fuelPump*` reference. Build the "actually used" list.
- Commands not on the list are either *deleted* or *moved to `IntakeTestCommands.java`* with a clear comment about their purpose.
- Identical duplicate methods are reconciled — keep one, delete the other.
- `.withName` values match the method name unless there's a documented reason otherwise.
- The misplaced `.finallyDo()` in `fuelPumpSetCycles` is fixed (`finallyDo` should attach to the outer `Commands.sequence`, not the final `waitSeconds`).
- File line count drops by 200+ lines.

**Effort:** 2–3 student-afternoons.

**Dependencies:** None.

**Teaching value:** This is the canonical "code review reveals what's actually used vs what's just present" exercise.

---

### P1-06 — Clean up RobotContainer dead bindings and AutoRoutines orphaned routines

**Source retros:** `RETROSPECTIVE_robotcontainer.md`, `RETROSPECTIVE_auton.md`.

**Problem statement:**
RobotContainer's auto chooser has ~13 commented-out routine bindings. AutoRoutines.java has corresponding orphaned routine methods. The driver `a` button is commented out. Operator rightTrigger, povLeft, povRight are commented out. These accumulate "dead bindings" that confuse readers.

**Acceptance criteria:**
- Pull FMS data for 2026 events to confirm which auto routines were actually selected.
- Routines in AutoRoutines.java that were never selected: delete (or move to `docs/autonomy-2026-history.md` if archaeologically interesting).
- Routines that were never wired to the chooser: delete (after confirming with team they weren't held back for a reason).
- The driver `a` button: either bound to something (PASS shot, x-brake, etc.) with documentation, or the commented lines are deleted.
- Operator unused triggers: same treatment.
- The double `xBrake` declaration in RobotContainer is collapsed to one.

**Effort:** 1–2 student-afternoons. Most of the time is in the FMS data pull and team confirmation.

**Dependencies:** Should precede P0-03 (AutoRoutineBuilder migration) so the builder doesn't migrate code that's about to be deleted.

---

### P1-07 — Populate or remove dead IOInputs fields

**Source retros:** `RETROSPECTIVE_shooter.md`, `RETROSPECTIVE_intake.md`, `RETROSPECTIVE_crosscutting.md`.

**Problem statement:**
Several IOInputs fields are declared but never populated by IOHardware:
- `ShooterIOInputs.flywheelCurrentAmps` — declared, never refreshed. Makes `isOverCurrent()` silently always false.
- `ShooterIOInputs.hoodAppliedVolts`, `hoodCurrentAmps`, `hoodAngleDegrees` — declared, never populated.
- `ShooterIO.setHoodVoltage` — declared in interface, no hardware implementation.
- `ShooterIO.setFlywheelVelocityTorqueFOC` — declared in interface, no hardware implementation.

Each is a landmine waiting for a future student to read the field as authoritative when it's not.

**Acceptance criteria:**
- For each dead field: populate it correctly OR delete it from IOInputs.
- For each dead interface method: implement it OR delete it from the interface.
- `isOverCurrent()` either works correctly or is deleted.
- Add a code review rule: "A field in IOInputs without a populator is a build error or a code-review reject."

**Effort:** 1–2 student-afternoons.

**Dependencies:** None.

---

### P1-08 — Fix the indexer's duplicated detection logic

**Source retros:** `RETROSPECTIVE_indexer.md`.

**Problem statement:**
Two parallel detection paths exist in the indexer:
- `inputs.chuteDetected` — boolean from the CANrange's onboard `getIsDetected()`, configured by `ProximityThreshold` in Phoenix Tuner X.
- `isFuelDetected` — computed in `IndexerSubsystem.periodic()` by comparing `inputs.chuteDistanceMeters` against `Constants.Indexer.FUEL_DETECTION_DISTANCE`.

Only `isFuelDetected` is used downstream. The hysteresis configured on the sensor is being thrown away because the software comparison doesn't replicate it.

**Acceptance criteria:**
- Pick one source of truth (recommend the sensor-side `chuteDetected` since hysteresis is configured there).
- Delete the unused path entirely.
- Delete the now-unused `Constants.Indexer.FUEL_DETECTION_DISTANCE` if applicable.
- Behavior verified unchanged on bench.

**Effort:** 0.5 student-afternoon. (May seem trivial; including for completeness.)

**Dependencies:** Best done as part of P0-01 (Indexer FSM rewrite) but could be done standalone.

---

### P1-09 — Wire up PitTests_EXPERIMENTAL and extend coverage

**Source retros:** `RETROSPECTIVE_commands.md`.

**Problem statement:**
`PitTests_EXPERIMENTAL.java` is well-designed (uses production code paths, auto-extends via `enum.values()`) but never wired to a button. The team did pit checks manually that this would have automated.

**Acceptance criteria:**
- `PitTests.testAll(...)` bound to a button on a debug controller or dashboard widget.
- Equivalent test sequences added for intake (slide extend/retract, roller forward/reverse) and indexer (conveyor forward/reverse, kicker forward/reverse).
- Each test sequence logs pass/fail to dashboard so the pit team sees "Pit test: TOWER preset — PASSED at 4.2s."
- File renamed from `PitTests_EXPERIMENTAL.java` to `PitTests.java` when ready.

**Effort:** 2 student-afternoons.

**Dependencies:** None.

---

## Tier P2 — Nice-to-haves

Items worth doing but not blocking. Can slide to early build season if needed.

---

### P2-01 — Constants tuning-history cleanup

**Source:** `RETROSPECTIVE_constants.md`.

Per-constant tuning-history comments (`// Was 1.5`, `// 0.5 too quick`, `// At States running 1.0`) pollute the constants file. Replace with `// Tuned at <event> <date>` line-end markers, with full history relegated to git log.

**Acceptance criteria:** Tuning-history block comments removed. Surviving comments use the line-end "Tuned" pattern from the Flywheel section.

**Effort:** 1 student-afternoon.

---

### P2-02 — Commented-out code purge across codebase

**Source:** `RETROSPECTIVE_crosscutting.md`.

Walk every retro'd file. Delete commented-out code older than 30 days unless it has a documented "we might need this" reason. Git preserves history for everything else.

**Acceptance criteria:** Commented-out method bodies and config alternatives removed across all subsystem and command files.

**Effort:** 1–2 student-afternoons across the whole codebase.

---

### P2-03 — Vision distance characterization

**Source:** `RETROSPECTIVE_shooter.md`, `RETROSPECTIVE_vision.md`.

The shooter's vision lookup table has only 3 active distance entries (`CLOSE`, `TOWER`, `FAR`) with several commented-out distances. Sparse data means linear interpolation does most of the work between widely-spaced anchors.

**Acceptance criteria:** Practice-field session to measure RPM and hood at 5–7 additional distances. Constants updated. `TRENCH` entry restored (it was commented out as "might add confusion" — re-evaluate).

**Effort:** 2–3 hours of practice-field time, 0.5 student-afternoon to update code.

---

### P2-04 — Add `getDistanceToHubMeters()` to VisionSubsystem

**Source:** `RETROSPECTIVE_vision.md`, `RETROSPECTIVE_shooter.md`.

Distance to hub is currently computed in `AlignAndShootCommand` from the fused pose. If multiple consumers need it (shooter, LED feedback, alignment commands), they should all read from one canonical source — `VisionSubsystem.getDistanceToHubMeters()`.

**Acceptance criteria:** Method added. AlignAndShootCommand uses it. Logged via AKit.

**Effort:** 1 student-afternoon.

---

### P2-05 — Fix `hasFreshTarget()` to actually check freshness

**Source:** `RETROSPECTIVE_vision.md`.

The method name implies a temporal check; the implementation is purely a validity check. A stale Limelight frame with a hub tag would still report `hasFreshTarget = true`.

**Acceptance criteria:** Method either renamed to `hasValidHubTarget()` OR add a real freshness check based on `inputs.totalLatencyMs` and a staleness threshold.

**Effort:** 0.5 student-afternoon.

---

### P2-06 — Migrate auto routines to Choreo event markers `[teaching]`

**Source:** `RETROSPECTIVE_auton.md`.

Most auto routines use `Commands.deadline(trajectory.cmd(), intakeFuelTimer(...))` for parallel intake during drive. The more idiomatic Choreo pattern is `trajectory.atTime("Intake").onTrue(...)`. `RtBulldozer` uses the event-marker pattern; everything else uses the deadline pattern.

**Acceptance criteria:** Either standardize on event markers (preferred — trajectory-relative timing scales with retunes) and update trajectory files with named events, OR document why the deadline pattern is being kept.

**Effort:** 2–3 student-afternoons. Most of the time is in Choreo trajectory file editing.

**Dependencies:** Should be done in conjunction with P0-03 (AutoRoutineBuilder).

---

### P2-07 — Operator preset-display documentation and LED feedback

**Source:** `RETROSPECTIVE_shooter.md`, `RETROSPECTIVE_robotcontainer.md`.

The shooter has a `setDisplayPreset()` mechanism that shows the operator's selected preset without firing. The feature is built but appears underused. If the LED subsystem is brought back for 2027, this is a natural integration point (LED color = current preset).

**Acceptance criteria:** Document the feature in the operator handbook. If LED is built, integrate.

**Effort:** Depends on LED status.

---

## Tier P3 — Aspirational / 2028 territory

Flagged for visibility. Probably won't happen in 2027 but worth keeping on the radar.

---

### P3-01 — Multi-Limelight architecture

**Source:** `RETROSPECTIVE_vision.md`.

Reference team 2056 runs 2-3 Limelights with role specialization. Current code is single-Limelight. `VisionIOLimelight(String limelightName)` is set up to support multi-instance, but `VisionSubsystem` doesn't compose multiple yet.

**Effort:** Significant (multiple weeks of design + implementation + tuning).

---

### P3-02 — CANivore migration for flywheel

**Source:** `RETROSPECTIVE_shooter.md`.

`setFlywheelVelocityTorqueFOC` is declared in the interface but never implemented because the flywheel motors are on RIO CAN (TorqueCurrentFOC requires CAN FD). Migration to CANivore would unlock torque-current control.

**Effort:** Hardware change + code wire-up. Depends on whether the team prioritizes the additional control fidelity.

---

### P3-03 — Vision/odometry disagreement detection

**Source:** `RETROSPECTIVE_vision.md`.

A common pose-estimator failure mode: wheel odometry and vision disagree by a lot (wheel slip, vision error, tag confusion). Computing and logging this disagreement is a powerful diagnostic.

**Effort:** 1 student-afternoon.

**Why P3:** small-effort but only valuable if someone actually reviews the logs. Promote to P1 if "vision behaving weirdly" becomes a recurring concern.

---

### P3-04 — Vision-side IMU readiness gate

**Source:** `RETROSPECTIVE_vision.md`.

`SetRobotOrientation` is called every cycle from `periodic()` starting at boot. If the gyro isn't fully initialized at that point, the first few MT2 reads are seeded with garbage yaw.

**Effort:** 0.5 student-afternoon.

---

### P3-05 — Logger metadata for log archaeology

**Source:** `RETROSPECTIVE_robot.md`.

`Logger.recordMetadata("ProjectName", "2026 Rebuilt")` is the only metadata recorded. Adding git commit hash, branch name, build timestamp, RoboRIO image version would let the team look at a log from a year ago and know exactly what code was running.

**Effort:** 0.5 student-afternoon.

---

## Process recommendations (not code items)

These aren't backlog items per se — they're institutional changes that prevent the recurring patterns identified in the cross-cutting findings.

### Code review checklist additions

1. **"When this PR introduces a new way to do something, does it delete the old way or document why both must exist?"**
2. **"When this PR removes or restructures a file/class/layer, are references to the old name/structure updated?"**
3. **"Are commented-out blocks in this PR older than 30 days? Delete them; git preserves history."**
4. **"For tunable values: are they in Constants?"**
5. **"For new IO fields and interface methods: is there a populator/implementation?"**
6. **"For safety-critical commands: is there a guard on the precondition?"**

### Build-season discipline

- **Weekly "dead code" sweep.** Friday afternoon, 15 minutes, walk one file, delete or restore everything commented out for >7 days.
- **Pre-event command-surface audit.** Before each event, grep `RobotContainer` for active bindings, walk the matching command factories, confirm no dead code is shipping.
- **Match-debrief auton retrospective.** Which routine was selected, what happened, what to change. Capture in a running doc.

### End-of-season ritual

- **Run the retrospective process again.** Use this binder as the template. The artifact should exist by mid-May.
- **File next-season's backlog items as GitHub issues**, not just in a markdown doc. Track them in the existing GitHub Projects setup.
- **Onboard new students with the previous-season's retro binder**, not with a "here's how the code works" walkthrough. The retros teach *why* more effectively than the code does.

---

*Last updated: May 2026. Authored as part of the 2026 post-season retrospective process.*
