# RobotContainer — 2026 Season Retrospective

You asked for the Good, Bad, and Ugly of `RobotContainer.java`. The file is 266 lines (209 LOC) — small for what it does. It instantiates every subsystem, wires up the auto chooser, and binds every button to every command. Driver UX is decided here.

There's a real story in this file that the other retros only hinted at: **the auto chooser is the source of truth for what shipped.** Most of what's in `AutoRoutines.java` never made it to a real match because it was never added to (or was commented out of) the chooser here.

## The Good

### Subsystem instantiation is clean and consistent

```java
indexer = new IndexerSubsystem(new IndexerIOHardware());
intake  = new IntakeSubsystem(new IntakeIOHardware());
shooter = new ShooterSubsystem(new ShooterIOHardware());
```

Three lines, identical pattern. The IO-pattern adoption (subsystem accepts an IO interface, hardware impl is constructed inline) is visible right here. Anyone reading the file sees "this team takes IO abstraction seriously" within 30 seconds.

### Vision instantiation demonstrates the dependency-injection pattern from the vision retro

```java
vision = new VisionSubsystem(
    new VisionIOLimelight(Constants.Vision.LIMELIGHT4_NAME),
    () -> drivetrain.getState().Pose.getRotation().getDegrees(),
    () -> Math.toDegrees(drivetrain.getState().Speeds.omegaRadiansPerSecond));
```

The yaw and yaw-rate suppliers are injected via lambdas — `VisionSubsystem` doesn't know about `drivetrain` directly, it just consumes `DoubleSupplier`s. This is the cleanest dependency-injection pattern in the codebase. Worth pointing at when teaching dependency management.

### `RobotModeTriggers.disabled()` keeps the swerve idle during disable

```java
RobotModeTriggers.disabled().whileTrue(
    drivetrain.applyRequest(() -> idle).ignoringDisable(true));
```

Without this, the modules would coast or behave unpredictably when the robot is disabled. The `.ignoringDisable(true)` is the magic — most commands won't run during disable, but the Idle request needs to. Real defensive code that prevents an entire category of "robot drifted in the queue" bugs.

### Driver `back` and Operator `back` both bind to vision-pose-reset

```java
driver.back().onTrue(drivetrain.resetPoseFromVisionCommand());
operator.back().onTrue(drivetrain.resetPoseFromVisionCommand());
```

The "rode up on a ball, wheels lost contact, odometry corrupted" recovery from the drivetrain retro is bound to *both* controllers. Either operator can fix the problem without having to coordinate. This is real human-factors thinking — when something goes wrong in match, the driver and operator are both stressed, and either one being able to issue the fix is a meaningful UX win.

### Sign-convention preservation comment

```java
// It is critical that these inputs are (-). Do not change them.
drivetrain.setDefaultCommand(
    drivetrain.applyRequest(() ->
        drive.withVelocityX(-driver.getLeftY() * MaxSpeed)
            .withVelocityY(-driver.getLeftX() * MaxSpeed)
            .withRotationalRate(-driver.getRightX() * MaxAngularRate)));
```

That comment is doing real work. The negation is a real engineering decision (joystick conventions vs field-relative conventions), and a future student "cleaning up unnecessary minus signs" would break field-centric driving immediately. The same preservation pattern as `// Sign confirmed on robot: negative = correct for rear-mounted camera` from AlignAndShootCommand. Both deserve to stay forever.

### The deadline-shoot-with-compression pattern

```java
operator.a().whileTrue(
    Commands.deadline(
        FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.TRENCH),
        FuelCommands.fuelCompressionWhenShooterReady(shooter, intake)
    ));
```

Every preset shoot binding wraps the shoot in a deadline with `fuelCompressionWhenShooterReady` running in parallel. While the shooter is spinning up and firing, the intake is actively compressing fuel through the chute to maintain a steady supply. The deadline ensures the compression stops when the shoot stops.

This is sophisticated coordination — the kind of pattern that distinguishes "we have a shooter" from "we have a *shot rhythm*." Worth elevating as a teaching example.

### Standby toggle is *fully* documented

```java
// Start (Menu ☰): Toggle flywheel standby pre-rev — operator sets once and forgets.
// When ON: flywheel holds at STANDBY_RPM (1800) between shots instead of stopping.
// When OFF: flywheel returns to full idle after each shot.
// Defaults OFF at robot startup — operator must enable explicitly.
operator.start().onTrue(Commands.runOnce(shooter::toggleStandbyMode, shooter));
```

Three pieces of information in four comment lines: what the button does, the two-state behavior, and the startup default. A new student reading this knows everything they need to know about that binding. Compare to most binding comments in any FRC codebase, which say nothing.

This level of comment should be the standard for every non-obvious binding. Most aren't (see Bad).

### Drive speed clamped centrally

```java
private double MaxSpeed = Constants.DRIVE_CLAMP * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
private double MaxAngularRate = Constants.DRIVE_CLAMP * RotationsPerSecond.of(1.0).in(RadiansPerSecond);
```

One constant (`DRIVE_CLAMP = 0.75`) scales both translation and rotation. To make the robot faster or slower for a particular event, change one number. The driver doesn't need to retune feel because the *ratio* between translation and rotation stays consistent.

## The Bad

### The auto chooser is mostly commented out

This is the headline finding. The chooser as it shipped:

```java
autoChooser.addRoutine("Left x1 Full Swipe", autoRoutines::Trench_FullSwipeSingleLt);
autoChooser.addRoutine("Left x2 Full Swipe", autoRoutines::Trench_FullSwipeDoubleLt);
autoChooser.addRoutine("PreTrench Left Swipe", autoRoutines::PreTrench_SwipeLt);
// autoChooser.addRoutine("Left x1 Ramp", autoRoutines::LtTrench_Ramp_Single);
// autoChooser.addRoutine("Left x1 Ramp ANGRY", autoRoutines::LtTrench_Ramp_Angry);
// autoChooser.addRoutine("Left x2 Ramp", autoRoutines::LtTrench_Ramp_Double);
// autoChooser.addRoutine("Left x2 Ramp Sweep SHOT", autoRoutines::LtTrench_Ramp_HubSweep);
// autoChooser.addRoutine("Left x2 Ramp PURGE", autoRoutines::LtTrench_Ramp_Sweep_Purge);
// autoChooser.addRoutine("Left PreTrench Center Shot", autoRoutines::LtPreTrench_Center);
```

Same shape on the right side. Roughly:

- **Active (selectable)**: 7 routines — 3 left swipes, 3 right swipes, 1 center, 1 bulldozer (right only).
- **Commented out**: ~13 routines.

This is a major finding for the auton retro. AutoRoutines.java declares 20+ routines; only 7 ever appeared in the drive coach's dropdown. Everything else is *dead code that compiles*. The Ramp variants, the HubSweep, the AngryPurge — none of them shipped, but they're still maintained as if they did.

This is the answer to the auton retro's open question "which routines were actually used in matches?" The lower-bound answer is "at most these 7" (with the actual selection data from FMS narrowing it further).

**Concrete cleanup:** delete every `AutoRoutine` method from `AutoRoutines.java` that isn't referenced (commented or otherwise) in this file. Move the commented-out chooser lines to a comment block at the top saying "Below is the list of routines that were tested but not deployed in 2026." That's a 5-minute change that cuts maybe 600 lines from `AutoRoutines.java`.

### Operator pad has multiple unused/commented bindings

```java
// operator.rightTrigger(0.5).whileTrue(Align indexer.reverse());
// operator.rightTrigger(0.5).whileTrue(indexer.reverse());

// operator.povLeft().onTrue(intake.extendSlidesFastCmd());
// operator.povRight().whileTrue(intake.fuelPumpCycleDelayed());
```

Right trigger, povLeft, povRight on the operator are all unused. That's three "missing" capabilities. Whether they should be wired depends on operator feedback — but the commented-out lines suggest the *intent* was there. Either restore them (with documentation matching the standby toggle's standard) or delete the comments.

The double rightTrigger commented line is the strongest tell — someone tried indexer.reverse, then tried "Align" something (probably half-typed), then commented both out. The actual decision history is invisible.

### Driver `a` button is half-finished

```java
// driver.a().whileTrue(
//     // drivetrain.applyRequest(() -> xBrake),
//     // FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.PASS)
// );
```

Was driver-a going to be xBrake? PASS shot? Both? The double-commenting (a comment inside a comment) shows the author was bouncing between options and stopped without picking one. The result: a useful button on the driver pad is unused, and a future student can't tell if it was deliberate or a TODO.

### Double `xBrake` declaration

```java
// private final SwerveRequest.SwerveDriveBrake xBrake = new SwerveRequest.SwerveDriveBrake();  // line 56

private final SwerveRequest.SwerveDriveBrake xBrake = new SwerveRequest.SwerveDriveBrake();    // line 62
```

One commented, one active. Tiny thing but it's a tell — someone moved the declaration and forgot to delete the original. Whichever is canonical is canonical; the other goes.

### Repetitive operator shot-preset bindings

```java
operator.a().whileTrue(Commands.deadline(FuelCommands.shootWithPreset(shooter, indexer, TRENCH), ...));
operator.b().whileTrue(Commands.deadline(FuelCommands.shootWithPreset(shooter, indexer, CLOSE), ...));
operator.x().whileTrue(Commands.deadline(FuelCommands.shootWithPreset(shooter, indexer, TOWER_FRONT), ...));
operator.y().whileTrue(Commands.deadline(FuelCommands.shootWithPreset(shooter, indexer, FAR), ...));
```

Four nearly-identical bindings. Same `Commands.deadline(shootWithPreset(..., PRESET), fuelCompressionWhenShooterReady(...))` shape, different preset. Each one also has a commented-out `drivetrain.applyRequest(() -> xBrake)` inside it — the xBrake-while-shooting plan that didn't ship.

A helper method would compress this to one binding-per-line:

```java
private void bindShotPreset(Trigger trigger, ShotPreset preset) {
    trigger.whileTrue(Commands.deadline(
        FuelCommands.shootWithPreset(shooter, indexer, preset),
        FuelCommands.fuelCompressionWhenShooterReady(shooter, intake)));
}

// In configureBindings():
bindShotPreset(operator.a(), ShotPreset.TRENCH);
bindShotPreset(operator.b(), ShotPreset.CLOSE);
bindShotPreset(operator.x(), ShotPreset.TOWER_FRONT);
bindShotPreset(operator.y(), ShotPreset.FAR);
```

Same behavior, half the LOC, every binding immediately readable. This is the same DRY argument as the AutoRoutines and the four `shootPresetAuton` named wrappers — when a pattern repeats, extract it.

### No documentation block listing all bindings

A judge or new student opening this file has to read 200 lines of code to understand what every button does. A one-page comment block at the top — "Driver: trigger = align+shoot, leftTrigger = intake, leftBumper = align-only, back = pose-reset" — would make this file self-documenting.

The shooter retro praised the standby-toggle documentation; the same pattern applied at the file level would be a big readability win.

## The Ugly

### The commented-out routines tell a season story

Read the chooser block top-to-bottom and you can reconstruct what happened:

1. Early season: simple ramp-crossing routines (`LtTrench_Ramp_Single`, `LtTrench_Ramp_Double`).
2. Mid-season experiments: variants with hub-sweep, purge, "angry" aggression.
3. Late season: the team settled on `FullSwipe` and `PreTrench` as the reliable strategies, commented out everything else.
4. Edge cases: `LtPreTrench_Center` was attempted but `LtBulldozer` never even got an `addRoutine` call (it's referenced as `autoRoutines::LtBulldozer` in a commented line).

That's a *real auton evolution log* embedded in commented code. Useful for archaeology, painful for current readability.

The cleanup recommendation: extract these into `docs/autonomy-2026-history.md` (or similar). The information is valuable; its location is wrong.

### The "test" reference points at code that may not exist

```java
// autoChooser.addRoutine("Rt Bulldozer 2026", autoRoutines.test::Bulldozer);
```

`autoRoutines.test` would access a field called `test`. The `Test` inner class in `AutoRoutines.java` is `public class Test` (capital T) without a `test` instance. So this commented line wouldn't compile if uncommented — it references a member that doesn't exist with that name.

A small example of how commented code "rusts" — it might have compiled at some point, but the surrounding code drifted and now it's literal dead syntax. This is the strongest argument for deleting commented code: you can't even tell if it would *work* anymore.

### `getGameDataTelemetry()` exists but doesn't appear to be called from here

The public getter is at the bottom:

```java
public GameDataTelemetry getGameDataTelemetry() {
    return gameDataTelemetry;
}
```

`gameDataTelemetry` is private. `updateGameData()` is called from `Robot.robotPeriodic()`. Who calls `getGameDataTelemetry()`? Possibly nothing in the codebase — it might be a leftover from a refactor that moved the telemetry consumer. Worth a grep.

### Telemetry initialization with no documentation

```java
private final Telemetry logger = new Telemetry(MaxSpeed);
// ...
drivetrain.registerTelemetry(logger::telemeterize);
```

What is `Telemetry`? What does it log? `MaxSpeed` gets passed to it — is that for scaling, for a calculated max-power calculation? A reader has to open `Telemetry.java` (not in our review pass) to find out. A one-line comment would help: `// SignalLogger-based telemetry for AdvantageScope replay`.

### `xBrake` is bound to operator povDown but the comments suggest it was supposed to be available more broadly

```java
operator.povDown().whileTrue(drivetrain.applyRequest(() -> xBrake));
```

The four shot-preset bindings all have `// drivetrain.applyRequest(() -> xBrake)` commented inside their deadline groups. The implication: at one point, the plan was for each shot to *also* x-brake the drivetrain. That plan apparently got abandoned, but the commented lines remain.

If x-brake-while-shooting is desirable (it usually is — locks the drivetrain so the robot doesn't drift during the shot), the right binding is `Commands.parallel(shoot, applyRequest(xBrake))`. If it's not desirable, delete the commented lines.

## Open questions for the student owner

1. **What's the FMS data for routine selection?** Pull match-by-match auton selections from FMS and confirm which of the 7 active routines actually got picked. That data answers most of the auton-cleanup priority questions.
2. **Were the unused operator bindings (rightTrigger, povLeft, povRight) deliberate omissions or unfinished plans?** Driver/operator feedback would tell the story.
3. **Did the team ever try x-brake-during-shot in practice?** The commented lines suggest yes. The Hoot logs from any match with shot-then-drift behavior would show whether the brake would have helped.
4. **Why was driver-a left empty?** PASS-shot binding makes sense (driver could request a pass instead of a vision shot) — was that the intent that got dropped?

## Lessons for next season

1. **The auto chooser is the source of truth for what shipped.** Treat it as such. Every routine in `AutoRoutines.java` that isn't in the chooser is dead code and should be deleted, archived to docs, or marked clearly.
2. **Binding documentation should match the standby-toggle standard.** Three pieces of information per non-obvious binding: what the button does, what state changes, what's the default. Comment-budget for `configureBindings()` should be ~30 lines of comments mixed with the bindings, not 0 lines.
3. **Repeated binding shapes deserve a helper.** The four shot-preset bindings should call `bindShotPreset(trigger, preset)`. Same DRY argument as everywhere else.
4. **A binding map at the top of the file is high-leverage documentation.** Judges, new students, and the drive coach all benefit. Costs ~20 lines, pays back constantly.
5. **Commented-out bindings expire fast.** The `autoRoutines.test::Bulldozer` reference no longer matches the actual class structure. Commented code becomes archaeologically interesting, not technically useful. Delete or restore in code review.
6. **Driver and operator should have a redundant pose-reset.** This codebase does it (both `back` buttons). Make it a team rule for any recovery action — both controllers get the binding.
