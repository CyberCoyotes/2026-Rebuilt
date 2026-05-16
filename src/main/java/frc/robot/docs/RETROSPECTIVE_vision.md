# Vision — 2026 Season Retrospective

## What this subsystem does

Vision provides AprilTag-based sensor data for two consumers: pose fusion (feeding the drivetrain's pose estimator) and relative alignment (helping commands drive the robot to line up with a target). The hardware is a single Limelight 4 running MegaTag2 pose estimation as the primary mode and MegaTag1 as a fallback. Raw target data (TX, TY, target area, tag ID) is also exposed for commands that prefer relative-sensor alignment over absolute pose.

## Architecture at a glance

The vision package splits into four files:

- `VisionIO.java` — interface defining what a vision sensor can do
- `VisionIOLimelight.java` — Limelight-specific implementation via `LimelightHelpers`
- `VisionSubsystem.java` — `SubsystemBase` that reads inputs each cycle and exposes a clean public API
- `LimelightHelpers.java` — vendor-distributed helper from Limelight Vision (not team-authored; not under retrospective)

The naming differs from the other subsystems — `VisionIOLimelight` rather than `VisionIOHardware`. Reasonable choice given that vision is more naturally specialized by camera type than by "hardware vs sim," but worth flagging for consistency.

## A note on scope before going further

Most subsystem retrospectives can be done in isolation — the indexer is the indexer, the shooter is the shooter. Vision is different. The *subsystem* in this folder is well-designed, but it's not the whole story. The actual intelligence — pose fusion math, alignment control law, distance-to-shot computation — lives outside this folder:

- **Pose fusion** is in `Robot.robotPeriodic()`. The vision subsystem deliberately doesn't call `addVisionMeasurement` — the comment is explicit: *"Pose fusion (addVisionMeasurement) is NOT done here — Robot.robotPeriodic() owns that with the distance-scaled std dev formula and omega gate. Keeping fusion in one place avoids double-counting measurements in the Kalman filter."*
- **Alignment** is in `AlignAndShootCommand` (and possibly related commands). The vision subsystem exposes `hasFreshTarget()` and `getTX()` so commands can drive `TX → 0`, but the actual control law isn't here.
- **Distance computation** for the shooter lookup table is somewhere outside this folder. The vision subsystem doesn't expose a `getDistanceMeters()` method.

This means this retrospective is **structurally incomplete on its own.** The follow-up — and likely a separate retrospective — needs to cover the alignment commands and the pose-fusion code in `Robot.java`. I'll flag in Open Questions which files are next.

## Decisions we made and why

### Single point of pose fusion

The most important architectural decision in the codebase, and it's documented in a single comment:

> *"Pose fusion (addVisionMeasurement) is NOT done here — Robot.robotPeriodic() owns that... Keeping fusion in one place avoids double-counting measurements in the Kalman filter."*

If the vision subsystem also called `addVisionMeasurement`, and `Robot.robotPeriodic()` also did, the Kalman filter would integrate the same measurement twice and produce overconfident estimates. The discipline of "one component owns fusion" is exactly right.

This is worth lifting as a general pattern: for any sensor that feeds the pose estimator, the *fusion call site* should be unique. Other subsystems should expose raw inputs; one place should fuse.

### Alliance-aware hub tag validation

`hasFreshTarget()` validates not just *"is the camera seeing something"* but *"is the camera seeing a tag from our hub specifically":*

```java
private boolean isHubTag(int id) {
    if (id < 0) return false;
    int[] hubTags;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        hubTags = Constants.Vision.RED_HUB_TAG_IDS;
    } else {
        hubTags = Constants.Vision.BLUE_HUB_TAG_IDS;
    }
    for (int hubTag : hubTags) {
        if (id == hubTag) return true;
    }
    return false;
}
```

This prevents a class of bug where the camera sees a coopertition or scoring tag, returns valid pose data, and alignment commands lock onto the wrong target. Defaults to blue when FMS hasn't reported — reasonable choice for practice runs.

### Multi-tag flipping handled by documented strategy

The comment on `getTagCount()` captures a real gotcha:

> *"tagCount == 1 → TX is the best rotation reference (single tag, no flip risk)*
> *tagCount >= 2 → odometry bearing from MT2 pose is more stable than TX (two tags on the same hub face cause primary-tag flipping)"*

"Primary-tag flipping" is the Limelight reporting different tags as `tid` between frames when multiple are visible. TX then discontinuously jumps as the primary changes. This isn't a hypothetical — it's a real failure mode that the team encountered, diagnosed, and developed a strategy for.

The strategy itself is documented but the *enforcement* lives in alignment commands we haven't read. Worth confirming in the next retrospective that the rule is actually applied.

### Hedge between absolute pose and relative sensor

The IO interface exposes both:

- MegaTag2 / MegaTag1 pose (absolute position on the field)
- Raw TX / TY / target area (relative angular offsets to the camera)

This is a deliberate hedge between two alignment philosophies:

1. **Absolute pose alignment** — use MegaTag to know "I am at X, Y, θ" and drive to "I should be at X', Y', θ'."
2. **Relative sensor alignment** — use TX to know "the target is N degrees to my right" and drive `TX → 0` without caring about absolute position.

Team 2056 famously commits to (2) for reliability — relative sensing tends to be more accurate at short range than absolute pose, because pose estimation accumulates errors as distance increases. The team's intent going into the season (per the programming notes) was to follow 2056's lead on this.

What shipped supports both. The IO exposes both data streams; the multi-tag comment says to use both depending on tag count. This is sophisticated, but it also means the team hasn't fully committed to one philosophy — and that's worth flagging as the big strategic question for next season.

### Constructor dependency injection

`VisionSubsystem(VisionIO io, DoubleSupplier yawDegrees, DoubleSupplier yawRateDegPerSec)` — the yaw and yaw-rate sources are injected rather than referenced statically. This is the cleanest constructor in the codebase from a testability standpoint. The vision logic doesn't know where the yaw comes from; it just consumes a `DoubleSupplier`. A test or sim could supply any source.

Worth pointing out as a teaching example for how to write subsystems that depend on data from other subsystems without creating hard coupling.

### AdvantageKit retained, only here

Vision is the only subsystem that still uses AdvantageKit logging. `VisionIOInputs implements LoggableInputs` with `toLog()` and `fromLog()`. `Logger.processInputs("Vision", inputs)` is called every cycle. `Logger.recordOutput(...)` publishes derived values.

This is worth highlighting because it confirms the "selective adoption" principle. Vision benefits from log replay much more than mechanism subsystems do — vision bugs are often "the camera saw something weird at this exact moment" and replaying that frame is the only way to diagnose. For an intake or indexer, you can usually reproduce a bug on the bench. For vision, you need the actual match conditions.

The team made the right call to keep AKit here when pulling it elsewhere.

## What worked well

### `hasFreshTarget()` as a public-API guardrail

Commands that want to use `getTX()` are expected to check `hasFreshTarget()` first. The comment on `getTX()` is explicit: *"Check hasFreshTarget() before using for control."* This is a small API discipline that prevents an entire class of bug — using a stale or zero TX value as if it were valid alignment data.

(Caveat below: `hasFreshTarget()` doesn't actually check freshness in the temporal sense. See findings.)

### Pose source telemetry

The dashboard publishes `"MegaTag2"`, `"MegaTag1"`, or `"None"` as the current pose source. This lets the driver or operator glance at the dashboard and immediately know vision confidence. Compare to "vision feels bad and we don't know why" — having the active estimator displayed eliminates one variable from in-match debugging.

### `SetRobotOrientation` ordering captured in comments

MegaTag2 requires `SetRobotOrientation(yaw, yawRate, ...)` to be called before reading the pose estimate, every cycle. The code does this correctly, and the comment chain explains why:

- In `VisionIOLimelight.updateInputs`: *"Must be sent before reading MegaTag2 — seeds the rotation-locked estimate."*
- In `VisionSubsystem.periodic`: *"SetRobotOrientation is called inside updateInputs (VisionIOLimelight) so AKit-logged MT2 reads use the same fresh orientation that Robot.java fuses."*

The reasoning preserved at both layers means a future student modifying either file will see the dependency. Good practice.

## What we'd reconsider

### `hasFreshTarget()` doesn't actually check freshness

The method name implies a temporal check — *is this data fresh?* But the implementation is:

```java
public boolean hasFreshTarget() {
    return inputs.hasTargets && isHubTag(inputs.tagId);
}
```

That's a *validity* check, not a *freshness* check. A frozen frame from a disconnected Limelight, or a stale `inputs` struct from a hung IO read, would still report `hasFreshTarget = true` as long as the last seen tag was a hub tag.

A real freshness check would compare `inputs.megaTag2Timestamp` (or `inputs.totalLatencyMs`) against the current robot time and reject data older than a threshold (say, 100 ms). Without this, a Limelight that drops off the network but still reports its last frame will silently keep alignment commands "working" with stale data.

Two fixes:
1. Rename to `hasValidHubTarget()` to match what it actually does.
2. Add a real freshness check based on `inputs.totalLatencyMs` and the read timestamp.

Either is fine. The current half-state — accurate-looking name, validity-only check — is the worst option because a student reading the code will trust the name.

### Single-Limelight architecture

The constructor takes one `VisionIO` and one Limelight is instantiated. Reference team 2056 ran two Limelight 3Gs for hub tracking plus a Limelight 4 with the Hailo accelerator for game-piece tracking — three cameras total.

Single-camera vision means:
- Any dropout is a complete loss of vision data
- No cross-validation between cameras
- Field of view is limited; the robot can lose all tags in some orientations
- No specialization (e.g., one camera optimized for close-range, one for far)

This is a real architectural growth opportunity for 2027, and the IO interface is set up to support it — `VisionIOLimelight(String limelightName)` takes a name parameter, suggesting multi-instance was anticipated. The subsystem just doesn't compose multiple instances yet.

### Both MegaTag estimators read every cycle

In `VisionIOLimelight.updateInputs`, both MT2 and MT1 are read every cycle, even when MT2 is valid and MT1 is unused:

```java
LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
// ... process mt2 ...
LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
// ... process mt1 ...
```

Each call is a NetworkTables read with parsing overhead. For a fallback that's rarely used, this is wasted work every loop. A guard could skip the MT1 read when MT2 is valid:

```java
if (!inputs.megaTag2Valid) {
    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    // ... process mt1 ...
}
```

Small fix, real win on loop time. The shooter's CAN-bus-optimization mindset applied to vision.

### `synchronized` on `updateInputs` is probably leftover

`VisionIOLimelight.updateInputs` is marked `synchronized` — none of the other IO files do this. The only caller visible in `VisionSubsystem.periodic()` is single-threaded (the WPILib scheduler). Either:
1. There's a thread reading vision data elsewhere that requires synchronization, or
2. The `synchronized` is defensive leftover from when alignment commands called the IO directly.

Worth confirming. If (2), it can be removed.

### Duplicate logging via `processInputs` and `recordOutput`

`txDegrees` is logged twice in AKit — once via `Logger.processInputs("Vision", inputs)` (which logs the whole inputs struct including TX) and again via `Logger.recordOutput("Vision/TX_deg", inputs.txDegrees)`. Same for `hasFreshTarget()` and `poseSource`.

Not a correctness issue, but AdvantageKit log files include both entries. Pick one — `processInputs` is the right choice for raw data, `recordOutput` for derived values. The current code is mixing them.

### No vision-vs-odometry disagreement detection

A common failure mode in pose estimation: wheels say the robot is at X=2.0m, vision says X=2.8m. Either wheel slip or vision error. Detecting and logging this disagreement is a powerful diagnostic — if you see the gap widen over a match, something is wrong.

The vision subsystem could compute and log `|visionPose - currentEstimatedPose|` every cycle (the pose estimator is accessible via the suppliers it already has). This isn't a 2026-season problem to fix; it's a 2027 instrumentation addition. Worth adding to the "lessons" list.

### Distance computation isn't here

The shooter retrospective flagged the sparse vision distance lookup table. The distance value flowing into that table comes from *somewhere*, but it's not exposed by `VisionSubsystem`. Most likely candidates:

- Computed in `AlignAndShootCommand` from `inputs.tyDegrees` and known mounting geometry
- Computed in `Robot.robotPeriodic()` from the fused pose's position relative to the hub tag's known field location
- Calling `LimelightHelpers.getCameraPose_TargetSpace(...)` directly in a command

This split makes it harder to:
- Reason about *which* distance the shooter is using
- Validate the measurement (no single place to add `Logger.recordOutput("Vision/Distance_m", ...)`)
- Reuse the distance for other consumers (LED indicators, alignment confidence)

A cleaner pattern: expose `getDistanceToHubMeters()` as a public method on `VisionSubsystem`, computed once per cycle, logged once. Both the shooter command and any other consumer use the same number. Worth doing as part of the offseason cleanup.

### IMU readiness gate is missing

`SetRobotOrientation` is called every cycle starting from the first `periodic()`. If the gyro isn't fully initialized at that point (e.g., still doing factory mounting calibration), the first few MT2 reads will be seeded with garbage yaw. Probably not catastrophic — the pose estimator filters out the bad early frames — but a gate like `if (gyroIsReady()) { ... }` would be cleaner.

This may or may not be a real problem in practice. Hoot logs from the first second of any match would show whether MT2 was reporting weird poses during gyro warmup. Worth a check.

## Open questions for the student owner

This subsystem is one of three or four files that together make up "the vision system." The retrospective is necessarily incomplete without the others. The student owner should walk the rest of the system and produce a follow-up retrospective covering:

1. **AlignAndShootCommand** (and any related commands). The actual alignment control law lives here. Questions: Is `TX → 0` PID-controlled? What's the feedforward? Does it use the documented "tagCount-based" switching between TX and odometry bearing? How does it handle no-target frames mid-alignment?
2. **Robot.robotPeriodic()** pose fusion. The comment promises "distance-scaled std dev formula and omega gate." Walk the code, confirm both are implemented, and document the actual formula. The std-dev weighting is the heart of how trustworthy vision is at different distances — if it's miscalibrated, everything downstream is wrong.
3. **Wherever distance to hub is computed.** The shooter retrospective flagged the sparse vision distance lookup table. Step one of fixing it is knowing where the input distance comes from.
4. **Did the multi-tag bearing strategy actually get used?** The comment on `getTagCount()` describes a TX-vs-odometry switching strategy. Is that implemented in `AlignAndShootCommand` or just documented as intent? Hoot logs from a multi-tag-visible moment will show whether TX was followed or odometry bearing was followed.
5. **Was hub-only tag filtering ever a problem?** `isHubTag()` rejects non-hub tags. If the robot is positioned to see a coopertition tag but not a hub tag, alignment commands will report `hasFreshTarget = false` and refuse to align. In practice was this ever an issue, or is the hub always visible during shot windows?
6. **Did Choreo + vision precision actually work end-to-end?** This is the question that drove this whole retrospective. The team's design intent was layered: Choreo for gross movement, vision for final precision. The IO data is here. The fusion is in Robot.java. The alignment is in AlignAndShootCommand. The shot is in ShooterSubsystem. The full pipeline crosses four files. *Did the pipeline as a whole produce reliable shots at all measured distances?* If not, where did it break?

## Lessons for next season

1. **Single point of fusion is the model.** Any sensor that feeds the pose estimator should have one — and only one — site that calls `addVisionMeasurement`. Extend this principle to anything else that feeds a Kalman filter or running estimate.
2. **Alliance-aware constants should be the default.** Whenever a value depends on red vs blue (tag IDs, field offsets, scoring positions), the lookup should be alliance-aware with a sensible default. The `isHubTag` pattern is the template.
3. **AdvantageKit was worth retaining for vision specifically.** Selective adoption worked. The principle for 2027: subsystems whose failures are *transient and condition-dependent* benefit from log replay; subsystems whose failures are *reproducible on the bench* don't. Vision was in the first category. Mechanism subsystems mostly weren't.
4. **Pick a vision philosophy by kickoff.** The 2026 code supports both absolute-pose alignment (MegaTag) and relative-sensor alignment (TX/TY) and switches between them based on tag count. That works, but it's sophisticated logic spread across multiple files. For 2027, decide whether vision is primarily an absolute localizer or a relative sensor — and if it's both, document the switching rule in one place that everyone (programmers, drive coach, judges) can find.
5. **`hasFreshTarget()` should literally check freshness.** Add a timestamp-based staleness check. The general principle: when a method has a temporal-sounding name, it should enforce a temporal condition, not just a validity condition.
6. **Centralize distance-to-hub.** If multiple consumers need the distance from the robot to the hub tag, compute it once in `VisionSubsystem`, log it once, expose it via `getDistanceToHubMeters()`. Don't let the calculation drift across multiple files where each one might compute it slightly differently.
7. **Plan for multi-Limelight in 2027.** The IO interface is set up for it. The subsystem isn't. Going from one Limelight to two or three is a real reliability and field-of-view upgrade — and other top teams have shown it's worth the complexity.
8. **The vision retrospective is system-level, not subsystem-level.** Vision spans `VisionSubsystem`, `AlignAndShootCommand`, `Robot.robotPeriodic()`, the shooter's distance lookup, and probably the LED subsystem (target-acquired feedback). When doing system reviews for 2027, vision should be reviewed as a *system retrospective*, not folded into a single subsystem retro. The other subsystems can stand alone; this one can't.
