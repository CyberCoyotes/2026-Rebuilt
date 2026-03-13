package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShotPreset;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.MetersPerSecond;

/**
 * Fuel Commands - Factory for shooter-related commands.
 *
 * This class provides static factory methods to create common shooter commands.
 * Using a factory pattern keeps command creation centralized and reusable.
 * 
 */
public class FuelCommands {

    /** Returns the hub center for the current alliance (defaults to blue if FMS not connected). */
    private static Translation2d getHubLocation() {
        return DriverStation.getAlliance()
                .filter(a -> a == DriverStation.Alliance.Red)
                .map(a -> Constants.Vision.RED_HUB_LOCATION)
                .orElse(Constants.Vision.BLUE_HUB_LOCATION);
    }

    // =========================================================================
    // PRIMARY SHOOT COMMANDS
    // =========================================================================

    /**
     * Shoots at whatever preset is currently selected.
     *
     * @param shooter The shooter subsystem
     * @param indexer The indexer subsystem
     * @return Command that shoots at current targets and returns to idle on release
     */

    public static Command shootAtCurrentTarget(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return Commands.sequence(
                Commands.runOnce(shooter::beginSpinUp, shooter),
                Commands.waitUntil(shooter::isReady).withTimeout(3.0),
                indexer.feed())
                .finallyDo(() -> shooter.setIdle())
                .withName("ShootAtCurrentTarget");
    }

    /**
     * Full preset shoot sequence that owns both the shooter and indexer subsystems.
     *
     * Flow:
     * 1. Sets the given RPM + hood target silently
     * 2. Calls beginSpinUp() — flywheel ramps up, hood moves to target
     * 3. Waits until both are at target (isReady()), with a 3-second safety timeout
     * 4. Runs indexer and conveyor forward to feed the game piece
     * 5. On trigger release (whileTrue interrupt): stops indexer/conveyor, returns
     * shooter to standby
     *
     * Use with whileTrue() on the shoot trigger.
     *
     * @param shooter The shooter subsystem
     * @param indexer The indexer subsystem
     * @param rpm     Target flywheel velocity in RPM
     * @param hood    Target hood position in rotations
     * @return Complete preset shoot command requiring both subsystems
     */
    public static Command shootWithPreset(ShooterSubsystem shooter, IndexerSubsystem indexer,
            double rpm, double hood) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    shooter.setTargetVelocity(rpm); // Set Constants._RPM
                    shooter.setTargetHoodPose(hood); // Set Constants._HOOD
                    shooter.beginSpinUp();         // void — transitions state machine to SPINNING_UP
                }, shooter),
                Commands.waitUntil(shooter::isReady).withTimeout(3.0),
                Commands.run(() -> {
                    indexer.conveyorForward();
                    indexer.indexerForward();
                }, indexer))
                .finallyDo(() -> {
                    indexer.indexerStop();
                    indexer.conveyorStop();
                    shooter.setIdle();
                }).withName("ShootWithPreset[" + rpm + "rpm]");
    }

    /**
     * Shoots at the given RPM/hood preset while simultaneously retracting the
     * intake slides (two-phase) and running the intake roller.
     *
     * The shooter+indexer sequence is the deadline — slide retraction runs in
     * parallel and is cancelled when the shot completes or the trigger is released.
     * The intake roller is always stopped on exit via the intake command's finallyDo.
     *
     * Use wherever {@link #shootWithPreset} is used when slides need to retract
     * during the shot (e.g. after intaking from the trench).
     *
     * @param shooter The shooter subsystem
     * @param indexer The indexer subsystem
     * @param intake  The intake subsystem
     * @param rpm     Target flywheel velocity in RPM
     * @param hood    Target hood position in rotations
     * @return Deadline command: shoot sequence (deadline) + slide retract in parallel
     */
    public static Command shootWithSlideRetract(ShooterSubsystem shooter, IndexerSubsystem indexer,
            IntakeSubsystem intake, double rpm, double hood) {
        return Commands.deadline(
                shootWithPreset(shooter, indexer, rpm, hood),
                intake.retractSlidesAuton())
                .withName("ShootWithSlideRetract[" + rpm + "rpm]");
    }

    public static Command shootPresetAuton(ShooterSubsystem shooter, IndexerSubsystem indexer,
            ShotPreset preset, double feedSeconds) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    shooter.setTargetVelocity(preset.rpm);
                    shooter.setTargetHoodPose(preset.hood);
                    shooter.beginSpinUp(); // void — transitions state machine to SPINNING_UP
                }, shooter),
                Commands.waitUntil(shooter::isReady),
                indexer.feed().withTimeout(feedSeconds))
                .finallyDo(() -> {
                    indexer.indexerStop();
                    indexer.conveyorStop();
                    shooter.setIdle();
                }).withName("ShootPresetAuton[" + preset.label + "]");
    }

    /**
     * Convenience overload — shoots using a {@link ShotPreset} enum value.
     *
     * @param shooter The shooter subsystem
     * @param indexer The indexer subsystem
     * @param preset  The named preset to use
     * @return Complete preset shoot command
     */
    public static Command shootWithPreset(ShooterSubsystem shooter, IndexerSubsystem indexer,
            ShotPreset preset) {
        return shootWithPreset(shooter, indexer, preset.rpm, preset.hood)
                .withName("ShootWithPreset[" + preset.label + "]");
    }


    /**
     * Air popper assist with intake.
     *
     * While held: runs intake + indexer feed while shooter stays in POPPER preset.
     */
    public static Command runAirPopper(IndexerSubsystem indexer, ShooterSubsystem shooter,
            IntakeSubsystem intake) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    shooter.setAirPopper(); // enters ShooterState.POPPER — vision cannot override
                }, shooter),
                Commands.deadline(
                        Commands.deadline( // parallel is NOT OK here!
                                intake.intakeFuel(),
                                indexer.feed()),
                        Commands.run(() -> {
                        }, shooter)))
                .finallyDo(() -> {
                    indexer.indexerStop();
                    indexer.conveyorStop();
                    shooter.setIdle();
                }).withName("RunAirPopperWithIntake");
    }

    // =========================================================================
    // UTILITY COMMANDS
    // =========================================================================

    /**
     * Agitates fuel by bouncing the intake roller against the bumpers, then shoots
     * once the flywheel is ready — feeding and bouncing run in parallel.
     *
     * Slides run in reverse (higher position = roller lower/out). The bounce rocks
     * the roller between SLIDE_BOUNCE_DOWN_POS (~40) and SLIDE_BOUNCE_UP_POS (~35).
     *
     * Sequence:
     *   1. Set trench preset targets and beginSpinUp (TESTING — swap preset as needed).
     *   2. Wait until shooter.isReady() — flywheel + hood settled (3 s timeout).
     *   3. Parallel:
     *        A. Run conveyor + indexer forward continuously (feeds the shot).
     *        B. Run slideBounceUp() repeatedly — keeps agitating while held.
     *   4. finallyDo: stops indexer/conveyor and returns shooter to idle.
     *
     * Phase 3 runs until the command is interrupted (whileTrue trigger released).
     * slideBounceUp().repeatedly() restarts the 2-second bounce cycle each time it
     * finishes, so the roller keeps rocking for as long as the button is held.
     *
     * Bind to a button with whileTrue().
     *
     * @param shooter The shooter subsystem
     * @param indexer The indexer subsystem
     * @param intake  The intake subsystem
     * @return Command that waits for flywheel ready, then feeds + bounces in parallel
     */
    // public static Command slideBounceAndShoot(ShooterSubsystem shooter, IndexerSubsystem indexer,
    //         IntakeSubsystem intake) {
    //     return Commands.sequence(
    //             // Step 1: arm trench preset and spin up (TODO: swap preset before competition)
    //             Commands.runOnce(() -> {
    //                 shooter.setTargetVelocity(Constants.Shooter.TRENCH_RPM);
    //                 shooter.setTargetHoodPose(Constants.Shooter.TRENCH_HOOD);
    //                 shooter.beginSpinUp();
    //             }, shooter),
    //             // Step 2: wait for flywheel + hood to settle before feeding
    //             Commands.waitUntil(shooter::isReady).withTimeout(3.0),
    //             // Step 3: feed + bounce in parallel until trigger is released
    //             Commands.parallel(
    //                     Commands.run(() -> {
    //                         indexer.conveyorForward();
    //                         indexer.indexerForward();
    //                     }, indexer),
    //                     intake.fuelPumpBasic().repeatedly()))
    //             .finallyDo(() -> {
    //                 indexer.indexerStop();
    //                 indexer.conveyorStop();
    //                 shooter.setIdle();
    //             }).withName("SlideBounceAndShoot");
    // }

    /**
     * Creates a command to eject jammed game pieces.
     * Reverses flywheel for a duration then returns to SPINUP.
     *
     * @param shooter         The shooter subsystem
     * @param durationSeconds How long to eject
     * @return Command that ejects then returns to idle
     */
    public static Command eject(ShooterSubsystem shooter, double durationSeconds) {
        return Commands.sequence(
                Commands.runOnce(shooter::eject, shooter),
                Commands.waitSeconds(durationSeconds)
        // Commands.runOnce(shooter::returnToStandby, shooter) // TODO: Do not use right
        // now **EXPERIMENTAL**
        ).withName("EjectShooter");
    }

    /**
     * Creates a command that waits until shooter is ready.
     *
     * @param shooter The shooter subsystem
     * @return Command that waits for shooter ready state
     */
    public static Command waitUntilReady(ShooterSubsystem shooter) {
        return Commands.waitUntil(shooter::isReady)
                .withTimeout(3.0)
                .withName("WaitForShooterReady");
    }

    public static Command shootPass(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    shooter.pass(); // enters ShooterState.PASS — vision cannot override
                }, shooter),
                Commands.waitUntil(shooter::isPassReady).withTimeout(3.0),
                Commands.run(() -> {
    indexer.conveyorForward();
    indexer.indexerForward();
}, indexer))
                .finallyDo(() -> {
                    indexer.indexerStop();
                    indexer.conveyorStop();
                    shooter.setIdle();
                }).withName("ShootPass");
    }

    // =========================================================================
    // POSE ALIGN AND SHOOT (primary match command — replaces VisionShootCommand)
    // =========================================================================

    /**
     * Pose-based hub alignment and shoot command.
     *
     * Replaces VisionShootCommand as a reusable factory method.
     *
     * Behavior (while trigger held):
     * 1. Computes robot-to-hub distance and bearing from drivetrain odometry.
     * 2. Calls updateFromDistance() every loop — keeps RPM and hood live-tracking.
     * 3. Drives rotation toward hub via a P-controller; driver controls translation.
     * 4. Feeds indexer once aligned within ALIGNMENT_TOLERANCE_DEGREES AND shooter isReady().
     * 5. On release: stops indexer, conveyor, returns shooter to idle.
     *
     * Feed tolerance fix: uses ALIGNMENT_TOLERANCE_DEGREES (2.0°) not ALIGNMENT_TOLERANCE_DEG
     * (0.5°). A P-only controller always has steady-state error; 0.5° was unreachable.
     *
     * @param shooter    Shooter subsystem
     * @param indexer    Indexer subsystem
     * @param drivetrain Swerve drivetrain (provides field pose via odometry/AprilTag fusion)
     * @param xSupplier  Driver left-Y velocity in m/s (scaled by MaxSpeed)
     * @param ySupplier  Driver left-X velocity in m/s (scaled by MaxSpeed)
     */
    public static Command poseAlignAndShoot(
            ShooterSubsystem shooter,
            IndexerSubsystem indexer,
            // IntakeSubsystem intake,
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {

        final SwerveRequest.FieldCentric alignRequest = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // NT diagnostics — created once per factory call (whileTrue caches the command object)
        final NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("VisionShoot");
        final DoublePublisher ntAngleToHub     = visionTable.getDoubleTopic("angleToHub_deg").publish();
        final DoublePublisher ntCurrentHeading = visionTable.getDoubleTopic("currentHeading_deg").publish();
        final DoublePublisher ntHeadingError   = visionTable.getDoubleTopic("headingError_deg").publish();
        final DoublePublisher ntRotRate        = visionTable.getDoubleTopic("rotRate_radps").publish();
        final DoublePublisher ntDistance       = visionTable.getDoubleTopic("distanceToHub_m").publish();
        final DoublePublisher ntLeadOffset     = visionTable.getDoubleTopic("leadOffset_deg").publish();

        // final Timer fuelPumpTimer = new Timer();

        return Commands.run(() -> {
            Translation2d hub = getHubLocation();
            Pose2d pose = drivetrain.getState().Pose;

            // 1. Distance → update shooter targets live
            double dx = hub.getX() - pose.getX();
            double dy = hub.getY() - pose.getY();
            double distance = MathUtil.clamp(
                    Math.hypot(dx, dy),
                    Constants.Vision.MIN_DISTANCE_M,
                    Constants.Vision.MAX_DISTANCE_M);

            shooter.updateFromDistance(distance);
            if (shooter.getState() != ShooterSubsystem.ShooterState.READY) {
                shooter.beginSpinUp(); // void — only transitions state; never call spinUp() (returns Command) here
            }

            // 2. Apply velocity offset for movement
            double angleToHubDeg = Math.toDegrees(Math.atan2(dy, dx));

            // Velocity lead compensation — offsets aim opposite to lateral movement
            var speeds = drivetrain.getState().Speeds;
            double vx = speeds.vxMetersPerSecond;
            double vy = speeds.vyMetersPerSecond;
            double hubAngleRad = Math.atan2(dy, dx);
            double lateralVelocity = -vx * Math.sin(hubAngleRad) + vy * Math.cos(hubAngleRad);
            double leadOffsetDeg = -lateralVelocity * Constants.Vision.LEAD_COMPENSATION_DEG_PER_MPS;

            double currentHeadingDeg = pose.getRotation().getDegrees();
            double headingErrorDeg   = angleToHubDeg + leadOffsetDeg - currentHeadingDeg;
            while (headingErrorDeg >  180) headingErrorDeg -= 360;
            while (headingErrorDeg < -180) headingErrorDeg += 360;

            ntLeadOffset.set(leadOffsetDeg);

            // 3. Rotation correction
            double rotRate = MathUtil.clamp(
                    headingErrorDeg * Constants.Vision.ROTATIONAL_KP,
                    -Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC,
                    Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC);

            ntAngleToHub.set(angleToHubDeg);
            ntCurrentHeading.set(currentHeadingDeg);
            ntHeadingError.set(headingErrorDeg);
            ntRotRate.set(rotRate);
            ntDistance.set(distance);

            drivetrain.setControl(
                    alignRequest
                            .withVelocityX(xSupplier.getAsDouble() *.40) //Setting max drive speed to 40% when Auto Shooting
                            .withVelocityY(ySupplier.getAsDouble() *.40)
                            .withRotationalRate(rotRate));

            // 4. Feed: aligned within 2° AND flywheel/hood settled
            if (shooter.isReady()) {
                indexer.conveyorForward();
                indexer.indexerForward();
            } else {
                indexer.indexerStop();
                indexer.conveyorStop();
                // fuelPumpTimer.reset(); // reset so pump starts fresh when shooter becomes ready
            }

        }, shooter, indexer, drivetrain /*, intake*/)
                .beforeStarting(Commands.runOnce(() -> {
                    shooter.beginSpinUp();
                    // fuelPumpTimer.reset();
                    // fuelPumpTimer.start();
                }, shooter))
                .finallyDo(() -> {
                    indexer.indexerStop();
                    indexer.conveyorStop();
                    shooter.setIdle();
                })
                .withName("PoseAlignAndShoot");
    }

    // =========================================================================
    // VISION ALIGN AND SHOOT (limelight tx — secondary/fallback)
    // [UNBOUND] visionAlignAndShoot, visionShootSequence, visionShot, trackTarget
    // None of these are wired to a button in RobotContainer. poseAlignAndShoot
    // (odometry-based) is the active shooting command on driver.rightTrigger().
    // =========================================================================

    /**
     * Combined vision-targeting and shooting command. Intended as the primary RT
     * binding.
     *
     * Behavior (all while trigger held):
     * 1. Arms the current POV-selected preset immediately as a fallback baseline.
     * 2. Continuously looks for the nearest hub AprilTag (alliance-aware).
     * 3. If a hub tag is visible (or in grace period): updates flywheel RPM and
     * hood position from the distance interpolation table every cycle.
     * 4. Overrides drivetrain rotation with a P-controller on tx (horizontal
     * angle),
     * while still allowing the driver to translate freely with the left stick.
     * 5. Fires the indexer and conveyor as soon as vision reports aligned AND
     * shooter reports ready. Stops feeding if either condition is lost.
     * 6. On trigger release: stops indexer, conveyor, and shooter.
     *
     * Subsystem requirements: shooter, vision, indexer, drivetrain
     * → interrupts the default drive command for the duration of the trigger hold.
     *
     * Tuning handles:
     * - Constants.Vision.ROTATIONAL_KP (rotation aggressiveness)
     * - Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC (rotation clamp)
     * - ShooterSubsystem FLYWHEEL_RPM_MAP / HOOD_ROT_MAP (shooter params)
     * See TUNING.md for the step-by-step procedure.
     *
     * @param shooter    Shooter subsystem
     * @param vision     Vision subsystem
     * @param indexer    Indexer subsystem
     * @param drivetrain Swerve drivetrain
     * @param xSupplier  Driver left-Y velocity in m/s (already scaled by MaxSpeed)
     * @param ySupplier  Driver left-X velocity in m/s (already scaled by MaxSpeed)
     */
    // public static Command visionAlignAndShoot(
    //         ShooterSubsystem shooter,
    //         VisionSubsystem vision,
    //         IndexerSubsystem indexer,
    //         CommandSwerveDrivetrain drivetrain,
    //         DoubleSupplier xSupplier,
    //         DoubleSupplier ySupplier) {

    //     final double maxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    //     // Created once; reused every execute cycle.
    //     // FieldCentric: driver controls X/Y translation, vision controls rotation.
    //     final SwerveRequest.FieldCentric alignRequest = new SwerveRequest.FieldCentric()
    //             .withDeadband(maxSpeed * 0.15)
    //             .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    //     return Commands.run(() -> {

    //         // ── 1. Shooter: update from vision or hold current targets ─────────
    //         if (vision.isUsableForShooting()) {
    //             // Live-update RPM + hood from distance table (also pushes to hardware
    //             // if already in READY state — see ShooterSubsystem.updateFromDistance)
    //             shooter.updateFromDistance(vision.getDistanceToTargetMeters());
    //         }
    //         // Keep commanding SPINNING_UP every cycle until READY — setState() no-ops if already there
    //         if (shooter.getState() != ShooterSubsystem.ShooterState.READY) {
    //             shooter.beginSpinUp(); // void — only transitions state; never call spinUp() (returns Command) here
    //         }

    //         // ── 2. Drivetrain: driver translation + vision rotation ────────────
    //         // tx returns 0.0 when NO_TARGET, so rotation correction drops to zero
    //         // automatically when the camera has nothing to track.
    //         double txDeg = vision.getHorizontalAngleDegrees();
    //         double rotRate = MathUtil.clamp(
    //                 txDeg * Constants.Vision.ROTATIONAL_KP,
    //                 -Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC,
    //                 Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC);

    //         drivetrain.setControl(
    //                 alignRequest
    //                         .withVelocityX(xSupplier.getAsDouble())
    //                         .withVelocityY(ySupplier.getAsDouble())
    //                         .withRotationalRate(rotRate));

    //         // ── 3. Feed: only when both conditions met ─────────────────────────
    //         if (vision.isAligned() && shooter.isReady()) {
    //             indexer.indexerForward();
    //             indexer.conveyorForward();
    //         } else {
    //             indexer.indexerStop();
    //             indexer.conveyorStop();
    //         }

    //     }, shooter, vision, indexer, drivetrain)
    //             .beforeStarting(Commands.runOnce(shooter::beginSpinUp, shooter))
    //             .finallyDo(() -> {
    //                 indexer.indexerStop();
    //                 indexer.conveyorStop();
    //                 shooter.setIdle();
    //             })
    //             .withName("VisionAlignAndShoot");
    // }

    // /**
    //  * Creates a vision-based shoot sequence with indexer coordination.
    //  *
    //  * @param shooter The shooter subsystem
    //  * @param vision  The vision subsystem
    //  * @param indexer The indexer subsystem
    //  * @return Vision-based shooting sequence
    //  */
    // public static Command visionShootSequence(ShooterSubsystem shooter, VisionSubsystem vision,
    //         IndexerSubsystem indexer) {
    //     return Commands.sequence(
    //             Commands.waitUntil(vision::hasTarget)/* REMOVED .withTimeout(1.0) */,
    //             visionShot(shooter, vision)
    //     ).withName("VisionShootSequence");
    // }

    // /**
    //  * Creates a command to shoot using vision-based distance calculation.
    //  *
    //  * @param shooter The shooter subsystem
    //  * @param vision  The vision subsystem
    //  * @return Command that uses vision for aiming
    //  */
    // public static Command visionShot(ShooterSubsystem shooter, VisionSubsystem vision) {
    //     return Commands.sequence(
    //             Commands.runOnce(() -> {
    //                 double distance = vision.getDistanceToTargetMeters();
    //                 shooter.updateFromDistance(distance);
    //                 shooter.beginSpinUp(); // void — transitions state machine to SPINNING_UP
    //             }, shooter, vision),
    //             Commands.waitUntil(shooter::isReady)/* REMOVED .withTimeout(3.0) */)
    //             .withName("VisionShot");
    // }

    // /**
    //  * Creates a command to continuously update shooter based on vision.
    //  *
    //  * @param shooter The shooter subsystem
    //  * @param vision  The vision subsystem
    //  * @return Command that continuously tracks target
    //  */

    // public static Command trackTarget(ShooterSubsystem shooter, VisionSubsystem vision) {
    //     return Commands.run(() -> {
    //         if (vision.hasTarget()) {
    //             double distance = vision.getDistanceToTargetMeters();
    //             shooter.updateFromDistance(distance);
    //         }
    //     }, shooter, vision)
    //             .withName("TrackTarget");
    // }

    public static class Auto {
        // ============================================================================
    // ADD THIS METHOD to ShooterCommands.java (or FuelCommands.java if that's the
    // current file name on the 362-auton-vision-shot-again branch)
    //
    // REQUIRED IMPORTS (add to existing import block if not already present):
    //   import edu.wpi.first.math.MathUtil;
    //   import edu.wpi.first.math.controller.PIDController;
    //   import com.ctre.phoenix6.swerve.SwerveRequest;
    //   import frc.robot.subsystems.CommandSwerveDrivetrain;
    // ============================================================================

    // [UNBOUND] visionShot_version1 — not registered in AutoRoutines or autoChooser.
    // All active auto routines use Auto.poseAlignAndShoot instead.
    /**
     * Vision align and shoot sequence — designed for Choreo autonomous event
     * markers.
     *
     * Choreo handles the bulk of robot positioning; this command handles final
     * adjustments:
     * Phase 1 (parallel):
     * - Branch A: Rotates robot using vision PID to center on the AprilTag
     * - Branch B: Spins up shooter using vision distance for RPM and hood angle
     * Both branches must complete before advancing to Phase 2.
     * Phase 2: Feed the game piece to the shooter
     * Phase 3: Return shooter to idle
     *
     * The overall 5-second timeout acts as the safety stop condition (replaces the
     * "hasShotBoolean" trigger you'd wire up later — swap .withTimeout(5.0) for
     * .until(hasShotBooleanSupplier) when that sensor is ready).
     *
     * CHOREO REGISTRATION (in RobotContainer.java configureNamedCommands()):
     * NamedCommands.registerCommand("Shoot",
     * ShooterCommands.visionAlignAndShoot(shooter, vision, indexer, drivetrain));
     *
     * @param shooter    The shooter subsystem
     * @param vision     The vision subsystem
     * @param indexer    The indexer subsystem
     * @param drivetrain The drivetrain (required by the alignment branch)
     * @return Complete vision-assisted auto shoot sequence
     */
    // public static Command visionShot_version1(
    //         ShooterSubsystem shooter,
    //         VisionSubsystem vision,
    //         IndexerSubsystem indexer,
    //         CommandSwerveDrivetrain drivetrain) {
    //     // PID for rotational alignment — mirrors VisionAlignToTargetCommand constants.
    //     // Created fresh each call so no stale integral state between auto shots.
    //     // TODO: Tune kP and kD. Start with just kP = 0.05 and work up.
    // PIDController alignPID = new PIDController(0.05, 0.0, 0.005);
    //     alignPID.setTolerance(2.0); // degrees — matches VisionConstants.ALIGNMENT_TOLERANCE_DEGREES

    //     // Field-centric drive request — same type as VisionAlignToTargetCommand
    //     SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

    //     return Commands.sequence(

    //             // ==== Phase 1: Align AND spin up at the same time =========================
    //             // Using parallel() so we don't waste time doing these steps sequentially.
    //             // The parallel block finishes only when BOTH branches are done.
    //             Commands.parallel(

    //                     // Branch A: Rotate robot to face the AprilTag
    //                     // Mirrors the logic in reference/VisionAlignToTargetCommand.java
    //                     Commands.run(() -> {
    //                         if (vision.hasTarget()) {
    //                             // Positive tx = target to the right → rotate counterclockwise to correct
    //                             // If robot spins the wrong direction, flip the sign on rotationSpeed
    //                             double tx = vision.getHorizontalAngleDegrees();
    //                             double rotationSpeed = MathUtil.clamp(
    //                                     -alignPID.calculate(tx, 0), // negative = correct direction for FieldCentric
    //                                     -1.0, 1.0 // TODO: Tune max rotation speed (rad/s)
    //                             );
    //                             drivetrain.setControl(driveRequest
    //                                     .withVelocityX(0)
    //                                     .withVelocityY(0)
    //                                     .withRotationalRate(rotationSpeed));
    //                         } else {
    //                             // No target — hold still and wait
    //                             drivetrain.setControl(driveRequest
    //                                     .withVelocityX(0)
    //                                     .withVelocityY(0)
    //                                     .withRotationalRate(0));
    //                         }
    //                     }, drivetrain)
    //                             .until(vision::isAligned) // VisionSubsystem already tracks this via state machine
    //                             .withTimeout(3.0), // Safety — don't spin forever if target disappears

    //                     // Branch B: Set shooter RPM and hood angle from vision distance, wait until
    //                     // ready
    //                     // Reuses the existing visionShot() factory method unchanged
    //                     visionShot(shooter, vision) // already includes .withTimeout(3.0)
    //             ),

    //             // ==== Phase 2: Both aligned and shooter ready — fire =====================
    //             indexer.feedTimed( 0.5),

    //             // ==== Phase 3: Return to idle ===========================================
    //             Commands.runOnce(shooter::setIdle, shooter)

    //     )
    //             // Overall timeout = the "boolean stop" for now.
    //             // Later: swap .withTimeout(5.0) for .until(hasShotBooleanSupplier)
    //             .withTimeout(5.0)
    //             .withName("VisionAlignAndShoot");
    // }

    // ============================================================================
    // RobotContainer.java — add to configureNamedCommands() or wherever you
    // register Choreo event markers. This wires the "Shoot" event marker in your
    // .chor file to the new command.
    // ============================================================================

    /*
    * // In RobotContainer.java — inside configureNamedCommands() or the
    * constructor:
    * 
    * NamedCommands.registerCommand("Shoot",
    * ShooterCommands.visionAlignAndShoot(shooter, vision, indexer, drivetrain));
    * 
    * // If you want the shooter pre-warming while Choreo drives to position,
    * // also register a "SpinUp" marker to fire earlier in the path:
    * NamedCommands.registerCommand("SpinUp",
    * ShooterCommands.visionShot(shooter, vision)); // start warming early
    */

    // ============================================================================
    // TUNING NOTES FOR STUDENTS
    // ============================================================================
    /*
    * The two numbers students are most likely to need to adjust:
    * 
    * 1. alignPID kP (currently 0.05):
    * - Too low → robot barely turns, takes forever to align
    * - Too high → robot oscillates back and forth around the target
    * - Start at 0.05, increase slowly until it snaps to target quickly without
    * overshoot
    * 
    * 2. Max rotation clamp (currently 1.0 rad/s):
    * - Reduces this if the robot overshoots badly during auto
    * - Try 0.5 rad/s as a conservative starting point
    * 
    * 3. feedTimed duration (currently 0.5 seconds):
    * - Increase if game pieces aren't making it all the way through
    * - Decrease if you're wasting time waiting after the piece is gone
    * 
    * HOW TO VERIFY IT'S WORKING:
    * - Watch the Vision/IsAligned signal on Elastic during auto replay
    * - In AdvantageScope: plot "Vision/HorizontalAngle_deg" — should approach 0
    * before feed fires
    * - If Phase 1 times out at 3s, the robot still proceeds to feed (by design for
    * now)
    * If you want to SKIP the shot on timeout, wrap the sequence in:
    * .onlyIf(() -> vision.hasTarget())
    */

    // =========================================================================



        // =============================================================================
        /**
         * Autonomous odometry-based align-and-shoot.
         *
         * Uses drivetrain odometry to compute heading error to the hub — no vision,
         * no driver input, no lead compensation (robot is stopped).
         *
         * Phase 1 (deadline): rotate toward hub + spin up shooter simultaneously.
         *   Ends when heading error ≤ ALIGNMENT_TOLERANCE_DEGREES AND shooter isReady(),
         *   or after a 3-second safety timeout.
         * Phase 2: feed indexer for feedSeconds.
         * finallyDo: stop indexer/conveyor, return shooter to idle.
         *
         * Use at Choreo "Shoot" event markers. Because this command requires the
         * drivetrain, it will interrupt (take over from) the Choreo path command
         * the moment it is scheduled — place the event marker at the end of the
         * segment or after the robot has reached its shooting position.
         *
         * @param shooter     Shooter subsystem
         * @param indexer     Indexer subsystem
         * @param drivetrain  Drivetrain (odometry pose source)
         * @param feedSeconds How long to run the indexer/conveyor after alignment
         * @return Autonomous align-and-shoot command
         */
        public static Command poseAlignAndShoot(
                ShooterSubsystem shooter,
                IndexerSubsystem indexer,
                CommandSwerveDrivetrain drivetrain,
                double safetyTimeout) {

            final SwerveRequest.FieldCentric alignRequest = new SwerveRequest.FieldCentric()
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

            return Commands.sequence(
                    // Phase 1: rotate to hub + spin up — both must be ready before feeding
                    Commands.deadline(
                            Commands.waitUntil(() -> {
                                Translation2d hub = getHubLocation();
                                Pose2d pose = drivetrain.getState().Pose;
                                double dx = hub.getX() - pose.getX();
                                double dy = hub.getY() - pose.getY();
                                double headingErrorDeg = Math.toDegrees(Math.atan2(dy, dx))
                                        - pose.getRotation().getDegrees();
                                while (headingErrorDeg >  180) headingErrorDeg -= 360;
                                while (headingErrorDeg < -180) headingErrorDeg += 360;
                                return Math.abs(headingErrorDeg) <= Constants.Vision.ALIGNMENT_TOLERANCE_DEGREES
                                        && shooter.isReady();
                            })/* REMOVED .withTimeout(3.0) */,
                            Commands.run(() -> {
                                Translation2d hub = getHubLocation();
                                Pose2d pose = drivetrain.getState().Pose;
                                double dx = hub.getX() - pose.getX();
                                double dy = hub.getY() - pose.getY();
                                double distance = MathUtil.clamp(
                                        Math.hypot(dx, dy),
                                        Constants.Vision.MIN_DISTANCE_M,
                                        Constants.Vision.MAX_DISTANCE_M);
                                shooter.updateFromDistance(distance);
                                if (shooter.getState() != ShooterSubsystem.ShooterState.READY) {
                                    shooter.beginSpinUp();
                                }
                                double headingErrorDeg = Math.toDegrees(Math.atan2(dy, dx))
                                        - pose.getRotation().getDegrees();
                                while (headingErrorDeg >  180) headingErrorDeg -= 360;
                                while (headingErrorDeg < -180) headingErrorDeg += 360;
                                double rotRate = MathUtil.clamp(
                                        headingErrorDeg * Constants.Vision.ROTATIONAL_KP,
                                        -Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC,
                                        Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC);
                                drivetrain.setControl(alignRequest
                                        .withVelocityX(0)
                                        .withVelocityY(0)
                                        .withRotationalRate(rotRate));
                            }, shooter, drivetrain)),
                    // Phase 2: feed until chute clears (safetyTimeout is the fallback)
                    indexer.feedUntilChuteEmpty(safetyTimeout)
            ).finallyDo(() -> {
                indexer.indexerStop();
                indexer.conveyorStop();
                shooter.setIdle();
            }).withName("Auto.PoseAlignAndShoot");
        }

        // =============================================================================
        // Presets
        // =============================================================================
        public static Command shootTrench(ShooterSubsystem shooter, IndexerSubsystem indexer,
                double safetyTimeout) {
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        shooter.setTargetVelocity(Constants.Shooter.TRENCH_RPM);
                        shooter.setTargetHoodPose(Constants.Shooter.TRENCH_HOOD);
                        shooter.beginSpinUp();
                    }, shooter),
                    Commands.waitUntil(shooter::isReady),
                    indexer.feedUntilChuteEmpty(safetyTimeout)
            ).finallyDo(() -> {
                indexer.indexerStop();
                indexer.conveyorStop();
                shooter.setIdle();
            }).withName("ShootTrenchAuton");
        }

        public static Command shootTrenchWithSlideRetract(ShooterSubsystem shooter,
                IndexerSubsystem indexer, IntakeSubsystem intake, double safetyTimeout) {
            return Commands.deadline(
                    shootTrench(shooter, indexer, safetyTimeout),
                    intake.retractSlidesAuton())
                    .withName("ShootTrenchWithSlideRetract");
        }

        public static Command shootHub(ShooterSubsystem shooter, IndexerSubsystem indexer,
                double safetyTimeout) {
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        shooter.setTargetVelocity(Constants.Shooter.CLOSE_RPM);
                        shooter.setTargetHoodPose(Constants.Shooter.CLOSE_HOOD);
                        shooter.beginSpinUp();
                    }, shooter),
                    Commands.waitUntil(shooter::isReady),
                    indexer.feedUntilChuteEmpty(safetyTimeout)).finallyDo(() -> {
                        indexer.indexerStop();
                        indexer.conveyorStop();
                        shooter.setIdle();
                    }).withName("ShootHubAuton");
        }

        public static Command shootTower(ShooterSubsystem shooter, IndexerSubsystem indexer,
                double safetyTimeout) {
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        shooter.setTargetVelocity(Constants.Shooter.TOWER_RPM);
                        shooter.setTargetHoodPose(Constants.Shooter.TOWER_HOOD);
                        shooter.beginSpinUp();
                    }, shooter),
                    Commands.waitUntil(shooter::isReady).withTimeout(6.0),
                    indexer.feedUntilChuteEmpty(safetyTimeout)).finallyDo(() -> {
                        indexer.indexerStop();
                        indexer.conveyorStop();
                        shooter.setIdle();
                    }).withName("ShootTowerAuton");
        } // end of command

        public static Command shootFar(ShooterSubsystem shooter, IndexerSubsystem indexer,
                double safetyTimeout) {
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        shooter.setTargetVelocity(Constants.Shooter.FAR_RPM);
                        shooter.setTargetHoodPose(Constants.Shooter.FAR_HOOD);
                        shooter.beginSpinUp();
                    }, shooter),
                    Commands.waitUntil(shooter::isReady).withTimeout(6.0),
                    indexer.feedUntilChuteEmpty(safetyTimeout)).finallyDo(() -> {
                        indexer.indexerStop();
                        indexer.conveyorStop();
                        shooter.setIdle();
                    }).withName("ShootFarAuton");
        } // end of command


    } 
    // end of class Auto

} // end of class FuelCommands