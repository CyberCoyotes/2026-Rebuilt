package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
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
                Commands.runOnce(shooter::prepareToShoot, shooter),
                Commands.waitUntil(shooter::isReady).withTimeout(3.0),
                Commands.run(() -> {
                    indexer.indexerForward();
                    indexer.conveyorForward();
                }, indexer)).finallyDo(() -> {
                    indexer.indexerStop();
                    indexer.conveyorStop();
                    shooter.setIdle(); // Stop all flywheel motors on trigger release
                }).withName("ShootAtCurrentTarget");
    }

    /**
     * Full preset shoot sequence that owns both the shooter and indexer subsystems.
     *
     * Flow:
     * 1. Sets the given RPM + hood target silently
     * 2. Calls prepareToShoot() — flywheel ramps up, hood moves to target
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
                    shooter.prepareToShoot();
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
                intake.retractSlidesWithRollerCmd())
                .withName("ShootWithSlideRetract[" + rpm + "rpm]");
    }

    // Template for the using ShotPresets in Auton
    public static Command shootPresetAuton(ShooterSubsystem shooter, IndexerSubsystem indexer,
            ShotPreset preset, double feedSeconds,
            boolean waitForReady) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    shooter.setTargetVelocity(preset.rpm);
                    shooter.setTargetHoodPose(preset.hood);
                    shooter.prepareToShoot();
                }, shooter),
                Commands.waitUntil(shooter::isReady).withTimeout(3.0),
                Commands.run(() -> {
                    indexer.indexerForward();
                    indexer.conveyorForward();
                }, indexer).withTimeout(feedSeconds) // primary end trigger: timeout
        ).finallyDo(() -> {
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
     * Air popper assist _without_ intake.
     *
     * Holds the shooter at POPPER preset while feeding the indexer/conveyor.
     * Use when an intake command is already being scheduled elsewhere.
     */
    // public static Command runAirPopperTest(IndexerSubsystem indexer,
    // ShooterSubsystem shooter) {
    // return Commands.sequence(
    // Commands.runOnce(() -> {
    // shooter.setAirPopper();
    // shooter.prepareToShoot();
    // }, shooter),
    // Commands.deadline(
    // indexer.feed(),
    // Commands.run(() -> {
    // }, shooter))).finallyDo(() -> {
    // indexer.indexerStop();
    // indexer.conveyorStop();
    // shooter.setIdle();
    // }).withName("RunAirPopper");
    // }

    /**
     * Air popper assist with intake.
     *
     * While held: runs intake + indexer feed while shooter stays in POPPER preset.
     */
    public static Command runAirPopper(IndexerSubsystem indexer, ShooterSubsystem shooter,
            IntakeSubsystem intake) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    shooter.setAirPopper();
                    shooter.prepareToShoot();
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
                    shooter.setTargetVelocity(Constants.Shooter.PASS_RPM);
                    shooter.setTargetHoodPose(Constants.Shooter.PASS_HOOD);
                    shooter.prepareToShoot();
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
                }).withName("ShootPass");
    }

    /**
     * Creates a command to return shooter to SPINUP at STANDBY_RPM.
     *
     * @param shooter The shooter subsystem
     * @return Command that returns shooter to standby spinning state
     */

    /** TODO: Do not use right now _EXPERIMENTAL_ */
    // public static Command returnToStandby(ShooterSubsystem shooter) {
    // return Commands.runOnce(shooter::returnToStandby, shooter)
    // .withName("ReturnToStandby");
    // }

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
                shooter.prepareToShoot();
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
            }

        }, shooter, indexer, drivetrain)
                .beforeStarting(Commands.runOnce(shooter::prepareToShoot, shooter))
                .finallyDo(() -> {
                    indexer.indexerStop();
                    indexer.conveyorStop();
                    shooter.setIdle();
                })
                .withName("PoseAlignAndShoot");
    }

    // =========================================================================
    // VISION ALIGN AND SHOOT (limelight tx — secondary/fallback)
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
    public static Command visionAlignAndShoot(
            ShooterSubsystem shooter,
            VisionSubsystem vision,
            IndexerSubsystem indexer,
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {

        final double maxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

        // Created once; reused every execute cycle.
        // FieldCentric: driver controls X/Y translation, vision controls rotation.
        final SwerveRequest.FieldCentric alignRequest = new SwerveRequest.FieldCentric()
                .withDeadband(maxSpeed * 0.15)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        return Commands.run(() -> {

            // ── 1. Shooter: update from vision or hold current targets ─────────
            if (vision.isUsableForShooting()) {
                // Live-update RPM + hood from distance table (also pushes to hardware
                // if already in READY state — see ShooterSubsystem.updateFromDistance)
                shooter.updateFromDistance(vision.getDistanceToTargetMeters());
            }
            // Keep commanding READY every cycle — setState() no-ops if already there
            if (shooter.getState() != ShooterSubsystem.ShooterState.READY) {
                shooter.prepareToShoot();
            }

            // ── 2. Drivetrain: driver translation + vision rotation ────────────
            // tx returns 0.0 when NO_TARGET, so rotation correction drops to zero
            // automatically when the camera has nothing to track.
            double txDeg = vision.getHorizontalAngleDegrees();
            double rotRate = MathUtil.clamp(
                    txDeg * Constants.Vision.ROTATIONAL_KP,
                    -Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC,
                    Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC);

            drivetrain.setControl(
                    alignRequest
                            .withVelocityX(xSupplier.getAsDouble())
                            .withVelocityY(ySupplier.getAsDouble())
                            .withRotationalRate(rotRate));

            // ── 3. Feed: only when both conditions met ─────────────────────────
            if (vision.isAligned() && shooter.isReady()) {
                indexer.indexerForward();
                indexer.conveyorForward();
            } else {
                indexer.indexerStop();
                indexer.conveyorStop();
            }

        }, shooter, vision, indexer, drivetrain)
                .beforeStarting(Commands.runOnce(() -> {
                    shooter.prepareToShoot(); // Enter READY state — don't wait for alignment
                }, shooter))
                .finallyDo(() -> {
                    indexer.indexerStop();
                    indexer.conveyorStop();
                    shooter.setIdle();
                })
                .withName("VisionAlignAndShoot");
    }

    /**
     * Creates a vision-based shoot sequence with indexer coordination.
     *
     * @param shooter The shooter subsystem
     * @param vision  The vision subsystem
     * @param indexer The indexer subsystem
     * @return Vision-based shooting sequence
     */
    public static Command visionShootSequence(ShooterSubsystem shooter, VisionSubsystem vision,
            IndexerSubsystem indexer) {
        return Commands.sequence(
                Commands.waitUntil(vision::hasTarget).withTimeout(1.0),
                visionShot(shooter, vision)
        // IndexerCommands.feedTimed(indexer, 0.5), // FIXME to use the
        // IndexerSubsystem's feed method instead of a command from IndexerCommands
        // Commands.runOnce(shooter::returnToStandby, shooter)
        ).withName("VisionShootSequence");
    }

    /**
     * Creates a command to shoot using vision-based distance calculation.
     *
     * @param shooter The shooter subsystem
     * @param vision  The vision subsystem
     * @return Command that uses vision for aiming
     */
    public static Command visionShot(ShooterSubsystem shooter, VisionSubsystem vision) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    double distance = vision.getDistanceToTargetMeters();
                    shooter.updateFromDistance(distance);
                    shooter.prepareToShoot();
                }, shooter, vision),
                Commands.waitUntil(shooter::isReady).withTimeout(3.0))
                .withName("VisionShot");
    }

    /**
     * Creates a command to continuously update shooter based on vision.
     *
     * @param shooter The shooter subsystem
     * @param vision  The vision subsystem
     * @return Command that continuously tracks target
     */

    public static Command trackTarget(ShooterSubsystem shooter, VisionSubsystem vision) {
        return Commands.run(() -> {
            if (vision.hasTarget()) {
                double distance = vision.getDistanceToTargetMeters();
                shooter.updateFromDistance(distance);
            }
        }, shooter, vision)
                .withName("TrackTarget");
    }

    public static class Auto {

        /**
         * Autonomous pose-aligned shoot command.
         *
         * Replaces fixed-preset auton shots where the robot needs to rotate to face
         * the hub before firing. Uses the same odometry-based heading math as
         * {@link FuelCommands#poseAlignAndShoot} but with no driver translation input.
         *
         * Sequence:
         * 1. Spins up shooter from pose distance while rotating toward hub.
         * 2. Waits until heading error ≤ ALIGNMENT_TOLERANCE_DEGREES AND shooter isReady()
         *    (3-second timeout — fires anyway if alignment cannot be reached in time).
         * 3. Feeds the indexer for {@code feedSeconds}.
         * 4. Returns shooter to idle and stops indexer/conveyor.
         *
         * @param shooter     Shooter subsystem
         * @param indexer     Indexer subsystem
         * @param drivetrain  Swerve drivetrain (provides field pose)
         * @param feedSeconds How long to run the indexer/conveyor after aligned and ready
         * @return Autonomous align-and-shoot command
         */
        public static Command autoAlignAndShoot(
                ShooterSubsystem shooter,
                IndexerSubsystem indexer,
                CommandSwerveDrivetrain drivetrain,
                double feedSeconds) {

            final SwerveRequest.FieldCentric alignRequest = new SwerveRequest.FieldCentric()
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

            return Commands.sequence(
                    // Phase 1: Spin up + rotate to hub simultaneously
                    Commands.deadline(
                            Commands.waitUntil(() -> {
                                Translation2d hub = getHubLocation();
                                Pose2d pose = drivetrain.getState().Pose;
                                double dx = hub.getX() - pose.getX();
                                double dy = hub.getY() - pose.getY();
                                double headingError = Math.toDegrees(Math.atan2(dy, dx))
                                        - pose.getRotation().getDegrees();
                                while (headingError >  180) headingError -= 360;
                                while (headingError < -180) headingError += 360;
                                return Math.abs(headingError) <= Constants.Vision.ALIGNMENT_TOLERANCE_DEGREES
                                        && shooter.isReady();
                            }).withTimeout(3.0),
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
                                    shooter.prepareToShoot();
                                }
                                double headingError = Math.toDegrees(Math.atan2(dy, dx))
                                        - pose.getRotation().getDegrees();
                                while (headingError >  180) headingError -= 360;
                                while (headingError < -180) headingError += 360;
                                double rotRate = MathUtil.clamp(
                                        headingError * Constants.Vision.ROTATIONAL_KP,
                                        -Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC,
                                        Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC);
                                drivetrain.setControl(
                                        alignRequest
                                                .withVelocityX(0)
                                                .withVelocityY(0)
                                                .withRotationalRate(rotRate));
                            }, shooter, drivetrain)),
                    // Phase 2: Feed
                    indexer.feed().withTimeout(feedSeconds))
                    .finallyDo(() -> {
                        indexer.indexerStop();
                        indexer.conveyorStop();
                        shooter.setIdle();
                    }).withName("AutoAlignAndShoot");
        }

        /* Autonomous shooting command */
        /* FIXME Trench is the working "Test" command for autonomous. Others should be updated after its working properly */
        public static Command shootTrench(ShooterSubsystem shooter, IndexerSubsystem indexer,
                double feedSeconds) {
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        shooter.setTargetVelocity(Constants.Shooter.TRENCH_RPM);
                        shooter.setTargetHoodPose(Constants.Shooter.TRENCH_HOOD);
                        shooter.prepareToShoot();
                    }, shooter),
                    Commands.waitUntil(shooter::isReady), // removed timeout
                    indexer.feed().withTimeout(feedSeconds) // removed sensor time
            ).finallyDo(() -> {
                indexer.indexerStop();
                indexer.conveyorStop();
                shooter.setIdle();
            }).withName("ShootTrenchAuton");
        }

        /**
         * Autonomous: shoot at trench preset while retracting intake slides (two-phase)
         * and running the intake roller concurrently.
         *
         * The shoot sequence ({@link #shootTrench}) is the deadline — slide retraction
         * is cancelled when feeding completes. Safe to compose with path-following or
         * other parallel commands in auton.
         *
         * @param shooter     The shooter subsystem
         * @param indexer     The indexer subsystem
         * @param intake      The intake subsystem
         * @param feedSeconds How long to run the indexer/conveyor after ready
         * @return Deadline command: shootTrench (deadline) + slide retract in parallel
         */
        public static Command shootTrenchWithSlideRetract(ShooterSubsystem shooter,
                IndexerSubsystem indexer, IntakeSubsystem intake, double feedSeconds) {
            return Commands.deadline(
                    shootTrench(shooter, indexer, feedSeconds),
                    intake.retractSlidesWithRollerCmd())
                    .withName("ShootTrenchWithSlideRetract");
        }

        /* Autonomous shooting command */
        public static Command shootHub(ShooterSubsystem shooter, IndexerSubsystem indexer,
                double feedSeconds) {
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        shooter.setTargetVelocity(Constants.Shooter.CLOSE_RPM);
                        shooter.setTargetHoodPose(Constants.Shooter.CLOSE_HOOD);
                        shooter.prepareToShoot();
                    }, shooter),
                    Commands.waitUntil(shooter::isReady), // remove timeout

                    // remove indexer::donePassingFuel for now
                    indexer.feed().withTimeout(feedSeconds)).finallyDo(() -> {
                        indexer.indexerStop();
                        indexer.conveyorStop();
                        shooter.setIdle();
                    }).withName("ShootHubAuton");
        }

        /* Autonomous shooting command */
        public static Command shootTower(ShooterSubsystem shooter, IndexerSubsystem indexer,
                double feedSeconds) {
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        shooter.setTargetVelocity(Constants.Shooter.TOWER_RPM);
                        shooter.setTargetHoodPose(Constants.Shooter.TOWER_HOOD);
                        shooter.prepareToShoot();
                    }, shooter),
                    Commands.waitUntil(shooter::isReady).withTimeout(6.0),
                    indexer.feed().until(indexer::donePassingFuel).withTimeout(feedSeconds)).finallyDo(() -> {
                        indexer.indexerStop();
                        indexer.conveyorStop();
                        shooter.setIdle();
                    }).withName("ShootTowerAuton");
        } // end of command
        /* Autonomous shooting command */

        public static Command shootFar(ShooterSubsystem shooter, IndexerSubsystem indexer,
                double feedSeconds) {
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        shooter.setTargetVelocity(Constants.Shooter.FAR_RPM);
                        shooter.setTargetHoodPose(Constants.Shooter.FAR_HOOD);
                        shooter.prepareToShoot();
                    }, shooter),
                    Commands.waitUntil(shooter::isReady).withTimeout(6.0),
                    indexer.feed().until(indexer::donePassingFuel).withTimeout(feedSeconds)).finallyDo(() -> {
                        indexer.indexerStop();
                        indexer.conveyorStop();
                        shooter.setIdle();
                    }).withName("ShootFarAuton");
        } // end of command

        /* Autonomous shooting command */
        public static Command shootTren(ShooterSubsystem shooter, IndexerSubsystem indexer,
                double feedSeconds) {
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        shooter.setTargetVelocity(Constants.Shooter.TRENCH_RPM);
                        shooter.setTargetHoodPose(Constants.Shooter.TRENCH_HOOD);
                        shooter.prepareToShoot();
                    }, shooter),
                    Commands.waitUntil(shooter::isReady).withTimeout(6.0),
                    indexer.feed().until(indexer::donePassingFuel).withTimeout(feedSeconds)).finallyDo(() -> {
                        indexer.indexerStop();
                        indexer.conveyorStop();
                        shooter.setIdle();
                    }).withName("ShootTrenchAuton");
        }
    } 
    // end of class Auto

} // end of class FuelCommands