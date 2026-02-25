package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
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
 * Current Test Shot Flow:
 * - Select a preset with POV left/right (sets target RPM + hood, but does not move yet)
 * - Driver holds shoot trigger;flywheel ramps to target, hood moves, waits until ready, feeds
 * - Driver releases trigger; returns to IDLE (TODO: STANDBY once spin logic validated)
 *
 */
public class FuelCommands {

    // =========================================================================
    // PRIMARY SHOOT COMMAND
    // =========================================================================

    /**
     * Shoots at whatever preset is currently armed (set by A/X/B).
     *
     * Behavior:
     * - Transitions shooter to READY — flywheel ramps to target RPM, hood moves to target angle
     * - Waits until both flywheel and hood are at target (isReady())
     * - Runs indexer and conveyor to feed the game piece
     * - On button release, stops indexer and returns shooter to SPINUP at IDLE_RPM
     *
     * Use with whileTrue() on the shoot trigger.
     *
     * @param shooter The shooter subsystem
     * @param indexer The indexer subsystem
     * @return Command that shoots at current targets and returns to idle on release
     */
    public static Command shootAtCurrentTarget(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return Commands.sequence(
            Commands.runOnce(shooter::prepareToShoot, shooter),
            Commands.waitUntil(shooter::isReady).withTimeout(1.0),
            Commands.run(() -> {
                // indexer.indexerForward();
                // indexer.conveyorForward();
            }, indexer)
        ).finallyDo(() -> {
            indexer.indexerStop();
            indexer.conveyorStop();
            shooter.setIdle(); // Stop all flywheel motors on trigger release
        }).withName("ShootAtCurrentTarget");
    }

    // =========================================================================
    // PRESET SHOOT COMMANDS (pseudo-superstructure — requires shooter + indexer)
    // =========================================================================

    /**
     * Full preset shoot sequence that owns both the shooter and indexer subsystems.
     *
     * Flow:
     *   1. Arms the given RPM + hood target silently
     *   2. Calls prepareToShoot() — flywheel ramps up, hood moves to target
     *   3. Waits until both are at target (isReady()), with a 3-second safety timeout
     *   4. Runs indexer and conveyor forward to feed the game piece
     *   5. On trigger release (whileTrue interrupt): stops indexer/conveyor, returns shooter to standby
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
                shooter.setTargetVelocity(rpm);
                shooter.setTargetHoodPose(hood);
                shooter.prepareToShoot();
            }, shooter),
            Commands.waitUntil(shooter::isReady).withTimeout(3.0),
            Commands.run(() -> {
                indexer.indexerForward();
                indexer.conveyorForward();
            }, indexer)
        ).finallyDo(() -> {
            indexer.indexerStop();
            indexer.conveyorStop();
            shooter.setIdle(); // Stop all flywheel motors on trigger release
        }).withName("ShootWithPreset[" + rpm + "rpm]");
    }

    /**
     * Autonomous-friendly preset shoot command using a named ShotPreset enum value.
     *
     * Ends via timeout (primary) — the {@code waitForReady} flag is reserved for future
     * use once sensor-based game-piece detection is available.
     *
     * Flow:
     *   1. Arms the given preset (RPM + hood) silently
     *   2. Calls prepareToShoot() — flywheel ramps up, hood moves to target
     *   3. Waits until both are at target (isReady()), with a 3-second safety timeout
     *   4. Runs indexer and conveyor for {@code feedSeconds} to deliver the game piece
     *   5. Stops indexer/conveyor and returns shooter to idle
     *
     * Typical auton usage:
     * <pre>
     *   FuelCommands.shootPresetAuton(shooter, indexer, ShotPreset.TRENCH, 1.0)
     * </pre>
     *
     * @param shooter      The shooter subsystem
     * @param indexer      The indexer subsystem
     * @param preset       The named shot preset (TRENCH, CLOSE, TOWER, …)
     * @param feedSeconds  How long to run the indexer/conveyor after ready; acts as the
     *                     end trigger (timeout). Tune per game-piece count needed.
     * @param waitForReady Reserved for future use — when true, will gate the feed on a
     *                     game-piece sensor rather than a timer. Pass {@code false} for now.
     * @return Complete autonomous shoot sequence
     */
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
     * Autonomous: shoot using the TRENCH preset.
     * Ends after {@code feedSeconds} of feed time (timeout).
     *
     * @param shooter     The shooter subsystem
     * @param indexer     The indexer subsystem
     * @param feedSeconds How long to run the indexer/conveyor after ready
     * @return Autonomous TRENCH shoot command
     */
    public static Command shootTrenchAuton(ShooterSubsystem shooter, IndexerSubsystem indexer,
                                           double feedSeconds) {
        return shootPresetAuton(shooter, indexer, ShotPreset.TRENCH, feedSeconds, false)
            .withName("ShootTrenchAuton");
    }

    /**
     * Autonomous: shoot using the CLOSE preset.
     * Ends after {@code feedSeconds} of feed time (timeout).
     *
     * @param shooter     The shooter subsystem
     * @param indexer     The indexer subsystem
     * @param feedSeconds How long to run the indexer/conveyor after ready
     * @return Autonomous CLOSE shoot command
     */
    public static Command shootCloseAuton(ShooterSubsystem shooter, IndexerSubsystem indexer,
                                          double feedSeconds) {
        return shootPresetAuton(shooter, indexer, ShotPreset.CLOSE, feedSeconds, false)
            .withName("ShootCloseAuton");
    }

    /**
     * Autonomous: shoot using the TOWER preset.
     * Ends after {@code feedSeconds} of feed time (timeout).
     *
     * @param shooter     The shooter subsystem
     * @param indexer     The indexer subsystem
     * @param feedSeconds How long to run the indexer/conveyor after ready
     * @return Autonomous TOWER shoot command
     */
    public static Command shootTowerAuton(ShooterSubsystem shooter, IndexerSubsystem indexer,
                                          double feedSeconds) {
        return shootPresetAuton(shooter, indexer, ShotPreset.TOWER, feedSeconds, false)
            .withName("ShootTowerAuton");
    }

    /**
     * Full shoot sequence using the preset currently selected via POV left/right cycling.
     * Defaults to the CLOSE preset (Close RPM + Close hood) on robot startup.
     *
     * This is the primary driver RT command — a pseudo-superstructure command that
     * coordinates shooter and indexer in a single command.
     *
     * Flow:
     *   1. Arms the selected preset (RPM + hood) silently
     *   2. Calls prepareToShoot() — flywheel ramps up, hood moves to target
     *   3. Waits until both are at target (isReady()), with a 3-second safety timeout
     *   4. Runs indexer and conveyor forward to feed the game piece
     *   5. On trigger release (whileTrue interrupt): stops indexer/conveyor, returns to standby
     *
     * Cycle the selected preset with POV left/right on the driver controller.
     * The active preset name is published to Shooter/SelectedPreset on NetworkTables for Elastic.
     *
     * Use with whileTrue() on the shoot trigger.
     *
     * @param shooter The shooter subsystem
     * @param indexer The indexer subsystem
     * @return Complete preset shoot command using the currently selected preset
     */
    public static Command shootWithSelectedPreset(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                shooter.armSelectedPreset();
                shooter.prepareToShoot();
            }, shooter),
            Commands.waitUntil(shooter::isReady).withTimeout(3.0),
            Commands.run(() -> {
                indexer.indexerForward();
                indexer.conveyorForward();
            }, indexer)
        ).finallyDo(() -> {
            indexer.indexerStop();
            indexer.conveyorStop();
            shooter.setIdle(); // Stop all flywheel motors on trigger release
        }).withName("ShootWithSelectedPreset");
    }

    // =========================================================================
    // SILENT PRESET COMMANDS (A / X / B)
    // =========================================================================
    // These only update the target RPM and hood angle.
    // No motors move until the shoot trigger is pressed.

    /**
     * Silently arms close shot targets (CLOSE_SHOT_RPM + CLOSE_SHOT_HOOD).
     * No motor movement. Use with onTrue().
     *
     * @param shooter The shooter subsystem
     * @return Command that silently sets close shot targets
     */
    public static Command armCloseShot(ShooterSubsystem shooter) {
        return Commands.runOnce(shooter::setCloseShotPreset, shooter)
            .withName("ArmCloseShot");
    }

    /**
     * Silently arms far shot targets (FAR_SHOT_RPM + FAR_SHOT_HOOD).
     * No motor movement. Use with onTrue().
     *
     * @param shooter The shooter subsystem
     * @return Command that silently sets far shot targets
     */
    public static Command armFarShot(ShooterSubsystem shooter) {
        return Commands.runOnce(shooter::setFarShotPreset, shooter)
            .withName("ArmFarShot");
    }

    /**
     * Silently arms pass shot targets (PASS_SHOT_RPM + PASS_SHOT_HOOD).
     * No motor movement. Use with onTrue().
     *
     * @param shooter The shooter subsystem
     * @return Command that silently sets pass shot targets
     */
    public static Command armPassShot(ShooterSubsystem shooter) {
        return Commands.runOnce(shooter::setPassShotPreset, shooter)
            .withName("ArmPassShot");
    }

    // =========================================================================
    // HOOD ADJUSTMENT
    // =========================================================================

    /**
     * Increases the target hood pose by the provided increment.
     * If shooter is already in READY state, hood moves immediately.
     *
     * @param shooter The shooter subsystem
     * @param increment Rotations to increase (positive value expected)
     * @return Command that adjusts target hood pose upward
     */
    public static Command increaseTargetHoodPose(ShooterSubsystem shooter, double increment) {
        return Commands.runOnce(() -> shooter.adjustTargetHoodPose(Math.abs(increment)), shooter)
            .withName("IncreaseHoodPose");
    }

    /**
     * Decreases the target hood pose by the provided increment.
     * If shooter is already in READY state, hood moves immediately.
     *
     * @param shooter The shooter subsystem
     * @param increment Rotations to decrease (positive value expected)
     * @return Command that adjusts target hood pose downward
     */
    public static Command decreaseTargetHoodPose(ShooterSubsystem shooter, double increment) {
        return Commands.runOnce(() -> shooter.adjustTargetHoodPose(-Math.abs(increment)), shooter)
            .withName("DecreaseHoodPose");
    }

    // =========================================================================
    // UTILITY COMMANDS
    // =========================================================================

    /**
     * Creates a command to increase target flywheel RPM by the provided increment.
     *
     * @param shooter The shooter subsystem
     * @param incrementRPM RPM delta to apply
     * @return Command that adjusts target flywheel velocity
     */
    public static Command increaseTargetVelocity(ShooterSubsystem shooter, double incrementRPM) {
        return Commands.runOnce(() -> shooter.adjustTargetVelocity(Math.abs(incrementRPM)), shooter)
            .withName("IncreaseShooterTargetVelocity");
    }

    /**
     * Creates a command to decrease target flywheel RPM by the provided increment.
     *
     * @param shooter The shooter subsystem
     * @param decrementRPM RPM delta to apply
     * @return Command that adjusts target flywheel velocity
     */
    public static Command decreaseTargetVelocity(ShooterSubsystem shooter, double decrementRPM) {
        return Commands.runOnce(() -> shooter.adjustTargetVelocity(-Math.abs(decrementRPM)), shooter)
            .withName("DecreaseShooterTargetVelocity");
    }

    /**
     * Creates a command to eject jammed game pieces.
     * Reverses flywheel for a duration then returns to SPINUP.
     *
     * @param shooter The shooter subsystem
     * @param durationSeconds How long to eject
     * @return Command that ejects then returns to idle
     */
    public static Command eject(ShooterSubsystem shooter, double durationSeconds) {
        return Commands.sequence(
            Commands.runOnce(shooter::eject, shooter),
            Commands.waitSeconds(durationSeconds)
            // Commands.runOnce(shooter::returnToStandby, shooter) // TODO: Do not use right now **EXPERIMENTAL**
        ).withName("EjectShooter");
    }

    /**
     * Creates a command to shoot using vision-based distance calculation.
     *
     * @param shooter The shooter subsystem
     * @param vision The vision subsystem
     * @return Command that uses vision for aiming
     */
    public static Command visionShot(ShooterSubsystem shooter, VisionSubsystem vision) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                double distance = vision.getDistanceToTargetMeters();
                shooter.updateFromDistance(distance);
                shooter.prepareToShoot();
            }, shooter, vision),
            Commands.waitUntil(shooter::isReady)
        ).withTimeout(3.0)
         .withName("VisionShot");
    }

    /**
     * Creates a command to continuously update shooter based on vision.
     *
     * @param shooter The shooter subsystem
     * @param vision The vision subsystem
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

    /**
     * Creates a complete shoot sequence with indexer coordination.
     *
     * @param shooter The shooter subsystem
     * @param indexer The indexer subsystem
     * @param velocityRPM Target flywheel velocity
     * @param hoodAngleDegrees Target hood angle
     * @return Complete shooting sequence
     */
    public static Command shootSequence(ShooterSubsystem shooter, IndexerSubsystem indexer,
                                        double velocityRPM, double hoodAngleDegrees) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                shooter.setTargetVelocity(velocityRPM);
                shooter.setTargetHoodPose(hoodAngleDegrees);
                shooter.prepareToShoot();
            }, shooter),
            Commands.waitUntil(shooter::isReady).withTimeout(3.0)
            // IndexerCommands.feedTimed(indexer, 0.5), // FIXME to use the IndexerSubsystem's feed method instead of a command from IndexerCommands
            // Commands.runOnce(shooter::returnToStandby, shooter) // TODO: Do not use right now **EXPERIMENTAL**
        ).withName("ShootSequence");
    }

    /**
     * Creates a vision-based shoot sequence with indexer coordination.
     *
     * @param shooter The shooter subsystem
     * @param vision The vision subsystem
     * @param indexer The indexer subsystem
     * @return Vision-based shooting sequence
     */
    public static Command visionShootSequence(ShooterSubsystem shooter, VisionSubsystem vision,
                                              IndexerSubsystem indexer) {
        return Commands.sequence(
            Commands.waitUntil(vision::hasTarget).withTimeout(1.0),
            visionShot(shooter, vision)
            // IndexerCommands.feedTimed(indexer, 0.5), // FIXME to use the IndexerSubsystem's feed method instead of a command from IndexerCommands
            // Commands.runOnce(shooter::returnToStandby, shooter)
        ).withName("VisionShootSequence");
    }

    // =========================================================================
    // VISION ALIGN AND SHOOT (primary match command)
    // =========================================================================

    /**
     * Combined vision-targeting and shooting command. Intended as the primary RT binding.
     *
     * Behavior (all while trigger held):
     *   1. Arms the current POV-selected preset immediately as a fallback baseline.
     *   2. Continuously looks for the nearest hub AprilTag (alliance-aware).
     *   3. If a hub tag is visible (or in grace period): updates flywheel RPM and
     *      hood position from the distance interpolation table every cycle.
     *   4. Overrides drivetrain rotation with a P-controller on tx (horizontal angle),
     *      while still allowing the driver to translate freely with the left stick.
     *   5. Fires the indexer and conveyor as soon as vision reports aligned AND
     *      shooter reports ready.  Stops feeding if either condition is lost.
     *   6. On trigger release: stops indexer, conveyor, and shooter.
     *
     * Subsystem requirements: shooter, vision, indexer, drivetrain
     *   → interrupts the default drive command for the duration of the trigger hold.
     *
     * Tuning handles:
     *   - Constants.Vision.ROTATIONAL_KP            (rotation aggressiveness)
     *   - Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC  (rotation clamp)
     *   - ShooterSubsystem FLYWHEEL_RPM_MAP / HOOD_ROT_MAP     (shooter params)
     *   See TUNING.md for the step-by-step procedure.
     *
     * @param shooter     Shooter subsystem
     * @param vision      Vision subsystem
     * @param indexer     Indexer subsystem
     * @param drivetrain  Swerve drivetrain
     * @param xSupplier   Driver left-Y velocity in m/s (already scaled by MaxSpeed)
     * @param ySupplier   Driver left-X velocity in m/s (already scaled by MaxSpeed)
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
            shooter.armSelectedPreset(); // Spin up to the POV-selected preset immediately
            shooter.prepareToShoot();    // Enter READY state — don't wait for alignment
        }, shooter))
        .finallyDo(() -> {
            indexer.indexerStop();
            indexer.conveyorStop();
            shooter.setIdle();
        })
        .withName("VisionAlignAndShoot");
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

    /**
     * Creates a command to return shooter to SPINUP at STANDBY_RPM.
     *
     * @param shooter The shooter subsystem
     * @return Command that returns shooter to standby spinning state
     */

     /** TODO: Do not use right now _EXPERIMENTAL_ */
    // public static Command returnToStandby(ShooterSubsystem shooter) {
    //     return Commands.runOnce(shooter::returnToStandby, shooter)
    //         .withName("ReturnToStandby");
    // }

    /* 
    * Run IndexerSubsystem convevor forward to assist with feeding into the hopper
    * Run IndexerSubsystem indexer forward to assist with feeding into the hopper 
    * Set ShooterSubsystem to popper preset (low RPM + hood at minimum pose) to assist with loading fuel into the hopper
     */

    // Run IndexerSubsystem conveyorforward, IndexerSubsystem indexer, and ShooterSubsystem flywheel in parallel */
public static Command runAirPopper(IndexerSubsystem indexer, ShooterSubsystem shooter) {
    return Commands.parallel(
        Commands.run(() -> {
            indexer.conveyorAirPopper();
            indexer.indexerAirPopper();
        }, indexer),                                    // indexer required ONCE
        Commands.run(shooter::shooterAirPopper, shooter)
    ).withName("RunAirPopper");                         // .withName() on the outer command
}
}