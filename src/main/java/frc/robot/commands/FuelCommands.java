package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Fuel Commands - Factory for shooter-related commands.
 *
 * This class provides static factory methods to create common shooter commands.
 * Using a factory pattern keeps command creation centralized and reusable.
 *
 * SHOT FLOW:
 * 1. Driver presses preset (A/X/B) — silently arms target RPM and hood angle
 * 2. Driver holds shoot trigger — flywheel ramps to target, hood moves, waits until ready, feeds
 * 3. Driver releases trigger — returns to IDLE (TODO: STANDBY once spin logic validated)
 *
 * @author @Isaak3
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
    // AUTO SHOOT COMMANDS (ball-detector terminated — for autonomous routines)
    // =========================================================================
    //
    // These commands use the PseudoBallDetector built into IndexerSubsystem to
    // terminate feeding automatically once no ball activity is detected for a
    // configurable window, rather than running for a fixed duration.
    //
    // HOW IT DIFFERS FROM THE TELEOP VARIANTS:
    //   Teleop commands (.shootWithSelectedPreset / .shootWithPreset) run the
    //   feed phase until the driver releases the trigger (whileTrue interrupt).
    //
    //   Auto commands (.autoShootWithSelectedPreset / .autoShootWithPreset)
    //   self-terminate once the detector reports no ball activity for windowSeconds,
    //   then fall through to the next command in the autonomous sequence.
    //
    // PAIRING WITH A SAFETY TIMEOUT:
    //   Always pair with .withTimeout(maxSeconds) so an empty hopper (no ball events
    //   ever fired) or stuck ball doesn't hang the auto routine indefinitely:
    //
    //     FuelCommands.autoShootWithSelectedPreset(shooter, indexer)
    //         .withTimeout(8.0)
    //
    // DETECTOR SIGNALS (see IndexerSubsystem for full documentation):
    //   - HopperA or HopperB ToF falling edge while FEEDING → ball left sensor zone
    //   - Indexer supply-current spike above threshold while FEEDING → ball in indexer

    /**
     * Autonomous shoot command using the currently selected preset.
     *
     * Identical to shootWithSelectedPreset() except the feed phase self-terminates
     * after no ball activity is detected for windowSeconds. This replaces a fixed
     * feedTimed() duration in auto — the robot moves on as soon as it's actually done,
     * not after an arbitrary wait.
     *
     * Flow:
     *   1. Resets ball detector, arms selected preset, calls prepareToShoot()
     *   2. Waits until shooter is ready (isReady()), safety timeout 3s
     *   3. Runs indexer + conveyor forward until isDoneShootingBalls(windowSeconds)
     *   4. finallyDo: stops indexer + conveyor, returns shooter to IDLE
     *
     * @param shooter       The shooter subsystem
     * @param indexer       The indexer subsystem
     * @param windowSeconds Seconds of inactivity after last ball before declaring done
     *                      (default: IndexerSubsystem.BALL_DETECTOR_WINDOW_SECONDS = 2.0)
     * @return Self-terminating shoot command for use in auto sequences
     */
    public static Command autoShootWithSelectedPreset(ShooterSubsystem shooter,
                                                      IndexerSubsystem indexer,
                                                      double windowSeconds) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                indexer.resetBallDetector();
                shooter.armSelectedPreset();
                shooter.prepareToShoot();
            }, shooter, indexer),
            Commands.waitUntil(shooter::isReady).withTimeout(3.0),
            Commands.run(() -> {
                indexer.indexerForward();
                indexer.conveyorForward();
            }, indexer)
            .until(() -> indexer.isDoneShootingBalls(windowSeconds))
        ).finallyDo(() -> {
            indexer.indexerStop();
            indexer.conveyorStop();
            shooter.setIdle();
        }).withName("AutoShootWithSelectedPreset[" + windowSeconds + "s]");
    }

    /**
     * Overload using the default {@link IndexerSubsystem#BALL_DETECTOR_WINDOW_SECONDS} window (2.0s).
     *
     * @param shooter The shooter subsystem
     * @param indexer The indexer subsystem
     * @return Self-terminating shoot command for use in auto sequences
     */
    public static Command autoShootWithSelectedPreset(ShooterSubsystem shooter,
                                                      IndexerSubsystem indexer) {
        return autoShootWithSelectedPreset(shooter, indexer,
            IndexerSubsystem.BALL_DETECTOR_WINDOW_SECONDS);
    }

    /**
     * Autonomous shoot command for a specific RPM and hood angle.
     *
     * Same adaptive termination logic as autoShootWithSelectedPreset() but with
     * explicit RPM and hood targets — useful for auto routines that set their
     * own preset values rather than cycling through ShotPreset.
     *
     * @param shooter       The shooter subsystem
     * @param indexer       The indexer subsystem
     * @param rpm           Target flywheel velocity in RPM
     * @param hood          Target hood position in rotations
     * @param windowSeconds Seconds of inactivity after last ball before declaring done
     * @return Self-terminating preset shoot command for use in auto sequences
     */
    public static Command autoShootWithPreset(ShooterSubsystem shooter,
                                              IndexerSubsystem indexer,
                                              double rpm, double hood,
                                              double windowSeconds) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                indexer.resetBallDetector();
                shooter.setTargetVelocity(rpm);
                shooter.setTargetHoodPose(hood);
                shooter.prepareToShoot();
            }, shooter, indexer),
            Commands.waitUntil(shooter::isReady).withTimeout(3.0),
            Commands.run(() -> {
                indexer.indexerForward();
                indexer.conveyorForward();
            }, indexer)
            .until(() -> indexer.isDoneShootingBalls(windowSeconds))
        ).finallyDo(() -> {
            indexer.indexerStop();
            indexer.conveyorStop();
            shooter.setIdle();
        }).withName("AutoShootWithPreset[" + rpm + "rpm," + windowSeconds + "s]");
    }

    /**
     * Overload using the default {@link IndexerSubsystem#BALL_DETECTOR_WINDOW_SECONDS} window (2.0s).
     *
     * @param shooter The shooter subsystem
     * @param indexer The indexer subsystem
     * @param rpm     Target flywheel velocity in RPM
     * @param hood    Target hood position in rotations
     * @return Self-terminating preset shoot command for use in auto sequences
     */
    public static Command autoShootWithPreset(ShooterSubsystem shooter,
                                              IndexerSubsystem indexer,
                                              double rpm, double hood) {
        return autoShootWithPreset(shooter, indexer, rpm, hood,
            IndexerSubsystem.BALL_DETECTOR_WINDOW_SECONDS);
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
}