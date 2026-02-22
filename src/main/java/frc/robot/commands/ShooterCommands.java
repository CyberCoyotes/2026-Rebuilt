package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * **UPDATE Move towards retiring?***
 * ShooterCommands - Factory for shooter-related commands.
 *
 * This class provides static factory methods to create common shooter commands.
 * Using a factory pattern keeps command creation centralized and reusable.
 *
 * SHOT FLOW:
 * 1. Driver presses preset (A/X/B) — silently arms target RPM and hood angle
 * 2. Driver holds shoot trigger — flywheel ramps to target, hood moves, waits until ready, feeds
 * 3. Driver releases trigger — returns to IDLE (TODO: STANDBY once spin logic validated)
 *
 * Pattern used by: FRC 254, 1678, 6328, and other top teams
 *
 * @author @Isaak3
 */
public class ShooterCommands {

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
            // indexer.indexerStop();
            // indexer.conveyorStop();
            shooter.returnToStandby();
        }).withName("ShootAtCurrentTarget");
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
            Commands.waitSeconds(durationSeconds),
            Commands.runOnce(shooter::returnToStandby, shooter)
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
            Commands.waitUntil(shooter::isReady).withTimeout(3.0),
            // IndexerCommands.feedTimed(indexer, 0.5), // FIXME to use the IndexerSubsystem's feed method instead of a command from IndexerCommands
            Commands.runOnce(shooter::returnToStandby, shooter)
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
            visionShot(shooter, vision),
            // IndexerCommands.feedTimed(indexer, 0.5), // FIXME to use the IndexerSubsystem's feed method instead of a command from IndexerCommands
            Commands.runOnce(shooter::returnToStandby, shooter)
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
    public static Command returnToStandby(ShooterSubsystem shooter) {
        return Commands.runOnce(shooter::returnToStandby, shooter)
            .withName("ReturnToStandby");
    }

    /**
     * Creates a command that ramps the flywheel up to a target RPM for testing.
     *
     * @param shooter The shooter subsystem
     * @param targetRPM Target flywheel velocity
     * @return Command that ramps flywheel and returns to idle on release
     */
    public static Command rampUpFlywheel(ShooterSubsystem shooter, double targetRPM) {
        return shooter.flywheelRampTest(targetRPM);
    }

//     // Private constructor to prevent instantiation
//     private ShooterCommands() {
//         throw new UnsupportedOperationException("This is a utility class!");
//     }

}