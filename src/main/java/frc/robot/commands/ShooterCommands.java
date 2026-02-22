package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * ShooterCommands — Factory for shooter-related commands.
 *
 * SHOT FLOW:
 * 1. Driver presses preset (A/B) — silently arms target RPM and hood angle.
 * 2. Driver presses RT — flywheel ramps to target, hood moves, waits until ready, feeds.
 * 3. Driver releases RT — returns to SPINUP at IDLE_RPM.
 *
 * For hub shots, AlignToHubCommand handles everything directly using pose-based
 * distance from VisionSubsystem. No shooter commands are needed for that path.
 */
public class ShooterCommands {

    // =========================================================================
    // PRIMARY SHOOT COMMAND
    // =========================================================================

    /**
     * Shoots at whatever preset is currently armed (A or B).
     *
     * - Transitions shooter to READY — flywheel ramps to target RPM, hood moves
     * - Waits until both flywheel and hood are at target (isReady())
     * - Runs indexer and conveyor to feed the game piece
     * - On release, stops indexer and returns shooter to SPINUP
     *
     * Use with whileTrue() on the shoot trigger.
     */
    public static Command shootAtCurrentTarget(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return Commands.sequence(
            Commands.runOnce(shooter::prepareToShoot, shooter),
            Commands.waitUntil(shooter::isReady).withTimeout(1.0),
            Commands.run(() -> {
                indexer.indexerForward();
                indexer.conveyorForward();
            }, indexer)
        ).finallyDo(() -> {
            indexer.indexerStop();
            indexer.conveyorStop();
            shooter.returnToIdle();
        }).withName("ShootAtCurrentTarget");
    }

    // =========================================================================
    // SILENT PRESET COMMANDS (A / B)
    // =========================================================================

    /**
     * Silently arms close shot targets. No motor movement. Use with onTrue().
     */
    public static Command armCloseShot(ShooterSubsystem shooter) {
        return Commands.runOnce(shooter::setCloseShotPreset, shooter)
            .withName("ArmCloseShot");
    }

    /**
     * Silently arms pass shot targets. No motor movement. Use with onTrue().
     */
    public static Command armPassShot(ShooterSubsystem shooter) {
        return Commands.runOnce(shooter::setPassShotPreset, shooter)
            .withName("ArmPassShot");
    }

    // =========================================================================
    // HOOD ADJUSTMENT
    // =========================================================================

    /**
     * Increases the target hood pose by the given increment.
     * If shooter is already in READY state, hood moves immediately.
     */
    public static Command increaseTargetHoodPose(ShooterSubsystem shooter, double increment) {
        return Commands.runOnce(() -> shooter.adjustTargetHoodPose(Math.abs(increment)), shooter)
            .withName("IncreaseHoodPose");
    }

    /**
     * Decreases the target hood pose by the given increment.
     * If shooter is already in READY state, hood moves immediately.
     */
    public static Command decreaseTargetHoodPose(ShooterSubsystem shooter, double increment) {
        return Commands.runOnce(() -> shooter.adjustTargetHoodPose(-Math.abs(increment)), shooter)
            .withName("DecreaseHoodPose");
    }

    // =========================================================================
    // UTILITY COMMANDS
    // =========================================================================

    /**
     * Increases target flywheel RPM by the given increment.
     */
    public static Command increaseTargetVelocity(ShooterSubsystem shooter, double incrementRPM) {
        return Commands.runOnce(() -> shooter.adjustTargetVelocity(Math.abs(incrementRPM)), shooter)
            .withName("IncreaseShooterTargetVelocity");
    }

    /**
     * Decreases target flywheel RPM by the given increment.
     */
    public static Command decreaseTargetVelocity(ShooterSubsystem shooter, double decrementRPM) {
        return Commands.runOnce(() -> shooter.adjustTargetVelocity(-Math.abs(decrementRPM)), shooter)
            .withName("DecreaseShooterTargetVelocity");
    }

    /**
     * Ejects jammed game pieces by reversing the flywheel, then returns to idle.
     */
    public static Command eject(ShooterSubsystem shooter, double durationSeconds) {
        return Commands.sequence(
            Commands.runOnce(shooter::eject, shooter),
            Commands.waitSeconds(durationSeconds),
            Commands.runOnce(shooter::returnToIdle, shooter)
        ).withName("EjectShooter");
    }

    /**
     * Shoots a timed sequence at a specific RPM and hood angle.
     * Useful for auto routines that need explicit targets.
     */
    public static Command shootSequence(ShooterSubsystem shooter, IndexerSubsystem indexer,
                                        double velocityRPM, double hoodAngle) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                shooter.setTargetVelocity(velocityRPM);
                shooter.setTargetHoodPose(hoodAngle);
                shooter.prepareToShoot();
            }, shooter),
            Commands.waitUntil(shooter::isReady).withTimeout(3.0),
            IndexerCommands.feedTimed(indexer, 0.5),
            Commands.runOnce(shooter::returnToIdle, shooter)
        ).withName("ShootSequence");
    }

    /**
     * Returns shooter to SPINUP at IDLE_RPM.
     */
    public static Command returnToIdle(ShooterSubsystem shooter) {
        return Commands.runOnce(shooter::returnToIdle, shooter)
            .withName("ReturnToIdle");
    }

    /**
     * Ramps the flywheel to a target RPM for testing purposes.
     */
    public static Command rampUpFlywheel(ShooterSubsystem shooter, double targetRPM) {
        return shooter.flywheelRampTest(targetRPM);
    }

    // Private constructor — utility class
    private ShooterCommands() {
        throw new UnsupportedOperationException("Utility class");
    }
}