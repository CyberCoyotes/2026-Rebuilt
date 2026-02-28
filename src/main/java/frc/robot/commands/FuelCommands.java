package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShotPreset;
import frc.robot.subsystems.vision.VisionSubsystem;

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
 * - Driver holds shoot trigger; flywheel ramps to target, hood moves, waits until ready, feeds
 * - Driver releases trigger; returns to IDLE
 */
public class FuelCommands {

    // =========================================================================
    // PRIMARY SHOOT COMMAND
    // =========================================================================

    /**
     * Shoots at whatever preset is currently armed (set by POV cycling).
     *
     * Behavior:
     * - Transitions shooter to READY — flywheel ramps to target RPM, hood moves to target angle
     * - Waits until both flywheel and hood are at target (isReady())
     * - Runs indexer and conveyor to feed the game piece
     * - On button release, stops indexer and returns shooter to IDLE
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
            shooter.setIdle();
        }).withName("ShootAtCurrentTarget");
    }

    // =========================================================================
    // PRESET SHOOT COMMANDS
    // =========================================================================

    /**
     * Full preset shoot sequence that owns both the shooter and indexer subsystems.
     *
     * Flow:
     *   1. Arms the given RPM + hood target silently
     *   2. Calls prepareToShoot() — flywheel ramps up, hood moves to target
     *   3. Waits until both are at target (isReady()), with a 3-second safety timeout
     *   4. Runs indexer and conveyor forward to feed the game piece
     *   5. On trigger release (whileTrue interrupt): stops indexer/conveyor, returns shooter to idle
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
            shooter.setIdle();
        }).withName("ShootWithPreset[" + rpm + "rpm]");
    }

    /**
     * Autonomous-friendly preset shoot command using a named ShotPreset enum value.
     *
     * Flow:
     *   1. Arms the given preset (RPM + hood) silently
     *   2. Calls prepareToShoot() — flywheel ramps up, hood moves to target
     *   3. Waits until both are at target (isReady()), with a 3-second safety timeout
     *   4. Runs indexer and conveyor for {@code feedSeconds} to deliver the game piece
     *   5. Stops indexer/conveyor and returns shooter to idle
     *
     * @param shooter      The shooter subsystem
     * @param indexer      The indexer subsystem
     * @param preset       The named shot preset (TRENCH, CLOSE, TOWER, …)
     * @param feedSeconds  How long to run the indexer/conveyor after ready
     * @param waitForReady Reserved for future use — pass {@code false} for now
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
            }, indexer).withTimeout(feedSeconds)
        ).finallyDo(() -> {
            indexer.indexerStop();
            indexer.conveyorStop();
            shooter.setIdle();
        }).withName("ShootPresetAuton[" + preset.label + "]");
    }

    /**
     * Autonomous: shoot using the TRENCH preset.
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
     * Defaults to the CLOSE preset on robot startup.
     *
     * This is the primary driver RT command.
     *
     * Flow:
     *   1. Arms the selected preset (RPM + hood) silently
     *   2. Calls prepareToShoot() — flywheel ramps up, hood moves to target
     *   3. Waits until both are at target (isReady()), with a 3-second safety timeout
     *   4. Runs indexer and conveyor forward to feed the game piece
     *   5. On trigger release: stops indexer/conveyor, returns to idle
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
            shooter.setIdle();
        }).withName("ShootWithSelectedPreset");
    }

    // =========================================================================
    // SILENT PRESET COMMANDS (POV left/right)
    // =========================================================================

    /**
     * Silently arms close shot targets. No motor movement. Use with onTrue().
     *
     * @param shooter The shooter subsystem
     * @return Command that silently sets close shot targets
     */
    public static Command armCloseShot(ShooterSubsystem shooter) {
        return Commands.runOnce(shooter::setCloseShotPreset, shooter)
            .withName("ArmCloseShot");
    }

    /**
     * Silently arms far shot targets. No motor movement. Use with onTrue().
     *
     * @param shooter The shooter subsystem
     * @return Command that silently sets far shot targets
     */
    public static Command armFarShot(ShooterSubsystem shooter) {
        return Commands.runOnce(shooter::setFarShotPreset, shooter)
            .withName("ArmFarShot");
    }

    /**
     * Silently arms pass shot targets. No motor movement. Use with onTrue().
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
     *
     * @param shooter   The shooter subsystem
     * @param increment Rotations to increase (positive value expected)
     * @return Command that adjusts target hood pose upward
     */
    public static Command increaseTargetHoodPose(ShooterSubsystem shooter, double increment) {
        return Commands.runOnce(() -> shooter.adjustTargetHoodPose(Math.abs(increment)), shooter)
            .withName("IncreaseHoodPose");
    }

    /**
     * Decreases the target hood pose by the provided increment.
     *
     * @param shooter   The shooter subsystem
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
     * Increases target flywheel RPM by the provided increment.
     *
     * @param shooter      The shooter subsystem
     * @param incrementRPM RPM delta to apply
     * @return Command that adjusts target flywheel velocity upward
     */
    public static Command increaseTargetVelocity(ShooterSubsystem shooter, double incrementRPM) {
        return Commands.runOnce(() -> shooter.adjustTargetVelocity(Math.abs(incrementRPM)), shooter)
            .withName("IncreaseShooterTargetVelocity");
    }

    /**
     * Decreases target flywheel RPM by the provided increment.
     *
     * @param shooter      The shooter subsystem
     * @param decrementRPM RPM delta to apply
     * @return Command that adjusts target flywheel velocity downward
     */
    public static Command decreaseTargetVelocity(ShooterSubsystem shooter, double decrementRPM) {
        return Commands.runOnce(() -> shooter.adjustTargetVelocity(-Math.abs(decrementRPM)), shooter)
            .withName("DecreaseShooterTargetVelocity");
    }

    /**
     * Ejects jammed game pieces by reversing the flywheel for a duration.
     *
     * @param shooter         The shooter subsystem
     * @param durationSeconds How long to eject
     * @return Command that ejects then returns to idle
     */
    public static Command eject(ShooterSubsystem shooter, double durationSeconds) {
        return Commands.sequence(
            Commands.runOnce(shooter::eject, shooter),
            Commands.waitSeconds(durationSeconds)
        ).withName("EjectShooter");
    }

    // =========================================================================
    // VISION ALIGN AND SHOOT
    // =========================================================================

    /**
     * Combined vision-targeting and shooting command. Intended as the primary RT binding
     * when vision alignment is desired.
     *
     * Behavior (all while trigger held):
     *   1. Arms the current POV-selected preset immediately as a fallback baseline.
     *   2. Continuously updates flywheel RPM and hood from the distance-to-hub table every cycle.
     *   3. Overrides drivetrain rotation using getAngleToHub() (pose-based, no raw tx needed).
     *   4. Fires the indexer and conveyor as soon as the robot is aligned AND shooter is ready.
     *   5. On trigger release: stops indexer, conveyor, and shooter.
     *
     * Subsystem requirements: shooter, vision, indexer, drivetrain.
     *
     * @param shooter    Shooter subsystem
     * @param vision     Vision subsystem
     * @param indexer    Indexer subsystem
     * @param drivetrain Swerve drivetrain
     * @param xSupplier  Driver left-Y velocity in m/s (already scaled by MaxSpeed)
     * @param ySupplier  Driver left-X velocity in m/s (already scaled by MaxSpeed)
     * @return Vision align and shoot command
     */
    public static Command visionAlignAndShoot(
            ShooterSubsystem shooter,
            VisionSubsystem vision,
            IndexerSubsystem indexer,
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {

        final double maxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

        final SwerveRequest.FieldCentric alignRequest = new SwerveRequest.FieldCentric()
                .withDeadband(maxSpeed * 0.15)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        return Commands.run(() -> {

            // ── 1. Shooter: update from pose-based distance ───────────────────
            double dist = vision.getDistanceToHub();
            if (dist > 0) {
                shooter.updateFromDistance(dist);
            }
            if (shooter.getState() != ShooterSubsystem.ShooterState.READY) {
                shooter.prepareToShoot();
            }

            // ── 2. Drivetrain: driver translation + pose-based rotation ───────
            // getAngleToHub() returns signed degrees; positive = hub is to the left
            double angleErrorDeg = vision.getAngleToHub();
            double rotRate = MathUtil.clamp(
                    angleErrorDeg * Constants.Vision.ROTATIONAL_KP,
                    -Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC,
                     Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC);

            drivetrain.setControl(
                    alignRequest
                            .withVelocityX(xSupplier.getAsDouble())
                            .withVelocityY(ySupplier.getAsDouble())
                            .withRotationalRate(rotRate));

            // ── 3. Feed: only when aligned and shooter is ready ───────────────
            boolean aligned = Math.abs(angleErrorDeg) < Constants.Vision.ALIGNMENT_TOLERANCE_DEGREES;
            if (aligned && shooter.isReady()) {
                indexer.indexerForward();
                indexer.conveyorForward();
            } else {
                indexer.indexerStop();
                indexer.conveyorStop();
            }

        }, shooter, vision, indexer, drivetrain)
        .beforeStarting(Commands.runOnce(() -> {
            shooter.armSelectedPreset();
            shooter.prepareToShoot();
        }, shooter))
        .finallyDo(() -> {
            indexer.indexerStop();
            indexer.conveyorStop();
            shooter.setIdle();
        })
        .withName("VisionAlignAndShoot");
    }

    /**
     * Waits until the shooter reports ready, with a 3-second timeout.
     *
     * @param shooter The shooter subsystem
     * @return Command that waits for shooter ready state
     */
    public static Command waitUntilReady(ShooterSubsystem shooter) {
        return Commands.waitUntil(shooter::isReady)
            .withTimeout(3.0)
            .withName("WaitForShooterReady");
    }

    // =========================================================================
    // AIR POPPER
    // =========================================================================

    /**
     * Runs the conveyor, indexer, and flywheel simultaneously in air-popper mode
     * to assist with loading fuel into the hopper.
     *
     * @param indexer The indexer subsystem
     * @param shooter The shooter subsystem
     * @return Command that runs all three mechanisms in air-popper mode
     */
    public static Command runAirPopper(IndexerSubsystem indexer, ShooterSubsystem shooter) {
        return Commands.parallel(
            Commands.run(() -> {
                indexer.conveyorAirPopper();
                indexer.indexerAirPopper();
            }, indexer),
            Commands.run(shooter::shooterAirPopper, shooter)
        ).withName("RunAirPopper");
    }
}