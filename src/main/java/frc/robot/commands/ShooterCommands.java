package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * ShooterCommands - Factory for shooter-related commands.
 *
 * This class provides static factory methods to create common shooter commands.
 * Using a factory pattern keeps command creation centralized and reusable.
 *
 * Pattern used by: FRC 254, 1678, 6328, and other top teams
 *
 * @author @Isaak3
 */
public class ShooterCommands {

    /** Percent tolerance for starting feed during ramp test (5%). */
    private static final double RAMP_TEST_FEED_TOLERANCE = 0.05;


    /**
     * Basic ramp-test shooter command.
     *
     * Behavior:
     *  - Commands the flywheel to ShooterSubsystem.RAMP_TEST_TARGET_RPM
     *  - Once flywheel is within target RPM tolerance, starts feeding (conveyor + indexer)
     *  - On end/cancel, stops feeding and returns shooter to IDLE
     *
     * Intended for use with a whileTrue() button binding.
     */
    public static Command rampTestShoot(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        final double targetRPM = ShooterSubsystem.RAMP_TEST_TARGET_RPM;

        return Commands.sequence(
            // Spin up to the ramp-test target
            Commands.runOnce(() -> {
                shooter.setTargetHoodPose(4.00); // TODO Test this value
                shooter.setTargetVelocity(3200); // TODO Test this value
                shooter.prepareToShoot(); // state machine
            }, shooter),

            // Wait until flywheel is within 15% of the target RPM
            Commands.waitUntil(() ->
                Math.abs(shooter.getCurrentVelocityRPM() - targetRPM)
                    <= Math.abs(targetRPM) * RAMP_TEST_FEED_TOLERANCE
            ),
            Commands.run(() -> {
                indexer.indexerForward();
                indexer.conveyorForward();
            }, indexer)
        )
        .finallyDo(() -> {
            indexer.indexerStop();
            indexer.conveyorStop();
            shooter.setIdle();
        });
    }

    /**
     * Creates a command to pre-rev the shooter flywheel (SPINUP state).
     * This gets the flywheel spinning at 20% max velocity to reduce spin-up time.
     *
     * @param shooter The shooter subsystem
     * @return Command that enters SPINUP state
     */
    public static Command spinUp(ShooterSubsystem shooter) {
        return Commands.runOnce(shooter::spinup, shooter)
            .withName("SpinUpShooter");
    }

    /**
     * Creates a command to prepare shooter for shooting with specific targets.
     * Sets targets and transitions to READY state.
     *
     * @param shooter The shooter subsystem
     * @param velocityRPM Target flywheel velocity
     * @param hoodPoseDegrees Target hood pose
     * @return Command that prepares shooter and waits until ready
     */
    public static Command prepareToShoot(ShooterSubsystem shooter, double velocityRPM, double hoodPoseDegrees) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                shooter.setTargetVelocity(velocityRPM);
                shooter.setTargetHoodPose(hoodPoseDegrees);
                shooter.prepareToShoot();
            }, shooter),
            Commands.waitUntil(shooter::isReady)
        ).withTimeout(3.0)  // Safety timeout
         .withName("PrepareToShoot");
    }

    /**
     * Creates a command to shoot using preset close shot.
     * Finishes when shooter is ready.
     *
     * @param shooter The shooter subsystem
     * @return Command that configures for close shot
     */
    public static Command closeShot(ShooterSubsystem shooter) {
        return Commands.sequence(
            Commands.runOnce(shooter::closeShot, shooter),
            Commands.waitUntil(shooter::isReady)
        ).withTimeout(3.0)
         .withName("CloseShot");
    }

    /**
     * Creates a command to using preset close shot.
     * Finishes when shooter is ready.
     *
     * @param shooter The shooter subsystem
     * @return Command that configures for close shot
     */
    public static Command closeShotPreset(ShooterSubsystem shooter) {
        return Commands.runOnce(shooter::closeShot, shooter)
                .withName("CloseShotPreset");
    }


    /**
     * Creates a command to shoot using preset far shot.
     * Finishes when shooter is ready.
     *
     * @param shooter The shooter subsystem
     * @return Command that configures for far shot
     */
    public static Command farShot(ShooterSubsystem shooter) {
        return Commands.sequence(
            Commands.runOnce(shooter::farShot, shooter),
            Commands.waitUntil(shooter::isReady)
        ).withTimeout(3.0)
         .withName("FarShot");
    }

    
    /**
     * Creates a command to using preset far shot.
     * Finishes when shooter is ready.
     *
     * @param shooter The shooter subsystem
     * @return Command that configures for far shot
     */
    public static Command farShotPreset(ShooterSubsystem shooter) {
        return Commands.runOnce(shooter::farShot, shooter)
         .withName("FarShotPreset");
    }

    /**
     * Creates a command to pass to another robot (PASS state).
     * Sets 50% max velocity and MAX_POSE.
     * Finishes when shooter is ready.
     *
     * @param shooter The shooter subsystem
     * @return Command that configures for passing
     */
    public static Command pass(ShooterSubsystem shooter) {
        return Commands.sequence(
            Commands.runOnce(shooter::pass, shooter),
            Commands.waitUntil(shooter::isPassReady)
        ).withTimeout(3.0)
         .withName("Pass");
    }

    /**
     * Creates a command to shoot using vision-based distance calculation.
     * Updates shooter parameters based on current distance to target.
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
     * Runs until interrupted - useful while moving.
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
     * Prepares shooter to target velocity/angle, waits until ready, feeds game piece, then idles.
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
            // Prepare shooter and wait until at target
            prepareToShoot(shooter, velocityRPM, hoodAngleDegrees),

            // Feed game piece
            // IndexerCommands.feedTimed(indexer, 0.5), // FIXME this is broken, it should reference the subsystem 


            // Return to idle
            Commands.runOnce(shooter::setIdle, shooter)
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
            // Wait for valid target
            Commands.waitUntil(vision::hasTarget).withTimeout(1.0),

            // Spin up based on vision
            visionShot(shooter, vision),

            // Feed game piece
            IndexerCommands.feedTimed(indexer, 0.5),

            // Return to idle
            Commands.runOnce(shooter::setIdle, shooter)
        ).withName("VisionShootSequence");
    }

    /**
     * Creates a command to increase target flywheel RPM by the provided increment.
     *
     * @param shooter The shooter subsystem
     * @param incrementRPM RPM delta to apply (positive increases)
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
     * @param decrementRPM RPM delta to apply (positive value expected)
     * @return Command that adjusts target flywheel velocity
     */
    public static Command decreaseTargetVelocity(ShooterSubsystem shooter, double decrementRPM) {
        return Commands.runOnce(() -> shooter.adjustTargetVelocity(-Math.abs(decrementRPM)), shooter)
            .withName("DecreaseShooterTargetVelocity");
    }

    /**
     * Creates a command to return shooter to idle state.
     * Stops motors and returns hood to home position.
     *
     * @param shooter The shooter subsystem
     * @return Command that sets shooter to idle
     */
    public static Command idle(ShooterSubsystem shooter) {
        return Commands.runOnce(shooter::setIdle, shooter)
            .withName("ShooterIdle");
    }

    /**
     * Creates a command to eject jammed game pieces.
     * Reverses flywheel for a duration then returns to idle.
     *
     * @param shooter The shooter subsystem
     * @param durationSeconds How long to eject
     * @return Command that ejects then idles
     */
    public static Command eject(ShooterSubsystem shooter, double durationSeconds) {
        return Commands.sequence(
            Commands.runOnce(shooter::eject, shooter),
            Commands.waitSeconds(durationSeconds),
            Commands.runOnce(shooter::setIdle, shooter)
        ).withName("EjectShooter");
    }

    /**
     * Creates a command that waits until shooter is ready.
     * Useful for parallel command groups.
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
     * Creates a command to warm up shooter (SPINUP state).
     * Pre-revs flywheel at 20% max for quick response when needed.
     *
     * @param shooter The shooter subsystem
     * @return Command that enters SPINUP state
     */
    public static Command warmUp(ShooterSubsystem shooter) {
        return Commands.runOnce(shooter::spinup, shooter)
            .withName("WarmUpShooter");
    }

    /**
     * Creates a command that ramps the flywheel up to a target RPM for belt-slip testing.
     * The ramp rate is controlled by ClosedLoopRamps in TalonFXConfigs (currently 4 seconds).
     * Hold the button to keep the flywheel spinning; release to idle.
     *
     * @param shooter The shooter subsystem
     * @param targetRPM Target flywheel velocity
     * @return Command that ramps flywheel and idles on release
     */
    public static Command rampUpFlywheel(ShooterSubsystem shooter, double targetRPM) {
        return shooter.flywheelRampTest(targetRPM);
    }

    // Private constructor to prevent instantiation
    private ShooterCommands() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
