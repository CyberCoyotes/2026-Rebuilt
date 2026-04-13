package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Individual motor direction tests for verifying each motor spins the correct
 * way BEFORE locking them into a lead-follow relationship during a match.
 *
 * Each test runs a single motor at a safe voltage while held (startEnd pattern).
 * Release the button and the motor stops. Follower tests automatically re-lock
 * the follower on release.
 *
 * Usage in RobotContainer (bind to operator buttons):
 * <pre>
 *   operatorController.a().whileTrue(MotorDirectionTests.flywheelLeader(shooter));
 *   operatorController.b().whileTrue(MotorDirectionTests.flywheelFollower(shooter));
 *   operatorController.x().whileTrue(MotorDirectionTests.kickerLeader(indexer));
 *   operatorController.y().whileTrue(MotorDirectionTests.kickerFollower(indexer));
 * </pre>
 *
 * Or run testAllSequence() to cycle through each motor automatically with a
 * hold time between each.
 */
public final class MotorDirectionTests {

    /** Safe test voltage — enough to see direction, not enough to launch anything. */
    private static final double TEST_VOLTS = 3.0;

    /** How long each motor runs during the automatic sequence. */
    private static final double HOLD_SECONDS = 2.0;

    /** Pause between motors so the operator can observe. */
    private static final double PAUSE_SECONDS = 1.0;

    private MotorDirectionTests() {}

    // =====================================================================
    // Individual hold-to-run tests (bind to buttons)
    // =====================================================================

    /** Hold to spin flywheel leader only. Follower mirrors since link is intact. */
    public static Command flywheelLeader(ShooterSubsystem shooter) {
        return shooter.testFlywheelLeader(TEST_VOLTS);
    }

    /** Hold to spin flywheel follower only. Leader stays still. Re-locks on release. */
    public static Command flywheelFollower(ShooterSubsystem shooter) {
        return shooter.testFlywheelFollower(TEST_VOLTS);
    }

    /** Hold to spin kicker leader only. Follower mirrors since link is intact. */
    public static Command kickerLeader(IndexerSubsystem indexer) {
        return indexer.testKickerLeader(TEST_VOLTS);
    }

    /** Hold to spin kicker follower only. Leader stays still. Re-locks on release. */
    public static Command kickerFollower(IndexerSubsystem indexer) {
        return indexer.testKickerFollower(TEST_VOLTS);
    }

    // =====================================================================
    // Automatic sequence (runs all four, one at a time)
    // =====================================================================

    /**
     * Runs each motor individually for HOLD_SECONDS, pausing between each.
     * Order: flywheel leader → flywheel follower → kicker leader → kicker follower.
     * Bind to a dashboard button or a spare controller button for pit testing.
     */
    public static Command testAllSequence(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return Commands.sequence(
            timedTest(shooter.testFlywheelLeader(TEST_VOLTS), "Flywheel Leader"),
            Commands.waitSeconds(PAUSE_SECONDS),
            timedTest(shooter.testFlywheelFollower(TEST_VOLTS), "Flywheel Follower"),
            Commands.waitSeconds(PAUSE_SECONDS),
            timedTest(indexer.testKickerLeader(TEST_VOLTS), "Kicker Leader"),
            Commands.waitSeconds(PAUSE_SECONDS),
            timedTest(indexer.testKickerFollower(TEST_VOLTS), "Kicker Follower")
        ).withName("MotorDirectionTest: All");
    }

    private static Command timedTest(Command testCommand, String label) {
        return testCommand
            .withTimeout(HOLD_SECONDS)
            .withName("MotorDirectionTest: " + label);
    }
}
