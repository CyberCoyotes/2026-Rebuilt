package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * SuperstructureCommands — Cross-subsystem command compositions.
 *
 * Coordinates the shooter and indexer together. Single-subsystem primitives
 * (closeShot, farShot, feed, etc.) live in their own subsystems. This class
 * only handles sequences/groups that span multiple subsystems.
 *
 * Pattern: Commands.deadline()
 *   - The shooter command is the deadline (never self-terminates — runs until
 *     the button is released via whileTrue()).
 *   - The indexer waits for the flywheel to reach speed, then feeds continuously.
 *   - Button release → whileTrue() cancels the group → shooter stopAndHome() runs,
 *     indexer stops. No manual cleanup needed.
 *
 * USAGE (in RobotContainer):
 *   operator.rightTrigger().whileTrue(SuperstructureCommands.closeShot(shooter, indexer));
 *   operator.leftTrigger().whileTrue(SuperstructureCommands.longShot(shooter, indexer));
 *   operator.rightBumper().whileTrue(SuperstructureCommands.trenchShot(shooter, indexer));
 */
public final class SuperCommands {

    // How long to wait for the flywheel to reach speed before feeding anyway.
    // Prevents getting stuck forever if the flywheel never hits its target.
    private static final double READY_TIMEOUT_SECONDS = 3.6; // TODO: Tune timeout per game situation

    // Private constructor — static utility class, never instantiated.
    private SuperCommands() {}

    // =========================================================================
    // SHOT COMMANDS
    // =========================================================================

    /**
     * Close shot: spins up to close preset, waits for flywheel ready, feeds continuously.
     *
     * Use with whileTrue() — flywheel and indexer stop automatically on button release.
     *
     * @param shooter ShooterSubsystem
     * @param indexer IndexerSubsystem
     */
    public static Command closeShot(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return Commands.deadline(
            shooter.closeShot(),                                          // flywheel + hood (deadline)
            Commands.sequence(
                Commands.waitUntil(shooter::isReady)                      // wait for flywheel
                        .withTimeout(READY_TIMEOUT_SECONDS),
                indexer.feed()                                            // feed until cancelled
            )
        ).withName("Superstructure.CloseShot");
    }

    /**
     * Long shot: spins up to far preset, waits for flywheel ready, feeds continuously.
     *
     * Use with whileTrue() — flywheel and indexer stop automatically on button release.
     *
     * @param shooter ShooterSubsystem
     * @param indexer IndexerSubsystem
     */
    public static Command towerShot(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return Commands.deadline(
            shooter.towerShot(),                                            // flywheel + hood (deadline)
            Commands.sequence(
                Commands.waitUntil(shooter::isReady)                      // wait for flywheel
                        .withTimeout(READY_TIMEOUT_SECONDS),
                indexer.feed()                                            // feed until cancelled
            )
        ).withName("Superstructure.LongShot");
    }

    /**
     * Trench shot: spins up to trench preset, waits for flywheel ready, feeds continuously.
     * Tune ShooterSubsystem.TRENCH_SHOT_RPM and TRENCH_SHOT_HOOD for your robot.
     *
     * Use with whileTrue() — flywheel and indexer stop automatically on button release.
     *
     * @param shooter ShooterSubsystem
     * @param indexer IndexerSubsystem
     */
    public static Command trenchShot(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return Commands.deadline(
            shooter.trenchShot(),                                         // flywheel + hood (deadline)
            Commands.sequence(
                Commands.waitUntil(shooter::isReady)                      // wait for flywheel
                        .withTimeout(READY_TIMEOUT_SECONDS),
                indexer.feed()                                            // feed until cancelled
            )
        ).withName("Superstructure.TrenchShot");
    }

        public static Command shoot3603(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return Commands.deadline(
            shooter.shoot3603(),                                         // flywheel + hood (deadline)
            Commands.sequence(
                Commands.waitUntil(shooter::isReady)                      // wait for flywheel
                        .withTimeout(READY_TIMEOUT_SECONDS),
                indexer.feed()                                            // feed until cancelled
            )
        ).withName("Superstructure.Shoot3603");
    }
}
