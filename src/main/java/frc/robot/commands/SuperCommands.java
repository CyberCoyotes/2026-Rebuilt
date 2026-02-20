package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * SuperCommands — Cross-subsystem command compositions.
 *
 * Coordinates shooter + indexer together. Single-subsystem commands live in their
 * own subsystems. This class only handles sequences that span multiple subsystems.
 *
 * Pattern: Commands.deadline()
 *   - Shooter command is the deadline (never self-terminates).
 *   - Indexer waits for flywheel to reach speed, then feeds continuously.
 *   - Button release via whileTrue() cancels the group; both subsystems stop cleanly.
 *
 * USAGE (in RobotContainer):
 *   driver.rightTrigger().whileTrue(SuperCommands.closeShot(shooter, indexer));
 *   driver.leftTrigger().whileTrue(SuperCommands.towerShot(shooter, indexer));
 *   driver.rightBumper().whileTrue(SuperCommands.trenchShot(shooter, indexer));
 */
public final class SuperCommands {

    // Max wait before feeding even if the flywheel hasn't hit its target.
    private static final double READY_TIMEOUT_SECONDS = 3.6; // TODO tune per game situation

    private SuperCommands() {}

    // =========================================================================
    // SHOT COMMANDS — use with whileTrue()
    // =========================================================================

    public static Command closeShot(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return shotCommand(shooter.closeShot(), shooter, indexer, "CloseShot");
    }

    public static Command towerShot(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return shotCommand(shooter.towerShot(), shooter, indexer, "TowerShot");
    }

    public static Command trenchShot(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return shotCommand(shooter.trenchShot(), shooter, indexer, "TrenchShot");
    }

    public static Command shoot3603(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return shotCommand(shooter.shoot3603(), shooter, indexer, "Shoot3603");
    }

    // =========================================================================
    // SHARED HELPER
    // =========================================================================

    /**
     * Runs shooterCommand as the deadline; feeds once the shooter reports ready.
     *
     * @param shooterCommand The shooter preset command (deadline — never ends on its own)
     * @param shooter        ShooterSubsystem (for isReady() check)
     * @param indexer        IndexerSubsystem (for feed())
     * @param name           Command name suffix for logging
     */
    private static Command shotCommand(
            Command shooterCommand,
            ShooterSubsystem shooter,
            IndexerSubsystem indexer,
            String name) {
        return Commands.deadline(
            shooterCommand,
            Commands.sequence(
                Commands.waitUntil(shooter::isReady).withTimeout(READY_TIMEOUT_SECONDS),
                indexer.feed()
            )
        ).withName("Superstructure." + name);
    }
}
