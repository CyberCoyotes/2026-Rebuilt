package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * AutoCommands - Factory for complete autonomous sequences.
 *
 * This class provides high-level commands that coordinate multiple subsystems.
 * These commands are useful for autonomous routines and complex teleop macros.
 *
 * Pattern used by: FRC 254, 1678, 6328, and other top teams
 *
 * @author @Isaak3
 */
public class AutoCommands {

    /**
     * Creates a complete intake sequence.
     * Coordinates intake and indexer to collect a game piece.
     *
     * @param intake The intake subsystem
     * @param indexer The indexer subsystem
     * @return Command that intakes and transports game piece
     */
    public static Command intakeSequence(IntakeSubsystem intake, IndexerSubsystem indexer) {
        return Commands.parallel(
            // Start intake
            IntakeCommands.startIntaking(intake),

            // Transport through indexer
            Commands.sequence(
                Commands.waitSeconds(0.2),  // Brief delay for intake to start
                IndexerCommands.transport(indexer)
            )
        ).andThen(
            // Retract intake when done
            IntakeCommands.retract(intake)
        ).withTimeout(5.0)  // Safety timeout
         .withName("IntakeSequence");
    }

    /**
     * Creates a complete shoot sequence with vision aiming.
     * Coordinates shooter, vision, and indexer to shoot at target.
     *
     * @param shooter The shooter subsystem
     * @param vision The vision subsystem
     * @param indexer The indexer subsystem
     * @return Command that aims and shoots
     */
    public static Command visionShootSequence(ShooterSubsystem shooter, VisionSubsystem vision,
                                              IndexerSubsystem indexer) {
        return Commands.sequence(
            // Verify we have a game piece
            Commands.either(
                // Has game piece - proceed with shot
                Commands.sequence(
                    // Wait for vision target
                    Commands.waitUntil(vision::hasTarget).withTimeout(1.0),

                    // Spin up shooter with vision
                    ShooterCommands.visionShot(shooter, vision),

                    // Feed and shoot
                    IndexerCommands.feedTimed(indexer, 0.5)
                ),

                // No game piece - do nothing
                Commands.print("No game piece to shoot!"),

                indexer::hasGamePiece
            ),

            // Return to idle
            Commands.parallel(
                ShooterCommands.idle(shooter),
                IndexerCommands.idle(indexer)
            )
        ).withName("VisionShootSequence");
    }

    /**
     * Creates a preset shoot sequence (no vision).
     * Uses close shot preset for quick shooting.
     *
     * @param shooter The shooter subsystem
     * @param indexer The indexer subsystem
     * @return Command that shoots at close range
     */
    public static Command closeShootSequence(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return Commands.sequence(
            // Verify we have a game piece
            Commands.either(
                // Has game piece - shoot
                Commands.sequence(
                    ShooterCommands.closeShot(shooter),
                    IndexerCommands.feedTimed(indexer, 0.5)
                ),

                // No game piece - do nothing
                Commands.print("No game piece to shoot!"),

                indexer::hasGamePiece
            ),

            // Return to idle
            Commands.parallel(
                ShooterCommands.idle(shooter),
                IndexerCommands.idle(indexer)
            )
        ).withName("CloseShootSequence");
    }

    /**
     * Creates an eject sequence for all subsystems.
     * Reverses all mechanisms to clear jams or remove game pieces.
     *
     * @param intake The intake subsystem
     * @param indexer The indexer subsystem
     * @param shooter The shooter subsystem
     * @param durationSeconds How long to eject
     * @return Command that ejects from all subsystems
     */
    public static Command ejectAll(IntakeSubsystem intake, IndexerSubsystem indexer,
                                   ShooterSubsystem shooter, double durationSeconds) {
        return Commands.parallel(
            IntakeCommands.ejectTimed(intake, durationSeconds),
            IndexerCommands.ejectTimed(indexer, durationSeconds),
            ShooterCommands.eject(shooter, durationSeconds)
        ).withName("EjectAll");
    }

    /**
     * Creates an idle sequence for all subsystems.
     * Returns all mechanisms to safe idle state.
     *
     * @param intake The intake subsystem
     * @param indexer The indexer subsystem
     * @param shooter The shooter subsystem
     * @return Command that idles all subsystems
     */
    public static Command idleAll(IntakeSubsystem intake, IndexerSubsystem indexer,
                                  ShooterSubsystem shooter) {
        return Commands.parallel(
            IntakeCommands.retract(intake),
            IndexerCommands.idle(indexer),
            ShooterCommands.idle(shooter)
        ).withName("IdleAll");
    }

    /**
     * Creates a shoot-on-the-move sequence.
     * Continuously tracks target while preparing to shoot.
     *
     * @param shooter The shooter subsystem
     * @param vision The vision subsystem
     * @param indexer The indexer subsystem
     * @return Command that tracks and shoots while moving
     */
    public static Command shootOnMove(ShooterSubsystem shooter, VisionSubsystem vision,
                                      IndexerSubsystem indexer) {
        return Commands.sequence(
            // Start tracking target
            Commands.deadline(
                // Wait for shooter to be ready
                ShooterCommands.waitUntilReady(shooter).withTimeout(2.0),

                // Continuously update based on vision
                ShooterCommands.trackTarget(shooter, vision)
            ),

            // Shoot when ready
            Commands.either(
                IndexerCommands.feedTimed(indexer, 0.5),
                Commands.print("Shooter not ready!"),
                shooter::isReady
            ),

            // Return to idle
            ShooterCommands.idle(shooter)
        ).withName("ShootOnMove");
    }

    /**
     * Creates a warmup sequence for competition.
     * Spins up shooter to low speed and prepares all subsystems.
     *
     * @param shooter The shooter subsystem
     * @return Command that warms up mechanisms
     */
    public static Command warmupSequence(ShooterSubsystem shooter) {
        return Commands.sequence(
            Commands.print("Starting warmup sequence..."),
            ShooterCommands.warmUp(shooter),
            Commands.print("Warmup complete!")
        ).withName("WarmupSequence");
    }

    // Private constructor to prevent instantiation
    private AutoCommands() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
