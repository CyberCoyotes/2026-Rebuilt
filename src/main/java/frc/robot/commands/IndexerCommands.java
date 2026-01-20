package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IndexerSubsystem;

/**
 * IndexerCommands - Factory for indexer-related commands.
 *
 * This class provides static factory methods to create common indexer commands.
 * Using a factory pattern keeps command creation centralized and reusable.
 *
 * Pattern used by: FRC 254, 1678, 6328, and other top teams
 *
 * @author @Isaak3
 */
public class IndexerCommands {

    /**
     * Creates a command to transport game piece from floor to holding position.
     * Finishes when game piece is detected.
     *
     * @param indexer The indexer subsystem
     * @return Command that runs floor transport until game piece detected
     */
    public static Command transport(IndexerSubsystem indexer) {
        return Commands.sequence(
            Commands.runOnce(indexer::startTransport, indexer),
            Commands.waitUntil(indexer::hasGamePiece)
        ).withTimeout(3.0)  // Safety timeout
         .withName("TransportGamePiece");
    }

    /**
     * Creates a command to feed game piece to shooter.
     * Runs until interrupted (typically by shooter command).
     *
     * @param indexer The indexer subsystem
     * @return Command that starts feeding
     */
    public static Command feed(IndexerSubsystem indexer) {
        return Commands.runOnce(indexer::startFeeding, indexer)
            .andThen(Commands.idle(indexer))
            .withName("FeedToShooter");
    }

    /**
     * Creates a command to feed for a specific duration.
     * Useful for ensuring game piece fully exits indexer.
     *
     * @param indexer The indexer subsystem
     * @param durationSeconds How long to feed
     * @return Command that feeds then returns to idle
     */
    public static Command feedTimed(IndexerSubsystem indexer, double durationSeconds) {
        return Commands.sequence(
            Commands.runOnce(indexer::startFeeding, indexer),
            Commands.waitSeconds(durationSeconds),
            Commands.runOnce(indexer::stop, indexer)
        ).withName("FeedTimed");
    }

    /**
     * Creates a command to eject game piece (reverse all motors).
     * Runs until interrupted.
     *
     * @param indexer The indexer subsystem
     * @return Command that starts ejecting
     */
    public static Command eject(IndexerSubsystem indexer) {
        return Commands.runOnce(indexer::startEject, indexer)
            .andThen(Commands.idle(indexer))
            .withName("EjectGamePiece");
    }

    /**
     * Creates a command to eject for a specific duration.
     *
     * @param indexer The indexer subsystem
     * @param durationSeconds How long to eject
     * @return Command that ejects then returns to idle
     */
    public static Command ejectTimed(IndexerSubsystem indexer, double durationSeconds) {
        return Commands.sequence(
            Commands.runOnce(indexer::startEject, indexer),
            Commands.waitSeconds(durationSeconds),
            Commands.runOnce(indexer::stop, indexer)
        ).withName("EjectTimed");
    }

    /**
     * Creates a command to wait until indexer has a game piece.
     * Useful for sequencing with intake.
     *
     * @param indexer The indexer subsystem
     * @return Command that waits until game piece detected
     */
    public static Command waitForGamePiece(IndexerSubsystem indexer) {
        return Commands.waitUntil(indexer::hasGamePiece)
            .withTimeout(5.0)  // Safety timeout
            .withName("WaitForGamePiece");
    }

    /**
     * Creates a command to return indexer to idle state.
     * Stops all motors.
     *
     * @param indexer The indexer subsystem
     * @return Command that sets indexer to idle
     */
    public static Command idle(IndexerSubsystem indexer) {
        return Commands.runOnce(indexer::stop, indexer)
            .withName("IndexerIdle");
    }

    /**
     * Creates a command to intake from floor with automatic state management.
     * Coordinates with intake subsystem to collect game piece.
     *
     * @param indexer The indexer subsystem
     * @return Command that transports from floor and waits for game piece
     */
    public static Command intakeFromFloor(IndexerSubsystem indexer) {
        return Commands.sequence(
            // Start floor transport
            Commands.runOnce(indexer::startTransport, indexer),

            // Wait until game piece detected or timeout
            Commands.either(
                // Game piece detected - success
                Commands.runOnce(indexer::stop, indexer),

                // Timeout - still idle
                Commands.runOnce(indexer::stop, indexer),

                indexer::hasGamePiece
            ).withTimeout(3.0)
        ).withName("IntakeFromFloor");
    }

    /**
     * Creates a command that holds the current state.
     * Useful for keeping indexer running during a sequence.
     *
     * @param indexer The indexer subsystem
     * @return Command that does nothing (holds current state)
     */
    public static Command hold(IndexerSubsystem indexer) {
        return Commands.idle(indexer)
            .withName("HoldIndexerState");
    }

    // Private constructor to prevent instantiation
    private IndexerCommands() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
