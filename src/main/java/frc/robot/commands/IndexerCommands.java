package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.IndexerSubsystem;

/**
 * IndexerCommands - Factory for indexer-related commands.
 *
 * Provides static factory methods for common indexer operations.
 * Follows the same command factory pattern as ShooterCommands.
 *
 * @author @Isaak3
 */
public class IndexerCommands {

    // ===== Motor Voltages =====
    // These are voltage values sent via VoltageOut for battery-independent behavior
    private static final double FEED_VOLTS = 9.6;        // Forward voltage to feed pieces to shooter
    private static final double CONVEYOR_VOLTS = 7.2;    // Forward voltage for conveyor belt
    private static final double EJECT_VOLTS = -7.2;      // Reverse voltage to eject jams
    // private static final double INTAKE_VOLTS = 4.8;      // Slower voltage for intaking into hopper

    /**
     * Creates a command that feeds a game piece to the shooter for a set duration.
     * Runs both conveyor and indexer motors forward, then stops.
     *
     * @param indexer The indexer subsystem
     * @param durationSeconds How long to feed (typically 0.5-1.0 seconds)
     * @return Timed feed command
     */
    public static Command feedTimed(IndexerSubsystem indexer, double durationSeconds) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                indexer.setState("FEEDING");
                indexer.setConveyorMotorVolts(CONVEYOR_VOLTS);
                indexer.setIndexerMotorVolts(FEED_VOLTS);
            }, indexer),
            Commands.waitSeconds(durationSeconds),
            Commands.runOnce(() -> {
                indexer.stop();
                indexer.setState("IDLE");
            }, indexer)
        ).withName("FeedTimed");
    }

    /**
     * Creates a command that continuously feeds game pieces to the shooter.
     * Runs until interrupted. Use this with whileTrue() bindings.
     *
     * @param indexer The indexer subsystem
     * @return Continuous feed command (runs until interrupted)
     */
    /*
    public static Command feed(IndexerSubsystem indexer) {
        return Commands.startEnd(
            () -> {
                // indexer.setState("FEEDING");
                indexer.conveyorForward;
                indexer.setIndexerMotorVolts(FEED_VOLTS);
            },
            () -> {
                indexer.stop();
                indexer.setState("IDLE");
            },
            indexer
        ).withName("Feed");
    } 
    */

    /**
     * Creates a command that runs the conveyor to intake game pieces into the hopper.
     * Only runs conveyor (not indexer motor) at a slower speed.
     * Runs until interrupted.
     *
     * @param indexer The indexer subsystem
     * @return Continuous intake command
     */
    // public static Command intake(IndexerSubsystem indexer) {
    //     return Commands.startEnd(
    //         () -> {
    //             indexer.setState("INTAKING");
    //             indexer.setConveyorMotorVolts(INTAKE_VOLTS);
    //         },
    //         () -> {
    //             indexer.stop();
    //             indexer.setState("IDLE");
    //         },
    //         indexer
    //     ).withName("IndexerIntake");
    // }

    /**
     * Creates a command that reverses both motors to eject jammed game pieces.
     * Runs for a set duration then stops.
     *
     * @param indexer The indexer subsystem
     * @param durationSeconds How long to eject (typically 0.5-1.0 seconds)
     * @return Timed eject command
     */
    public static Command eject(IndexerSubsystem indexer, double durationSeconds) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                indexer.setState("EJECTING");
                indexer.setConveyorMotorVolts(EJECT_VOLTS);
                indexer.setIndexerMotorVolts(EJECT_VOLTS);
            }, indexer),
            Commands.waitSeconds(durationSeconds),
            Commands.runOnce(() -> {
                indexer.stop();
                indexer.setState("IDLE");
            }, indexer)
        ).withName("Eject");
    }

    /**
     * Creates a command that immediately stops all indexer motors.
     *
     * @param indexer The indexer subsystem
     * @return Instant stop command
     */
    public static Command stop(IndexerSubsystem indexer) {
        return Commands.runOnce(() -> {
            indexer.stop();
            indexer.setState("IDLE");
        }, indexer).withName("StopIndexer");
    }

    // Private constructor to prevent instantiation
    private IndexerCommands() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
