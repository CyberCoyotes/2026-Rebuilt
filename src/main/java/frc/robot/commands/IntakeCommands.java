package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * IntakeCommands - Factory for intake-related commands.
 *
 * This class provides static factory methods to create common intake commands.
 * Using a factory pattern keeps command creation centralized and reusable.
 *
 * Pattern used by: FRC 254, 1678, 6328, and other top teams
 *
 * @author @Isaak3
 */
public class IntakeCommands {

    /**
     * Creates a command to start intaking (extend and run motors).
     * Runs until interrupted.
     *
     * @param intake The intake subsystem
     * @return Command that extends intake and runs motors forward
     */
    public static Command startIntaking(IntakeSubsystem intake) {
        return Commands.runOnce(intake::startIntaking, intake)
            .andThen(Commands.idle(intake))
            .withName("StartIntaking");
    }

    /**
     * Creates a command to intake and automatically retract when done.
     * Finishes when the intake is fully retracted.
     *
     * @param intake The intake subsystem
     * @param indexerHasGamePiece Supplier that returns true when indexer has a game piece
     * @return Command that intakes until game piece detected, then retracts
     */
    public static Command intakeAndRetract(IntakeSubsystem intake,
                                           java.util.function.BooleanSupplier indexerHasGamePiece) {
        return Commands.sequence(
            // Start intaking
            Commands.runOnce(intake::startIntaking, intake),

            // Wait until game piece detected
            Commands.waitUntil(indexerHasGamePiece),

            // Retract intake
            retract(intake)
        ).withName("IntakeAndRetract");
    }

    /**
     * Creates a command to retract the intake.
     * Finishes when fully retracted.
     *
     * @param intake The intake subsystem
     * @return Command that retracts intake and waits until retracted
     */
    public static Command retract(IntakeSubsystem intake) {
        return Commands.runOnce(intake::retract, intake)
            .andThen(Commands.waitUntil(intake::isRetracted))
            .withTimeout(2.0)  // Safety timeout
            .withName("RetractIntake");
    }

    /**
     * Creates a command to eject game pieces (reverse motors).
     * Runs until interrupted.
     *
     * @param intake The intake subsystem
     * @return Command that extends and reverses motors
     */
    public static Command eject(IntakeSubsystem intake) {
        return Commands.runOnce(intake::eject, intake)
            .andThen(Commands.idle(intake))
            .withName("EjectIntake");
    }

    /**
     * Creates a command to eject for a specific duration then retract.
     *
     * @param intake The intake subsystem
     * @param durationSeconds How long to eject
     * @return Command that ejects then retracts
     */
    public static Command ejectTimed(IntakeSubsystem intake, double durationSeconds) {
        return Commands.sequence(
            Commands.runOnce(intake::eject, intake),
            Commands.waitSeconds(durationSeconds),
            retract(intake)
        ).withName("EjectTimed");
    }

    /**
     * Creates a command to stop all intake motors immediately.
     * Finishes immediately.
     *
     * @param intake The intake subsystem
     * @return Command that stops motors
     */
    public static Command stop(IntakeSubsystem intake) {
        return Commands.runOnce(intake::stopMotors, intake)
            .withName("StopIntake");
    }

    /**
     * Creates a command to zero the slide encoder.
     * Use when intake is manually positioned at retracted position.
     *
     * @param intake The intake subsystem
     * @return Command that zeros slide encoder
     */
    public static Command zeroSlide(IntakeSubsystem intake) {
        return Commands.runOnce(intake::zeroSlide, intake)
            .withName("ZeroIntakeSlide");
    }

    /**
     * Creates a command that automatically retracts if intake is jammed.
     * Monitors jam status and retracts if detected.
     *
     * @param intake The intake subsystem
     * @return Command that monitors for jams and retracts if needed
     */
    public static Command retractIfJammed(IntakeSubsystem intake) {
        return Commands.run(() -> {
            if (intake.isJammed()) {
                intake.retract();
            }
        }, intake)
        .withName("RetractIfJammed");
    }

    // Private constructor to prevent instantiation
    private IntakeCommands() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
