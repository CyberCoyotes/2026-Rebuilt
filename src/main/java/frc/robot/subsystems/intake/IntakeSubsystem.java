package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeSubsystem extends SubsystemBase {

    // ── IO Layer ───────────────────────────────────────────────────────────────
    private final IntakeIO io;
    private final IntakeIOInputs inputs = new IntakeIOInputs();

    // ── Constants ──────────────────────────────────────────────────────────────
    static final double SLIDE_RETRACTED_POSITION = 0.0;
    static final double SLIDE_EXTENDED_POSITION  = 1.85;

    // Voltage to run roller during intake — 4V was not enough
    static final double ROLLER_INTAKE_VOLTS = 6.0;

    // Current threshold for jam detection — used when sensor is re-enabled
    static final double JAM_CURRENT_THRESHOLD = 60.0;

    // Distance threshold for game piece detection (mm, ~4 inches)
    static final int INTAKE_THRESHOLD = 1000;

    public IntakeSubsystem(IntakeIO intakeIO) {
        this.io = intakeIO;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // Logger.processInputs("Intake", inputs); // FIXME: enable when AdvantageKit wired up
    }

    // ── Low-Level Hardware Methods ─────────────────────────────────────────────
    // These wrap io calls so commands and higher-level methods stay readable.

    public void runRoller() {
        io.setRollerVoltage(ROLLER_INTAKE_VOLTS);
    }

    public void reverseRoller() {
        io.setRollerVoltage(-ROLLER_INTAKE_VOLTS);
    }

    public void stopRoller() {
        io.stopRoller();
    }

    public void extendSlides() {
        io.setSlidePosition(SLIDE_EXTENDED_POSITION);
    }

    public void retractSlides() {
        io.setSlidePosition(SLIDE_RETRACTED_POSITION);
    }

    // ── Slide State Queries ────────────────────────────────────────────────────
    public double getSlidePosition() {
        return inputs.slidePositionRotations;
    }

    public boolean isSlideExtended() {
        return Math.abs(inputs.slidePositionRotations - SLIDE_EXTENDED_POSITION) < 0.05;
    }

    public boolean isSlideRetracted() {
        return Math.abs(inputs.slidePositionRotations - SLIDE_RETRACTED_POSITION) < 0.05;
    }

    // ── Sensor Queries (re-enable when hardware is connected) ──────────────────
    // public boolean hasGamePiece() { return inputs.hasGamePiece; }
    // public boolean isJammed()     { return inputs.rollerCurrentAmps > JAM_CURRENT_THRESHOLD; }

    // ── Commands ───────────────────────────────────────────────────────────────

    /** Extends slides and runs roller while held, stops roller on release. Slides stay extended. */
    public Command intakeFuel() {
        return Commands.startEnd(
            () -> {
                extendSlides();
                runRoller();
            },
            this::stopRoller,
            this
        );
    }

    /** Reverses roller while held, stops on release. */
    public Command ejectFuel() {
        return Commands.startEnd(
            this::reverseRoller,
            this::stopRoller,
            this
        );
    }

    /** Stops roller and retracts slides. Instant command. */
    public Command stowIntake() {
        return Commands.runOnce(
            () -> {
                stopRoller();
                retractSlides();
            },
            this
        );
    }
}